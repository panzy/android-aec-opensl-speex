#include "delay_estimator.h"
#include <stdlib.h>
#include <android/log.h>
#include <sys/time.h>
#include <queue>
#include <stdio.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/errno.h>
#include <math.h>
#include <unistd.h>

#define TAG "aec_est" // log tag

bool silent(short *data, int samps)
{
  int n0 = 0;
  for (int i = 0; i < samps; ++i) {
    if (abs(data[i]) < 250/* TODO is this method reliable enough?*/)
      ++n0;
  }
  return n0 * 100 / samps > 90;
}

void delay_estimator::speex_ec_open (int sampleRate, int bufsize, int totalSize)
{
  //init
  st = speex_echo_state_init(bufsize, totalSize);
  speex_echo_ctl(st, SPEEX_ECHO_SET_SAMPLING_RATE, &sampleRate);
}

void delay_estimator::speex_ec_close ()
{
  if (st) {
    speex_echo_state_destroy(st);
    st = NULL;
  }
}

void delay_estimator::echo_cancel(short *in, short *ref, int samps,
        short *out, float *cancellation_ratio)
{
  // 由于我们要查找的delay是精确到单个frame的，speex 的 filter length 为 1xframe
  // 即可（1xframe 比 2xframe 快大约 20%）
  speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 1);

  short *p1 = in, *p2 = ref, *p3 = out;
  int n = samps;
  while (n >= FRAME_SAMPS) {
    speex_echo_cancellation(st, p1, p2, p3);
    ++comp_times;
    p1 += FRAME_SAMPS;
    p2 += FRAME_SAMPS;
    p3 += FRAME_SAMPS;
    n -= FRAME_SAMPS;
  }

  speex_ec_close();

  if (cancellation_ratio) {
    int sum1 = 0, sum2 = 0;
    for (int i = 0; i < samps; ++i) {
      sum1 += abs(in[i]);
      sum2 += abs(out[i]);
    }
    *cancellation_ratio = sum1 == 0 ? 0 : (float)sum2 / sum1;
  }

  //__android_log_print(ANDROID_LOG_DEBUG, TAG, "out frame");
  //for (int i = 0; i < samps; ++i)
  //    __android_log_print(ANDROID_LOG_DEBUG, TAG, "%d", out[i]);
}

// return: count of |FRAME_SAMPS|
int delay_estimator::search_audio(short *haystack, int haystack_samps, short *needle, int needle_samps, float *quality)
{
  if (silent(needle, needle_samps)) {
    //__android_log_print(ANDROID_LOG_DEBUG, TAG, "ignore silent needle at frame");
    return -1;
  }

  float best_ratio = 99;
  int best_delay = -1; // in frame
  short *out = new short[needle_samps];
  int pos = 0; // in samps
  while (pos < haystack_samps - needle_samps) {
    float ratio;
    echo_cancel(needle, haystack + pos, needle_samps, out, &ratio);
#if 0
    __android_log_print(ANDROID_LOG_DEBUG, TAG,
        "echo cancellation ratio at frame %d: %0.2f", pos / FRAME_SAMPS, ratio);
#endif

    if (ratio > 0 && ratio < 0.9 && ratio < best_ratio) {
      best_ratio = ratio;
      best_delay = pos / FRAME_SAMPS;
    }

    pos += FRAME_SAMPS;
  }

  // dump best result
  if (best_delay >= 0) {
    if (quality) *quality = 1 / best_ratio;
    pos = best_delay * FRAME_SAMPS;
#if 0
    __android_log_print(ANDROID_LOG_DEBUG, TAG,
        "best echo cancellation ratio at frame %d: %0.2f", best_delay, best_ratio);
#endif
    
#if 0
    // reproduce
    echo_cancel(needle, haystack + pos, needle_samps, out, NULL);
    // dump
    char filename[128];
    sprintf(filename, "sdcard/tmp/out_%d_%0.2f.raw", best_delay, best_ratio);
    FILE *f = fopen(filename, "w+");
    fwrite(out, needle_samps, 2, f);
    fclose(f);
#endif
  } else {
    if (quality) *quality = 0;
  }

  delete[] out;

  return best_delay;
}

delay_estimator::delay_estimator(int sr, int frame_samps, int max_delay, int nearend_frames)
: SR(sr),
  FRAME_SAMPS(frame_samps),
  MAX_DELAY(max_delay),
  MAX_FAR_SAMPS(FRAME_SAMPS * MAX_DELAY),
  MAX_NEAR_SAMPS(FRAME_SAMPS * nearend_frames),
  far_samps(0), far_offset(0),
  near_samps(0), near_offset(0),
  best_quality(0),
  best_delay(0),
  last_delay(0),
  comp_times(0)
{
  far = new short[MAX_FAR_SAMPS];
  near = new short[MAX_NEAR_SAMPS];
  delay_score = new char[MAX_DELAY]; // [delay] => score
  memset(delay_score, 0, MAX_DELAY * sizeof(delay_score[0]));
}

delay_estimator::~delay_estimator()
{
  delete[] far;
  delete[] near;
  delete[] delay_score;
}

int delay_estimator::add_far(short *data, int n)
{
  if (far_samps + n > MAX_FAR_SAMPS) {
    int shift = far_samps + n - MAX_FAR_SAMPS;
    memmove(far, far + shift, (MAX_FAR_SAMPS - n) * 2);
    far_samps = MAX_FAR_SAMPS - n;
    far_offset += shift;
  }
  memcpy(far + far_samps, data, n * 2);
  far_samps += n;
  return n;
}

int delay_estimator::process_near(short *data, int n, int hint, float *quality_)
{
  // push new frame to |near|
  if (near_samps + n > MAX_NEAR_SAMPS) {
    int shift = near_samps + n - MAX_NEAR_SAMPS;
    memmove(near, near + shift, (MAX_NEAR_SAMPS - n) * 2);
    near_samps = MAX_NEAR_SAMPS - n;
    near_offset += shift;
  }
  memcpy(near + near_samps, data, n * 2);
  near_samps += n;

  if (near_samps < MAX_NEAR_SAMPS)
    return -1;

  // iterate |near| buffer to search in |far| buffer
  float quality = 0;
  int result = -1;
  {
    int d = -1;

    // use |last_delay| as search hint for better efficiency
    if (hint <= 0)
      hint = last_delay;

    if (hint <= 0) {
      // search from the begining of |far| buffer
      d = search_audio(far, far_samps, near, near_samps, &quality);
    } else {
      int k = hint >= 2 ? hint - 2 : hint;
      d = search_audio(far + k * FRAME_SAMPS, far_samps - k * FRAME_SAMPS, near, near_samps, &quality);
      if (d >= 0) {
        d += k;
      } else {
        d = search_audio(far, far_samps - k * FRAME_SAMPS, near, near_samps, &quality);
      }
    }

    result = last_delay = (near_offset - far_offset) / FRAME_SAMPS - d;
    if (d >= 0 && result >= 0) {
      if (quality_) *quality_ = quality;
      __android_log_print(ANDROID_LOG_DEBUG, TAG, "estimated delay: (%d~%d,%d+%d)=>%d, %0.2f",
          near_offset / FRAME_SAMPS, (near_offset + near_samps) / FRAME_SAMPS,
          far_offset / FRAME_SAMPS, d, result, quality);
      if (++delay_score[result] > 1 && (result == best_delay || quality > best_quality)) {
        best_quality = quality;
        best_delay = result;
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "estimated delay, the final result: %d, %0.2f, hit times: %d, compare times: %d",
            result, quality, delay_score[result], comp_times);
      } else {
        result = -1;
      }
    } else {
      result = -1;
    }
  }

  return result;
}

