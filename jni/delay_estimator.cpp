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

#define TAG "aec" // log tag

bool silent(short *data, int samps)
{
  // TODO is this method reliable enough?
  int n0 = 0;
  for (int i = 0; i < samps; ++i) {
    if (abs(data[i]) < 150)
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
  speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 2);

  short *p1 = in, *p2 = ref, *p3 = out;
  int n = samps;
  while (n >= FRAME_SAMPS) {
    speex_echo_cancellation(st, p1, p2, p3);
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
    __android_log_print(ANDROID_LOG_DEBUG, TAG, "ignore silent needle at frame");
    return -1;
  }

  float best_ratio = 99;
  int best_delay = -1; // in frame
  short *out = new short[needle_samps];
  int pos = 0; // in samps
  while (pos < haystack_samps - needle_samps) {
    float ratio;
    echo_cancel(needle, haystack + pos, needle_samps, out, &ratio);
    //__android_log_print(ANDROID_LOG_DEBUG, TAG,
    //    "echo cancellation ratio at frame %d: %0.2f", pos / FRAME_SAMPS, ratio);

    if (ratio > 0 && ratio < 0.99 && ratio < best_ratio) {
      best_ratio = ratio;
      best_delay = pos / FRAME_SAMPS;
    }

    pos += FRAME_SAMPS;
  }

  // dump best result
  if (best_delay >= 0) {
    if (quality) *quality = 1 / best_ratio;
    pos = best_delay * FRAME_SAMPS;
    // reproduce
    echo_cancel(needle, haystack + pos, needle_samps, out, NULL);
    //__android_log_print(ANDROID_LOG_DEBUG, TAG,
    //    "best echo cancellation ratio at frame %d: %0.2f", best_delay, best_ratio);
    
    // dump
#if 0
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

delay_estimator::delay_estimator(int sr, int frame_samps, int max_delay)
: SR(sr),
  FRAME_SAMPS(frame_samps),
  MAX_DELAY(max_delay),
  MAX_FAR_SAMPS(FRAME_SAMPS * MAX_DELAY),
  MAX_NEAR_SAMPS(FRAME_SAMPS * 10),
  far_samps(0), near_samps(0), passed(0),
  best_quality(0),
  best_delay(0)
{
  far = new short[MAX_FAR_SAMPS];
  near = new short[MAX_NEAR_SAMPS];
  delay_score = new char[MAX_DELAY]; // [delay] => score
}

delay_estimator::~delay_estimator()
{
  delete[] far;
  delete[] near;
  delete[] delay_score;
}

int delay_estimator::add_far(short *buf, int n)
{
  if (far_samps + n > MAX_FAR_SAMPS)
    n = MAX_FAR_SAMPS - far_samps;
  memcpy(far + far_samps, buf, n * 2);
  far_samps += n;
  return n;
}

int delay_estimator::process_near(short *frame, float *quality_)
{
  // push new frame to |near|
  if (near_samps == MAX_NEAR_SAMPS) {
    // shift a frame
    memmove(near, near + FRAME_SAMPS, (MAX_NEAR_SAMPS - FRAME_SAMPS) * 2);
    //for (int i = 0; i < MAX_NEAR_SAMPS - FRAME_SAMPS; ++i) near[i] = near[i + FRAME_SAMPS];
    near_samps -= FRAME_SAMPS;
    ++passed;
  }
  memcpy(near + near_samps, frame, FRAME_SAMPS * 2);
  near_samps += FRAME_SAMPS;

  if (near_samps < MAX_NEAR_SAMPS)
    return -1;

  // iterate |near| buffer to search in |far| buffer
  memset(delay_score, 0, sizeof(delay_score) / sizeof(delay_score[0]));
  float quality = 0;
  int result = -1;
  {
    int d = search_audio(far, far_samps, near, near_samps, &quality);
    if (d >= 0) {
      d = passed - d;
      if (quality_) *quality_ = quality;
      __android_log_print(ANDROID_LOG_DEBUG, TAG, "estimated delay: (%d~%d), %0.2f", passed, d, quality);
      if (++delay_score[d] > 3 && quality > best_quality) {
        result = d;
        best_quality = quality;
        best_delay = result;
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "estimated delay, the final result: %d, %0.2f", d, quality);
      }
    }
  }

  return result;
}

