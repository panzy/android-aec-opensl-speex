#include "delay_estimator.h"
#include "common.h"
#include <stdlib.h>
#include <sys/time.h>
#include <queue>
#include <stdio.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/errno.h>
#include <math.h>
#include <unistd.h>

#define TAG "aec_est" // log tag

// get timestamp of today in ms.
int64_t delay_estimator::timestamp(int64_t base)
{
  timeval t;
  gettimeofday(&t, NULL);
  return ((int64_t)t.tv_sec * 1000 + (int64_t)(t.tv_usec / 1000)) - base;
}

bool delay_estimator::silent(short *data, int samps)
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

  //D("out frame");
  //for (int i = 0; i < samps; ++i)
  //    D("%d", out[i]);
}

// return: count of |FRAME_SAMPS|
int delay_estimator::search_audio(short *haystack, int haystack_samps, short *needle, int needle_samps, float *quality)
{
  D("search_audio, %d/%d", needle_samps, haystack_samps);

  if (silent(needle, needle_samps)) {
    D("ignore silent needle at frame");
    return -1;
  }

  float best_ratio = 99;
  int best_delay = -1; // in frame
  short *out = new short[needle_samps];
  int pos = 0; // in samps
  while (pos < haystack_samps - needle_samps) {
    float ratio;
    echo_cancel(needle, haystack + pos, needle_samps, out, &ratio);
#if 1
    D("echo cancellation ratio at frame %d: %0.2f", pos / FRAME_SAMPS, ratio);
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
    D("best echo cancellation ratio at frame %d: %0.2f", best_delay, best_ratio);
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

// ctor
delay_estimator::delay_estimator(int sr, int frame_samps, int max_delay, int nearend_frames)
: SR(sr),
  FRAME_SAMPS(frame_samps),
  MAX_DELAY(max_delay),
  MAX_FAR_SAMPS(FRAME_SAMPS * MAX_DELAY),
  MAX_FAR2_SAMPS(MAX_FAR_SAMPS),
  MAX_NEAR_SAMPS(FRAME_SAMPS * nearend_frames),
  MAX_NEAR2_SAMPS(MAX_FAR2_SAMPS),
  far(NULL), far2(NULL),
  near(NULL), near2(NULL),
  far_samps(0), far2_samps(0), far_offset(0),
  near_samps(0), near2_samps(0), near_offset(0),
  best_quality(0),
  best_delay(0),
  last_delay(0),
  comp_times(0),
  processing(false),
  async_hint(0),
  async_result(-1)
{
  far = new short[MAX_FAR_SAMPS];
  near = new short[MAX_NEAR_SAMPS];
  delay_score = new char[MAX_DELAY]; // [delay] => score
  memset(delay_score, 0, MAX_DELAY * sizeof(delay_score[0]));
  pthread_mutex_init(&process_lock, NULL);
}

delay_estimator::~delay_estimator()
{
  delete[] far;
  delete[] near;
  delete[] delay_score;
  pthread_join(process_thrd, NULL);
  pthread_mutex_destroy(&process_lock);
}

int delay_estimator::array_push(short *dst, int dst_len, int dst_capacity,
    short *src, int n, int *dst_offset)
{
  if (n >= dst_capacity) {
    memcpy(dst, src + n - dst_capacity, dst_capacity * 2);
    n = dst_capacity;
  } else {
    // shift
    if (dst_len + n > dst_capacity) {
      int shift = dst_len + n - dst_capacity;
      memmove(dst, dst + shift, (dst_capacity - n) * 2);
      dst_len = dst_capacity - n;
      *dst_offset += shift;
    }
    // push
    memcpy(dst + dst_len, src, n * 2);
  }
  return n;
}

int delay_estimator::add_far(short *data, int n)
{
  pthread_mutex_lock(&process_lock);

  if (processing) {
    // push data to |far2| buffer
    if (far2_samps + n > MAX_FAR2_SAMPS) {
      n = MAX_FAR2_SAMPS - far2_samps;
    }
    memcpy(far2 + far2_samps, data, n * 2);
    far2_samps += n;
  } else {
    // move data from |far2| buffer to |far|, if any
    if (far2_samps > 0 && n < MAX_FAR_SAMPS) {
      far_samps += array_push(far, far_samps, MAX_FAR_SAMPS, far2, far2_samps, &far_offset);
    }
    
    // push |data| to |far| buffer
    far_samps += array_push(far, far_samps, MAX_FAR_SAMPS, data, n, &far_offset);
    if (far_samps > MAX_FAR_SAMPS)
      far_samps = MAX_FAR_SAMPS;
  }

  pthread_mutex_unlock(&process_lock);

  return n;
}

int delay_estimator::add_near(short *data, int n)
{
  pthread_mutex_lock(&process_lock);

  if (processing) {
    // push data to |near2| buffer
    if (near2_samps + n > MAX_NEAR2_SAMPS) {
      n = MAX_NEAR2_SAMPS - near2_samps;
    }
    memcpy(near2 + near2_samps, data, n * 2);
    near2_samps += n;
  } else {
    // move data from |near2| buffer to |near|, if any
    if (near2_samps > 0 && n < MAX_NEAR_SAMPS) {
      near_samps += array_push(near, near_samps, MAX_NEAR_SAMPS, near2, near2_samps, &near_offset);
    }
    
    // push |data| to |near| buffer
    near_samps += array_push(near, near_samps, MAX_NEAR_SAMPS, data, n, &near_offset);
    if (near_samps > MAX_NEAR_SAMPS)
      near_samps = MAX_NEAR_SAMPS;
  }

  pthread_mutex_unlock(&process_lock);

  return n;
}

void *process_proc(void *me)
{
  delay_estimator *p = (delay_estimator*)me;
  p->async_result = p->process(p->async_hint);
}

bool delay_estimator::process_async(int hint)
{
  pthread_mutex_lock(&process_lock);
  if (processing) {
    D("process thread already runs, return false");
    pthread_mutex_unlock(&process_lock);
    return false;
  }
  pthread_mutex_unlock(&process_lock);

  async_hint = hint;
  pthread_create(&process_thrd, NULL, process_proc, this);

  return true;
}

int delay_estimator::process(int hint)
{
  // set |processing| as true
  pthread_mutex_lock(&process_lock);
  processing = true;
  pthread_mutex_unlock(&process_lock);

  int result = -1;
  float quality = 0;

  int64_t t0 = timestamp(0);

  if (near_samps < MAX_NEAR_SAMPS) {
    D("nearend buffer not enough (%d/%d), return", near_samps, MAX_NEAR_SAMPS);
    goto RETURN;
  }

  {
    int d = -1;

    // use |last_delay| as search hint for better efficiency
    if (hint <= 0)
      hint = last_delay;

    // search the |k|-th frame of |far| buffer
    int k = hint + (far_offset - near_offset) / FRAME_SAMPS;

    if (hint <= 0 || k <= 0 || far_samps - k * FRAME_SAMPS < near_samps) {
      // search from the begining of |far| buffer
      d = search_audio(far, far_samps, near, near_samps, &quality);
    } else {
      d = search_audio(far + k * FRAME_SAMPS, far_samps - k * FRAME_SAMPS, near, near_samps, &quality);
      if (d >= 0) {
        d += k;
      } else {
        d = search_audio(far, far_samps - k * FRAME_SAMPS, near, near_samps, &quality);
      }
    }

    result = (near_offset - far_offset) / FRAME_SAMPS - d;
    if (d >= 0 && result >= 0) {
      ++delay_score[result];
      last_delay = result;
      D("estimated delay: (%d~%d,%d+%d)=>%d, quality %0.2f, hit %d",
          near_offset / FRAME_SAMPS, (near_offset + near_samps) / FRAME_SAMPS,
          far_offset / FRAME_SAMPS, d, result, quality, delay_score[result]);
      D("current best result: delay %d, quality %0.2f, hit %d",
          best_delay, best_quality, delay_score[best_delay]);
      if (delay_score[result] > 1 && (result == best_delay || quality > best_quality)) {
        best_quality = quality;
        best_delay = result;
        D("estimated delay, the final result: %d, %0.2f, hit times: %d, compare times: %d",
            result, quality, delay_score[result], comp_times);
      } else {
        result = -1;
      }
    } else {
      result = -1;
    }
  }

RETURN:

  I("process(hint %d) done, result %d, elapse %dms", hint, result, (int)timestamp(t0));

  // set |processing| as false
  pthread_mutex_lock(&process_lock);
  processing = false;
  pthread_mutex_unlock(&process_lock);

  return result;
}
