/*
 * vim: shiftwidth=2
 */

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

// delay_estimator::echo_cancel() 中是否总是重新初始化speex？
#define REOPEN_SPEEX 0

const int SEARCH_STEP = 1;

// get timestamp of today in ms.
int64_t delay_estimator::timestamp(int64_t base)
{
  timeval t;
  gettimeofday(&t, NULL);
  return ((int64_t)t.tv_sec * 1000 + (int64_t)(t.tv_usec / 1000)) - base;
}

/**
 * threshold_amp - threshold amplitude, recommanded value: 2500 for farend, 1000
 * for nearend.
 */
bool delay_estimator::silent(const short *data, int samps, short threshold_amp)
{
  for (int i = 0, j = 0, k = samps >> 3; i < samps; ++i) {
    if (abs(data[i]) > threshold_amp && ++j >= k)
      return false;
  }
  return true;
}

void delay_estimator::speex_ec_open(int sampleRate, int bufsize, int totalSize)
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

void delay_estimator::echo_cancel(const short *in, const short *ref, int samps,
        short *out, float *cancellation_ratio)
{
  // XXX 由于我们要查找的delay是精确到单个frame的，speex 的 filter length 为
  // 1xframe即可（1xframe 比 2xframe 快大约 20%）
#if REOPEN_SPEEX 
  speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 1);
#endif

  const short *p1 = in, *p2 = ref;
  short *p3 = out;
  int n = samps;
  while (n >= FRAME_SAMPS) {
    speex_echo_cancellation(st, p1, p2, p3);
    ++comp_times;
    p1 += FRAME_SAMPS;
    p2 += FRAME_SAMPS;
    p3 += FRAME_SAMPS;
    n -= FRAME_SAMPS;
  }

#if REOPEN_SPEEX 
  speex_ec_close();
#endif

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

// ctor
delay_estimator::delay_estimator(int sr, int frame_samps)
: SR(sr),
  FRAME_SAMPS(frame_samps),
  comp_times(0)
{
}

delay_estimator::~delay_estimator()
{
}

int delay_estimator::estimate(const short *far, int far_size,
    const short *near, int near_size,
    int filter_size, float *cancel_ratio) 
{
  *cancel_ratio = 1;
  return estimate_(far, far_size, near, near_size, filter_size,
      -1, cancel_ratio);
}

// @param base |near|相对于第一次调用时的偏移量——这是个递归算法。
//              第一次调用时，请传递 －1。
int delay_estimator::estimate_(const short *far, int far_size,
    const short *near, int near_size,
    int filter_size, int base, float *cancel_ratio) 
{
  // Speex AEC filter length.
  // 决定了搜索的细致程度，或者说最终结果的精度。
  const static int MIN_FILTER_SIZE = 2 * FRAME_SAMPS;

  // 传递给 try_echo_cancel() 的 |near| 的最小长度，
  const static int MIN_NEAR_SIZE = 10 * FRAME_SAMPS;

  // 传递给 try_echo_cancel() 的 |near| 的最大长度，
  const static int MAX_NEAR_SIZE = 100 * FRAME_SAMPS;

  // 决定了搜索范围
  const static int MAX_DELAY_SIZE = 50 * FRAME_SAMPS;

  D("delay_estimator::estimate(far,%d,near,%d,%d,%d)",
      far_size / FRAME_SAMPS,
      near_size / FRAME_SAMPS,
      filter_size / FRAME_SAMPS, base / FRAME_SAMPS);

  // trim silent samples
  {
    // find max amplitude in farend buf
    int max_amp = -1; // frame index
    for (int i = 0; i < far_size - FRAME_SAMPS; ++i) {
      if (i % FRAME_SAMPS == 0 && silent(far + i, FRAME_SAMPS, 2000))
        continue;
      if (max_amp < 0 || abs(far[max_amp]) < abs(far[i])) {
        max_amp = i;
        if (abs(far[i]) > (32768 >> 2))
          break;
      }
    }

    if (max_amp > 0) {
      D("got max_amp: %d at %dms", abs(far[max_amp]), max_amp * 20 / FRAME_SAMPS);
      const short *trim_start = far + max_amp;
      while(trim_start >= far && !silent(trim_start, FRAME_SAMPS, 2000))
        trim_start -= FRAME_SAMPS;
      if (trim_start > far) {
        int j = trim_start - far;
        D("trim %d frames(%dms)", j / FRAME_SAMPS, 20 * j / FRAME_SAMPS);
        far += j;
        far_size -= j;
        near += j;
        near_size -= j;
      }
    }
  }


  // 尝试 |n| 个回声延迟值，相邻两个值的差距是 |step_size|。
  // 
  // 这些值是相对于 |base| 的。
  //
  // 如果这不是递归的第一层，那么我们的尝试范围仅限于上一次所选择的回声延迟值的
  // 范围，恰好是上一次的 |step_size|，也就是本次 |step_size|x2。
  int n = 0;
  int step_size = filter_size / 2;
  if (base < 0) {
    n = std::min(MAX_DELAY_SIZE, near_size - MIN_NEAR_SIZE) / step_size + 1;
  } else {
    n = 2;
  }

  if (n < 1) {
    return base;
  }

  int min_idx = -1;
  for (int i = 0; i < n; ++i) {
    float ratio = try_echo_cancel(
        far,
        far_size,
        near + i * step_size,
        std::min(MAX_NEAR_SIZE, near_size - i * step_size),
        filter_size);
    //D("---- delay %d*%d => %0.2f", step_size / FRAME_SAMPS, i, ratio);
    // 刚刚经过了一个足够好的极点
    if (min_idx >= 0 && ratio > *cancel_ratio && *cancel_ratio < 0.6) {
      break;
    }
    if (ratio < 0.9 && (min_idx < 0 || ratio < *cancel_ratio)) {
      min_idx = i;
      *cancel_ratio = ratio;
    }
  }

  if (min_idx < 0) {
    return base;
  }

  if (base < 0)
    base = 0;

  // 得到了回声延迟值，我们可以就此返回，也可以在它所指示的范围内采用更精细的
  // filter size 来寻找更精确的值。
  if (filter_size / 2 <= MIN_FILTER_SIZE) {
    return min_idx * step_size + base;
  }
  return estimate_(
      far, 
      far_size,
      near + min_idx * step_size,
      near_size - min_idx * step_size,
      filter_size / 2,
      base + min_idx * step_size,
      cancel_ratio);
}

float delay_estimator::try_echo_cancel(
    const short *far, int far_size,
    const short *near, int near_size,
    int filter_size)
{
  float ratio = 1;
  if (rich_nearend(near, near_size)) {
    speex_ec_open(SR, FRAME_SAMPS, filter_size);
    int samps = far_size > near_size ? near_size : far_size;
    short *out = new short[samps];
    echo_cancel(near, far, samps, out, &ratio);
    delete[] out;
    speex_ec_close();
  }

  D("delay_estimator::try_echo_cancel(far,%d,near,%d,%d) => %0.2f",
      far_size / FRAME_SAMPS, near_size / FRAME_SAMPS, filter_size / FRAME_SAMPS, ratio);
  return ratio;
}

bool delay_estimator::rich_nearend(const short *near, int near_size)
{
  // find max amplitude in nearend buf
  short max_amp = 0;
  for (int i = 0; i < near_size; ++i) {
    if (max_amp < near[i]) {
      max_amp = near[i];
    } else if (-max_amp > near[i]) {
      max_amp = -near[i];
    }
  }
  //D("max amplitude in nearend: %d", max_amp);
  if (max_amp < 1000)
    return false;

  // check if there're too many silent frames
  const short *b = near;
  for (int i = 0, n = near_size / FRAME_SAMPS, m = 0;
      i < n;
      ++i, b += FRAME_SAMPS) {
    if (!silent(b, FRAME_SAMPS, max_amp / 4)) {
      if (++m > 10) {
        return true;
      }
    }
  }

  return false;
}

