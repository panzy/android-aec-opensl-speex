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
  for (int i = 0, j = 0, k = samps >> 4; i < samps; ++i) {
    if (abs(data[i]) > threshold_amp && ++j > k)
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

// return: count of |FRAME_SAMPS|
int delay_estimator::search_audio(short *haystack, int haystack_samps, short *needle, int needle_samps, float *quality)
{
  //D("search_audio, %d/%d", needle_samps, haystack_samps);

  if (silent(needle, needle_samps, 1200)) {
    D("ignore silent needle at frame");
    return -1;
  }

  float best_ratio = 99;
  int result = -1; // in frame
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
      result = pos / FRAME_SAMPS;
    }

    pos += FRAME_SAMPS * SEARCH_STEP;
  }

  // dump best result
  if (result >= 0) {
    if (quality) *quality = 1 / best_ratio;
    pos = result * FRAME_SAMPS;
#if 0
    D("best echo cancellation ratio at frame %d: %0.2f", result, best_ratio);
#endif
    
#if 0
    // reproduce
    echo_cancel(needle, haystack + pos, needle_samps, out, NULL);
    // dump
    char filename[128];
    sprintf(filename, "sdcard/tmp/out_%d_%0.2f.raw", result, best_ratio);
    FILE *f = fopen(filename, "w+");
    fwrite(out, needle_samps, 2, f);
    fclose(f);
#endif
  } else {
    if (quality) *quality = 0;
  }

  delete[] out;

  return result;
}

// ctor
delay_estimator::delay_estimator(int sr, int frame_samps, int max_delay, int nearend_frames)
: SR(sr),
  FRAME_SAMPS(frame_samps),
  MAX_DELAY(max_delay),
  MAX_FAR_SAMPS(FRAME_SAMPS * (max_delay + nearend_frames)),
  MAX_NEAR_SAMPS(FRAME_SAMPS * nearend_frames),
  total_far_samps(0),
  total_near_samps(0),
  best_delay(-1),
  second_best_delay(-1),
  last_delay(-1),
  largest_delay(-1),
  comp_times(0),
  processing(false),
  async_hint(0),
  succ_times(0)
{
  farbuf = new short[MAX_FAR_SAMPS];
  nearbuf = new short[MAX_NEAR_SAMPS];
  delay_hits = new char[MAX_DELAY]; // [delay] => hits
  delay_quality = new float[MAX_DELAY];
  memset(farbuf, 0, MAX_FAR_SAMPS * 2);
  memset(nearbuf, 0, MAX_NEAR_SAMPS * 2);
  memset(delay_hits, 0, MAX_DELAY * sizeof(delay_hits[0]));
  memset(delay_quality, 0, MAX_DELAY * sizeof(delay_quality[0]));
  pthread_mutex_init(&process_lock, NULL);
  pthread_mutex_init(&buf_lock, NULL);
}

delay_estimator::~delay_estimator()
{
  delete[] farbuf;
  delete[] nearbuf;
  delete[] delay_hits;
  pthread_join(process_thrd, NULL);
  pthread_mutex_destroy(&process_lock);
  pthread_mutex_destroy(&buf_lock);
}

int delay_estimator::array_push(short *dst, int dst_len, int dst_capacity,
    short *src, int n)
{
  if (n >= dst_capacity) {
    memcpy(dst, src + n - dst_capacity, dst_capacity * 2);
  } else {
    // shift
    if (dst_len + n > dst_capacity) {
      int shift = dst_len + n - dst_capacity;
      memmove(dst, dst + shift, (dst_capacity - n) * 2);
      dst_len = dst_capacity - n;
    }
    // push
    memcpy(dst + dst_len, src, n * 2);
  }
  return n;
}

int delay_estimator::add_far(short *data, int n)
{
  pthread_mutex_lock(&buf_lock);

  total_far_samps += array_push(farbuf, total_far_samps % MAX_FAR_SAMPS, MAX_FAR_SAMPS, data, n);

  pthread_mutex_unlock(&buf_lock);

  return n;
}

int delay_estimator::add_near(short *data, int n)
{
  pthread_mutex_lock(&buf_lock);

  total_near_samps += array_push(nearbuf, total_near_samps % MAX_NEAR_SAMPS, MAX_NEAR_SAMPS, data, n); 

  pthread_mutex_unlock(&buf_lock);

  return n;
}

void *process_proc(void *me)
{
  delay_estimator *p = (delay_estimator*)me;
  p->process(p->async_hint);
}

int delay_estimator::is_processing()
{
  pthread_mutex_lock(&process_lock);
  if (processing) {
    pthread_mutex_unlock(&process_lock);
    return true;
  } else {
    pthread_mutex_unlock(&process_lock);
    return false;
  }
}

bool delay_estimator::process_async(int hint)
{
  if (0 != pthread_mutex_trylock(&process_lock))
  {
    D("another process thread already runs, abort");
    pthread_mutex_unlock(&process_lock);
    return false;
  } else {
    bool r = false;
    if (processing) {
      D("another process thread already runs, abort (2)");
    } else {
      async_hint = hint;
      pthread_create(&process_thrd, NULL, process_proc, this);
      r = true;
    }
    pthread_mutex_unlock(&process_lock);
    return r;
  }
}

int delay_estimator::process(int hint)
{
  // set |processing| as true;
  //
  // begin process_lock ///////////////
  if (0 != pthread_mutex_trylock(&process_lock)) {
    D("another process thread already runs, abort (3)");
    return -1;
  }
  // single running instance
  if (processing) {
    D("another process thread already runs, abort (4)");
    return -1;
  }
  if (total_near_samps < MAX_NEAR_SAMPS) {
    D("nearend buffer not enough (%d/%d), return", total_near_samps, MAX_NEAR_SAMPS);
    processing = false;
  } else {
    processing = true;
  }
  pthread_mutex_unlock(&process_lock);
  // end process_lock ///////////////

  int result = -1;
  if (processing)
  {
#if !REOPEN_SPEEX 
    speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 1);
#endif

    // take snapshot of farend and nearend buffer
    int total_far_samps_ = 0;
    int total_near_samps_ = 0;
    short *far = NULL;
    short *near = NULL;

    float quality = 0;

    int64_t t0 = timestamp(0);

    ////// begin buf_lock ///////
    pthread_mutex_lock(&buf_lock);
    //
    total_far_samps_ = total_far_samps;
    total_near_samps_ = total_near_samps;
    far = new short[MAX_FAR_SAMPS];
    near = new short[MAX_NEAR_SAMPS];
    memcpy(far, farbuf, MAX_FAR_SAMPS * 2);
    memcpy(near, nearbuf, MAX_NEAR_SAMPS * 2);
    //
    pthread_mutex_unlock(&buf_lock);
    ////// end buf_lock ///////

    int pos_far = total_far_samps_ < MAX_FAR_SAMPS ? 0 : total_far_samps_ - MAX_FAR_SAMPS;
    int pos_near = total_near_samps_ - MAX_NEAR_SAMPS;
    int d = -1; // result of search_audio()

    // formula:
    // pos_far + d = pos_near - delay
    // 
    // |------------------------------------------------------------> time
    // |
    // |        pos_far
    // |----------|===========|//////|==========|------------------- farend
    // |          |<----d---->|                 |
    // |          |<-----------MAX_FAR--------->|
    // |                      |
    // |                      |             pos_near
    // |----------------------|---------------|//////|-------------- nearend
    // |                      |<----delay---->|      |
    // |                                      |<---->|
    // |                                      MAX_NEAR
    // |

    // use |last_delay| as search hint for better efficiency
    if (hint <= 0)
        hint = last_delay;

    // search from the |k|-th frame of |far| buffer
    int k = (pos_near - pos_far) / FRAME_SAMPS - hint;
    k -= 2;

    if (hint <= 0 || k <= 0) {
        // search from the begining of |far| buffer
        d = search_audio(far, MAX_FAR_SAMPS, near, MAX_NEAR_SAMPS, &quality);
    } else {
        d = search_audio(far + k * FRAME_SAMPS, MAX_FAR_SAMPS - k * FRAME_SAMPS, near, MAX_NEAR_SAMPS, &quality);
        if (d >= 0) {
            d += k;
            I("search_audio skip %d", k);
        } else {
            d = search_audio(far, k * FRAME_SAMPS + MAX_NEAR_SAMPS, near, MAX_NEAR_SAMPS, &quality);
            if (d >= 0)
              I("search_audio skip failed, start from %d, got %d", k, d);
        }
    }

    if (d >= 0) {
        result = (pos_near - (pos_far + d * FRAME_SAMPS)) / FRAME_SAMPS;

        if (result >= 0) {

            // record hits and quality
            delay_hits[result] += 2;
            if (delay_quality[result] < quality) {
                delay_quality[result] = quality;
            }

            D("search_audio, delay %d=(pn %d - pf %d - d %d), quality %0.2f, hit %d",
                    result,
                    pos_near / FRAME_SAMPS,
                    pos_far / FRAME_SAMPS,
                    d,
                    quality, delay_hits[result]);

            // promote adjacency
            if (result - 1 >= 0) {
                delay_hits[result - 1] += quality < delay_quality[result - 1] ? 2 : 1;
            }
            if (result + 1 < MAX_DELAY) {
                delay_hits[result + 1] += quality < delay_quality[result + 1] ? 2 : 1;
            }

            // record best delay
            if (best_delay < 0
                || (best_delay != result
                  && score_delay(best_delay) < score_delay(result)))
            {
              best_delay = result;
              D("refresh best result: delay %d, quality %0.2f, hits : %d, compare times: %d",
                  result, delay_quality[result], delay_hits[result], comp_times);
            }

            // record last delay
            last_delay = result;

            // record largest delay
            if (largest_delay < result)
              largest_delay = result;

            ++succ_times;
        }
    }

    I("process done, (%d,%d~%d,%d~%d), result(best/curr) %d/%d, q %0.2f/%0.2f, hit %d/%d, %dms",
        hint,
        pos_near / FRAME_SAMPS,
        total_near_samps_ / FRAME_SAMPS,
        pos_far / FRAME_SAMPS,
        total_far_samps_ / FRAME_SAMPS,
        best_delay,
        result,
        best_delay < 0 ? 0 : delay_quality[best_delay],
        result < 0 ? 0 : delay_quality[result],
        delay_hits[best_delay],
        delay_hits[result],
        (int)timestamp(t0));

    // set |processing| as false
    pthread_mutex_lock(&process_lock);
    processing = false;
    pthread_mutex_unlock(&process_lock);

    if (far)
      delete[] far;
    if (near)
      delete[] near;

#if !REOPEN_SPEEX 
    speex_ec_close();
#endif
  }

  return result;
}

int delay_estimator::score_delay(int delay)
{
  return delay_quality[delay] * 100 + delay_hits[delay]; 
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
    for (int i = 0; i < far_size - FRAME_SAMPS; i += FRAME_SAMPS) {
      if (silent(far + i, FRAME_SAMPS, 2000))
        continue;
      if (max_amp < 0 || abs(far[max_amp]) < abs(far[i])) {
        max_amp = i;
        if (abs(far[i]) > 32768 / 4)
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
    D("---- delay %d*%d => %0.2f", step_size / FRAME_SAMPS, i, ratio);
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

