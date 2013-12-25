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

const int SEARCH_STEP = 1;

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
    if (abs(data[i]) < 1000 /* TODO is this method reliable enough?*/)
      ++n0;
  }
  return n0 * 100 / samps > 95;
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

void delay_estimator::echo_cancel(short *in, short *ref, int samps,
        short *out, float *cancellation_ratio)
{
  // XXX 由于我们要查找的delay是精确到单个frame的，speex 的 filter length 为
  // 1xframe即可（1xframe 比 2xframe 快大约 20%）
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
  //D("search_audio, %d/%d", needle_samps, haystack_samps);

  if (silent(needle, needle_samps)) {
    D("ignore silent needle at frame");
    return -1;
  }

  float best_ratio = 99;
  int result = -1; // in frame
  short *out = new short[needle_samps];
  int pos = 0; // in samps
  int idx = 0;
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

    // 如果占用太多CPU资源，会不会影响音频主线程？所以 sleep 一会
    if (idx % 4 == 0)
      usleep(1000);
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
    return best_delay;
  }
  // single running instance
  if (processing) {
    D("another process thread already runs, abort (4)");
    return best_delay;
  }
  if (total_near_samps < MAX_NEAR_SAMPS) {
    D("nearend buffer not enough (%d/%d), return", total_near_samps, MAX_NEAR_SAMPS);
    processing = false;
  } else {
    processing = true;
  }
  pthread_mutex_unlock(&process_lock);
  // end process_lock ///////////////

  if (processing)
  {
    // take snapshot of farend and nearend buffer
    int total_far_samps_ = 0;
    int total_near_samps_ = 0;
    short *far = NULL;
    short *near = NULL;

    int result = -1;
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
  }

  return best_delay;
}

int delay_estimator::score_delay(int delay)
{
  return delay_quality[delay] * 100 + delay_hits[delay]; 
}
