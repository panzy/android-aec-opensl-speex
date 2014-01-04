#ifndef _DELAY_ESTIMATOR_H_
#define _DELAY_ESTIMATOR_H_

/* delay_estimator
 *
 * 用迭代调用 speex 回声消除的方法在预设范围内搜索回声延迟的值。
 */
//#include "<pthread.h>" // not found
#include "/opt/android-ndk-r9b/platforms/android-9/arch-arm/usr/include/pthread.h"
#include "speex/speex_echo.h"
#include "opensl_io2.h" // circular buffer

class delay_estimator {
  int SR;
  int FRAME_SAMPS;
  int MAX_DELAY;
  int MAX_FAR_SAMPS; // search scope in farend
  int MAX_NEAR_SAMPS; // size of search target
  short *farbuf;
  short *nearbuf;
  int total_far_samps;
  int total_near_samps;
  char *delay_hits; // [delay] => hit_times
  float *delay_quality; // [delay] => quality
  int best_delay;
  int second_best_delay;
  int last_delay;
  int largest_delay;
  int comp_times;
  bool processing; // is processing in progress?
  pthread_mutex_t process_lock;
  pthread_mutex_t buf_lock;
  pthread_t process_thrd;
  SpeexEchoState *st;

  public:
  int async_hint; // hint param of process_async method.
  int succ_times; // success times of processing

  //------------------------------------------------------------
  public:

  // max_delay - size of farend buffer in frames
  // nearend_frames - size of nearend buffer in frames
  delay_estimator(int sr, int frame_samps, int max_delay, int nearend_frames);
  ~delay_estimator();
  int add_far(short *data, int samps);
  int add_near(short *data, int samps);
  // hint - if >= 0, used as search hint
  int process(int hint);
  bool process_async(int hint);

  int get_best_delay() { return best_delay <= 0 ? last_delay : best_delay; }
  int get_2nd_best_delay() { return second_best_delay; }
  int get_largest_delay() { return largest_delay; }
  int get_best_quality() { return get_best_delay() >= 0 ? delay_quality[get_best_delay()] : 0; }
  int get_best_hit() { return get_best_delay() >= 0 ? delay_hits[get_best_delay()] : 0; }
  int get_far_samps() { return total_far_samps; }
  int get_near_samps() { return total_near_samps; }
  int is_processing();
  static bool silent(short *data, int samps, short threshold_amp);

  // Estimate echo delay.
  //
  // @param far farend samples.
  // @param far_size sample count.
  // @param near nearend samples.
  // @param near_size sample count.
  // @param filter_size Speex AEC filter size in samples.
  // @param cancel_ratio output cancellation ratio.
  // @return echo delay in samples.
  int estimate(const short *far, int far_size,
          const short *near, int near_size,
          int filter_size, float *cancel_ratio);
  //------------------------------------------------------------
  private:

  int64_t timestamp(int64_t base);
  void speex_ec_open (int sampleRate, int bufsize, int totalSize);
  void speex_ec_close ();
  void echo_cancel(const short *in, const short *ref, int samps,
          short *out, float *cancellation_ratio);
  // recursive helper of estimate().
  int estimate_(const short *far, int far_size,
          const short *near, int near_size,
          int filter_size, int base, float *cancel_ratio);
  int search_audio(short *haystack, int haystack_samps,
          short *needle, int needle_samps, float *quality);
  int score_delay(int delay);
  float try_echo_cancel(
          const short *far, int far_size,
          const short *near, int near_size,
          int filter_size);

  // treat |dst| array as a FIFO queue.
  // return shorts been pushed |n|.
  static int array_push(short *dst, int dst_len, int dst_capacity, short *src, int n);
};

#endif
