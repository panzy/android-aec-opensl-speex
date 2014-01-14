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
  int comp_times;
  SpeexEchoState *st;

  //------------------------------------------------------------
  public:

  delay_estimator(int sr, int frame_samps);
  ~delay_estimator();

  static bool silent(const short *data, int samps, short threshold_amp);

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
  // 判断缓冲区里是否有足够的非静音信号来试验回声消除。
  bool rich_nearend(const short *near, int near_size);
  int search_audio(short *haystack, int haystack_samps,
          short *needle, int needle_samps, float *quality);
  // 返回nearend在回声消除后、前的信号强度之比，此值越小于1，意味着回声消除效果越
  // 好。通常 < 0.9 就意味着回声消除已经起作用了。
  float try_echo_cancel(
          const short *far, int far_size,
          const short *near, int near_size,
          int filter_size);
};

#endif
