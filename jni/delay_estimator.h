/* delay_estimator
 *
 * 用迭代调用 speex 回声消除的方法在预设范围内搜索回声延迟的值。
 */
//#include "<pthread.h>" // not found
#include "/opt/android-ndk-r9b/platforms/android-9/arch-arm/usr/include/pthread.h"
#include "speex/speex_echo.h"

class delay_estimator {
  int SR;
  int FRAME_SAMPS;
  int MAX_DELAY;
  int MAX_FAR_SAMPS, MAX_FAR2_SAMPS; // search scope in farend
  int MAX_NEAR_SAMPS, MAX_NEAR2_SAMPS; // size of search target
  short *far, *far2; // far2 is for relay
  short *near, *near2;
  int far_samps, far2_samps;
  int far_offset; // count of passed samples in |far| buffer
  int near_samps, near2_samps; // count of samps in |near| buffer
  int near_offset; // count of passed samples in |near| buffer
  char *delay_score; // [delay] => hit_times
  float best_quality;
  int best_delay;
  int last_delay;
  int comp_times;
  bool processing; // is processing in progress?
  pthread_mutex_t process_lock;
  pthread_t process_thrd;
  SpeexEchoState *st;

  public:
  int async_hint; // hint param of process_async method.
  int async_result;

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

  int get_far_offset() { return far_offset; }
  int get_near_offset() { return near_offset; }

  //------------------------------------------------------------
  private:

  int64_t timestamp(int64_t base);
  bool silent(short *data, int samps);
  void speex_ec_open (int sampleRate, int bufsize, int totalSize);
  void speex_ec_close ();
  void echo_cancel(short *in, short *ref, int samps,
      short *out, float *cancellation_ratio);
  int search_audio(short *haystack, int haystack_samps, short *needle, int needle_samps, float *quality);

  // treat |dst| array as a FIFO queue.
  // return shorts been pushed |n|.
  static int array_push(short *dst, int dst_len, int dst_capacity, short *src, int n, int *dst_offset);
};

