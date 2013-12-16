/* delay_estimator
 *
 * 用迭代调用 speex 回声消除的方法在预设范围内搜索回声延迟的值。
 */
#include "speex/speex_echo.h"

class delay_estimator {
  int SR;
  int FRAME_SAMPS;
  int MAX_DELAY;
  int MAX_FAR_SAMPS; // search scope in farend
  int MAX_NEAR_SAMPS; // size of search target
  short *far;
  short *near;
  char *delay_score; // [delay] => hit_times
  int far_samps;
  int far_offset; // count of passed samples in |far| buffer
  int near_samps; // count of samps in |near| buffer
  int near_offset; // count of passed samples in |near| buffer
  float best_quality;
  int best_delay;
  int search_times;
  SpeexEchoState *st;

  public:

  // max_delay - size of farend buffer in frames
  // nearend_frames - size of nearend buffer in frames
  delay_estimator(int sr, int frame_samps, int max_delay, int nearend_frames);
  ~delay_estimator();
  int add_far(short *data, int samps);
  // quality - output, can be NULL
  int process_near(short *data, int samps, float *quality);

  int get_far_offset() { return far_offset; }
  int get_near_offset() { return near_offset; }

  private:

  void speex_ec_open (int sampleRate, int bufsize, int totalSize);
  void speex_ec_close ();
  void echo_cancel(short *in, short *ref, int samps,
      short *out, float *cancellation_ratio);
  int search_audio(short *haystack, int haystack_samps, short *needle, int needle_samps, float *quality);
};

