/*
 * vim: shiftwidth=2
 */

#include <jni.h>
#include <sys/time.h>
#include <queue>
#include <stdio.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <sys/errno.h>
#include <math.h>
#include <unistd.h>
#include <sys/resource.h> // setpriority
#include "opensl_io2.h"

#include "speex/speex_echo.h"
#include "speex/speex_preprocess.h"

#include "opensl_example.h"
#include "common.h"
#include "delay_estimator.h"

// from <inttypes.h>
#define	PRId64			"lld"		/* int64_t */

#define SR 8000 // sample rate
#define SAMPLE_SIZE 2 // bytes per sample

// AEC frame, 10ms
#define FRAME_SAMPS 160 // samples per frame
#define FRAME_MS (1000 * FRAME_SAMPS / SR) // frame duration in ms
#define FRAME_RATE (1000 / FRAME_MS) // frames per second

// WebRtc delay estimator params
#define WEBRTC_SPECTRUM_SIZE (80)
#define WEBRTC_SPECTRUM_MS (WEBRTC_SPECTRUM_SIZE * FRAME_MS / FRAME_SAMPS)
#define WEBRTC_HISTORY_SIZE ((1500 / 1000) * SR * SAMPLE_SIZE)

// 在主循环中，将控制连续处理 |LOOP_FRAMES|*|FRAME_SAMPS| 所需的时间，使之趋于
// |LOOP_FRAMES|*|FRAME_MS|。
const int LOOP_FRAMES = 4;

const int MAX_DELAY = 50;
const int NEAREND_SIZE = 50; // 太短可能会得到完全错误的结果（取决于实际输入）
// 回声延迟（以frame为单位）的有效值 >= 0，下面是几个特殊的无效值
const int ECHO_DELAY_NULL = -1;
const int ECHO_DELAY_FAILED = -2;
// ……下次估算回声延迟的时间间隔，或者说估算结果的有效期
const int ECHO_DELAY_INTERVAL_MS = 10 * 1000;
const int EST_BUF_CAPACITY = MAX_DELAY * 5 * FRAME_SAMPS; // in samps

// delay tolerance in frames.
// 如果新评估的回声延迟值不大于当前正在使用的延迟值加上此容差，则不作调整。
// 因为每次调整后的一段短暂时间里回声消除都会失效，所以想减少调整。
// 理论上此值小于 |SPEEX_FILTER_SIZE| 就行了。
const int DELAY_TOL = 1;
// speex AEC filter size in frame
const int SPEEX_FILTER_SIZE = 8;

#define TAG "aec" // log tag

//--------------------------------------------------------------------------------

void cleanup();
bool estimation_not_bad(delay_estimator *d);
void fwrite_samps(short *frame, FILE *fd, int samps);
void reverse_frame(short *frame, int samps);
void speex_ec_open (int sampleRate, int bufsize, int totalSize);
void speex_ec_close ();
int playback(short *_farend, int samps, bool with_aec_analyze);
float *to_float(short *frame);
void set_playback_stream_type(jint stream_type);
void stat_ec(const short *inbuffer, const short *outbuffer, int samps);
static jint get_api_level(JNIEnv *env);

//--------------------------------------------------------------------------------

SpeexEchoState *st = NULL;
SpeexPreprocessState *den = NULL;

delay_estimator *delayEst = NULL;
pthread_t delay_est_thrd;
bool delay_est_thrd_stopped;

static int on = 0;
OPENSL_STREAM  *p = NULL;

// 远端信号缓冲区。由于无法保证上层调用push()的节奏，需要此缓冲区来为OpenSL层提
// 供稳定的音频流。
// 这个缓冲区对 echo delay 不会产生影响。
// 上层输入的远端信号经过 farend_buf 缓存后再输入给 audio sytem.
// 当 farend_buf 发生 underrun （入不敷出）时，会插入一定长度的静音。
circular_buffer *farend_buf = NULL;
// echo_buf 与 farend_buf 的区别在于：
// 它的开头有|echo_delay|个 slience frame；
circular_buffer *echo_buf = NULL;
// 消除了回声的近端信号，供上层pull
circular_buffer *nearend_buf = NULL;
// 以上几个缓冲区的容量，in samps
int farend_buf_size = 0;
int nearend_buf_size = 0;
int echo_buf_size = 0;
//
// for delay estimation
//
circular_buffer *render_hist = NULL;
circular_buffer *capture_hist = NULL;

int64_t t_start;

// dump audio data
FILE *fd_farend = NULL;
FILE *fd_farend2 = NULL;
FILE *fd_nearend = NULL;
FILE *fd_nearend2 = NULL;
FILE *fd_echo = NULL;
FILE *fd_send = NULL;

short silence[FRAME_SAMPS];

int playback_delay = 0; // samps
int echo_delay = ECHO_DELAY_NULL; // samps, relative to playback
int echo_delay2 = ECHO_DELAY_NULL; // samps, relative to playback, estimated by WebRtc 
bool echo_delay2_desired = false;
int64_t last_est_time = 0; // last time we got |echo_delay2|
// 记录最近一次评估回声延迟时得到的最好消除效果（输出、输入信号强度之比），作为
// stat_ec() 中判断当前回声消除是否有效的参考标准。之所以需要这么一个动态标准，
// 是因为不同设备之间的最好效果差别很大。
float recent_best_ec_ratio = -1;
// 准实时统计的回声消除效果
float curr_ec_ratio = -1;
int in_buffer_cnt = 0;
int out_buffer_cnt = 0;

bool dump_raw = false;

// 用于评估回声延迟的缓冲区
short far_est_buf[EST_BUF_CAPACITY];
short near_est_buf[EST_BUF_CAPACITY];
// 以下指针指向以上缓冲区的当前写入位置。
short *far_est_p = NULL;
short *near_est_p = NULL;
int far_est_size = 0;
int near_est_size = 0;

//----------------------------------------------------------------------

// 根据 estimate_delay() 的返回值，设置 |echo_delay2| 全局变量的值。
// 之所以不用简单的赋值，是因为存在 |DELAY_TOL| 常数。
void adjust_echo_delay2(int value)
{
  if (value >= 0) {
    if (echo_delay != ECHO_DELAY_NULL
        && echo_delay <= value
        && echo_delay + DELAY_TOL > value) {
      echo_delay2 = echo_delay;
    } else {
      echo_delay2 = value - DELAY_TOL;
    }
  } else {
    echo_delay2 = ECHO_DELAY_FAILED;
  }
}

void align_farend_buf(int n, short *last_render_frame, int last_render_samps)
{
  // 如果用静音帧来补充，会有破音，进而对录音和回声消除产生严重影响，但是如果用
  // 上一帧补充，有可能产生音量很大的嗡鸣，这可能是由于波形上的锯齿，想象
  // |last_render_frame| 是一个水平放置的梯形，为了避免锯齿，创建一个该梯形的副
  // 本并且将其旋转180度。
  const int PAD_SAMPS = 80; // padding samps
  short rframe[PAD_SAMPS]; // reversed frame
  if (last_render_samps > PAD_SAMPS) {
    last_render_frame += last_render_samps - PAD_SAMPS;
    last_render_samps -= PAD_SAMPS;
  }
  memcpy(rframe, last_render_frame, last_render_samps * sizeof(short));
  reverse_frame(rframe, last_render_samps);
  short *p = rframe;
  while (n > 0) {
    if (n <= last_render_samps) {
      playback(p, n, true);
      n = 0;
    } else {
      playback(p, last_render_samps, true);
      n -= last_render_samps;
    }
    if (p == rframe) {
      p = last_render_frame;
    } else {
      p = rframe;
    }
  }
}

void reverse_frame(short *frame, int samps)
{
  short t;
  for (short *p = frame, *q = frame + samps - 1;
      p <= q;
      ++p, --q) {
    t = *p;
    *p = *q;
    *q = t;
  }
}

// get timestamp of today in ms.
int64_t timestamp(int64_t base)
{
  timeval t;
  gettimeofday(&t, NULL);
  return ((int64_t)t.tv_sec * 1000 + (int64_t)(t.tv_usec / 1000)) - base;
}

void fwrite_samps(short *frame, FILE *fd, int samps)
{
  if (fd) {
    int n = fwrite(frame, samps, 2, fd);
    if (2 != n) {
      E("fwrite failed, return value = %d(expect 2), errno %d", n, errno);
    }
  }
}

// TODO rename to open_est_dump_files
void open_log_files()
{
  //I("open_log_files");
  const char *mode = "w+";
  int r = mkdir("/mnt/sdcard/tmp", 755);
  if (r == 0 || errno == EEXIST) {
    fd_farend2 = fopen("/mnt/sdcard/tmp/far2.raw", mode);
    fd_nearend2 = fopen("/mnt/sdcard/tmp/near2.raw", mode);
  } else {
    fd_farend2 = fd_nearend2 = NULL;
  }
}

void open_dump_files(const char *mode)
{
  if (dump_raw) {
    int r = mkdir("/mnt/sdcard/tmp", 755);
    if (r == 0 || errno == EEXIST) {
      fd_farend = fopen("/mnt/sdcard/tmp/far.raw", mode);
      fd_nearend = fopen("/mnt/sdcard/tmp/near.raw", mode);
      fd_echo = fopen("/mnt/sdcard/tmp/echo.raw", mode);
      fd_send = fopen("/mnt/sdcard/tmp/send.raw", mode);
    } else {
      fd_farend = fd_nearend = fd_echo = fd_send = NULL;
    }
  }
}

// TODO rename to close_est_dump_files
void close_log_files()
{
  //I("close_log_files");
  if (fd_farend2)
    fclose(fd_farend2);
  if (fd_nearend2)
    fclose(fd_nearend2);
  fd_farend2 = fd_nearend2 = NULL;
}

void close_dump_files()
{
  if (dump_raw) {
    if (fd_farend)
      fclose(fd_farend);
    if (fd_nearend)
      fclose(fd_nearend);
    if (fd_echo)
      fclose(fd_echo);
    if (fd_send)
      fclose(fd_send);
  }
  fd_farend = fd_nearend = fd_echo = fd_send = NULL;
  //I("dump files closed");
}

void start(JNIEnv *env, jint track_min_buf_size, jint record_min_buf_size,
    jint playback_delay_ms, jint echo_delay_ms, jint _dump_raw)
{
  dump_raw = _dump_raw;
  playback_delay = playback_delay_ms / FRAME_MS;
  //
  // 设置变量：
  // - echo_delay 立即用于AEC
  // - echo_delay2 要么为 NULL 要么为 echo_delay，若为前者，
  //    runNearendProcessing() 将创建一个线程来运行 estimate_delay()。
  //    一旦 echo_delay2 被计算出来，就会用来修正 echo_delay。
  echo_delay2_desired = echo_delay_ms < 0;
  if (echo_delay_ms < 0) {
    echo_delay = 10;
    echo_delay2 = ECHO_DELAY_NULL; // 要求动态评估
  } else {
    echo_delay = echo_delay_ms / FRAME_MS;
    echo_delay2 = echo_delay;
    echo_delay2 = ECHO_DELAY_NULL; // 要求动态评估
  }

  const int sample_size = sizeof(short);
  in_buffer_cnt = ceil((float)record_min_buf_size / sample_size / FRAME_SAMPS);
  out_buffer_cnt = ceil((float)track_min_buf_size / sample_size / FRAME_SAMPS);

  I("AudioRecord min buf size %dB, AudioTrack min buf size %dB",
      record_min_buf_size, track_min_buf_size);
  I("OpenSL audio in-buffer samps %dx%d, out-buffer samps %dx%d, echo delay %d(x%d=%dms)",
      FRAME_SAMPS, in_buffer_cnt, FRAME_SAMPS, out_buffer_cnt,
      echo_delay, FRAME_MS, echo_delay * FRAME_MS);

  // 太小的 out_buffer_cnt （比如3） 不足以保证流畅的播放，因为播放队列缓冲的
  // 数据太少，主循环稍有迟滞就会导致播放数据供应不足。根据经验，缓冲500~1000ms
  // 比较保保险。
  const int min_outbuf_cnt = 100;
  const int min_inbuf_cnt = 50;
  if (out_buffer_cnt < min_outbuf_cnt)
    out_buffer_cnt = min_outbuf_cnt;
  if (in_buffer_cnt < min_inbuf_cnt)
    in_buffer_cnt = min_inbuf_cnt;

  p = android_OpenAudioDevice(SR, 1, 1,
      FRAME_SAMPS,
      in_buffer_cnt,
      FRAME_SAMPS,
      out_buffer_cnt,
      get_api_level(env));
  if(p == NULL) return; 

  farend_buf_size =  FRAME_SAMPS * std::max(FRAME_RATE * 5, 20 + playback_delay);
  farend_buf = create_circular_buffer(farend_buf_size);

  // 观测到过上层的java代码调用 pull() 的最大时间间隔达 3200+ms
  nearend_buf_size = (FRAME_SAMPS * FRAME_RATE * 5);
  nearend_buf = create_circular_buffer(nearend_buf_size);

  echo_buf_size = (MAX_DELAY /*+ playback_delay*/ + 4) * 2 * FRAME_SAMPS;
  echo_buf = create_circular_buffer(echo_buf_size);

  int hist_size = EST_BUF_CAPACITY + 1 
    + out_buffer_cnt * FRAME_SAMPS;
  render_hist = create_circular_buffer(hist_size);
  capture_hist = create_circular_buffer(hist_size);

  speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * SPEEX_FILTER_SIZE);

  open_dump_files("w+");
  on = 1;
  delay_est_thrd_stopped = true;
  far_est_p = near_est_p = NULL;
  far_est_size = near_est_size = 0;
  recent_best_ec_ratio = -1;
  curr_ec_ratio = -1;

  memset(silence, 0, FRAME_SAMPS * sizeof(short));
  D("start at %" PRId64 "ms" , timestamp(0)); 
}

void runNearendProcessing() 
{
  short inbuffer[FRAME_SAMPS]; // record
  short refbuf[FRAME_SAMPS];
  short processedbuffer[FRAME_SAMPS];
  short render_buf[FRAME_SAMPS];

  int pri = getpriority(PRIO_PROCESS, 0);
  setpriority(PRIO_PROCESS, 0, -19);
  I("runNearendProcessing, niceness: %d=>%d", pri, getpriority(PRIO_PROCESS, 0));

  memset(render_buf, 0, FRAME_SAMPS * 2);

  // delay echo_buf (relative to farend_buf)
  if (echo_delay > 0) {
    for (int i = 0; i < echo_delay; ++i)
      write_circular_buffer(echo_buf, silence, FRAME_SAMPS);
  }

  //
  // 主循环
  //
  // 我们并不试图让每一次循环的持续时间精确为 FRAME_MS，因为这不可能做到：尽管绝
  // 大多数情况下我们只需要比如说3ms就处理完了一个frame，但是偶尔（进程调度？）
  // 也会需要明显超过 FRAME_MS 的毫秒数。
  // 所以我们能做的是在一个更大的时间跨度上让实际流逝时间与音频时间大体上保持一致
  // ，这个偏差要在相关缓冲区的承受范围之内。
  //
  // 另外，AudioSystem record buffer overflow 好像也跟这个有关。
  //
  // 把时间提前量定义为
  //    ahead = 物理时长 - 已处理的音频的时长
  // 然后在循环体中试图将 |ahead| 维持在 [min_ahead, max_ahead] 区间内。
  // |min_ahead| 不能太小，否则就令 opensl_stream::outrb 失去了缓冲效果。
  int max_ahead = (out_buffer_cnt - 2) * FRAME_MS; // (int)(out_buffer_cnt * 3 / 4) * FRAME_MS;
  int min_ahead = max_ahead - 1 * FRAME_MS;// std::max(max_ahead / 2, 5 * FRAME_MS);
  I("main loop time ahead target range: (%d,%d)ms, or (%d,%d)frames",
      min_ahead, max_ahead, min_ahead / FRAME_MS, max_ahead / FRAME_MS);

  // 每次循环后:
  // 1, loop_idx++
  // 2, rendered_samps += 略大于等价的物理流逝时间
  // 
  // 对于第2条，我们通过以下手段保证：
  // A, 如果上游提供的 far buffer 供应不足，用静音补充
  // B, 如果向下游（OpenSL）提供的 out buffer 达到某阈值，sleep
  //
  // 总之，循环的节奏从根本上说以物理时间为准。
  //
  int loop_idx = -1;
  int rendered_samps = 0;
  int captured_samps = 0;
  int64_t t0 = timestamp(0), t1 = 0; // loop body begins
  // 首次从mic读取到数据后才开始播放。
  // 不这么做的话对缓冲区进行回声延迟修正时可能出现偏差。在集成测试中，表现
  // 为己方发起的语音通话不能正确消除回声，但对方发起的则可以。
  // 这个时间间隔貌似主要由 |record_min_buf_size| 决定。
  int64_t first_cap = 0; // first time of capture.
  t_start = t0;
  while(on) {
    t1 = timestamp(0);
    ++loop_idx;

    //
    // estimate echo delay
    //

    // |render_hist| 和 |capture_hist| 必须按相同的进度 shift。
    //
    // 写满 |EST_BUF_CAPACITY| 就 shift 一次。
    if (checkspace_circular_buffer(render_hist, 0) >= EST_BUF_CAPACITY
        && checkspace_circular_buffer(capture_hist, 0) >= EST_BUF_CAPACITY) {

      if (loop_idx > playback_delay
          && delay_est_thrd_stopped
          && (last_est_time == 0
            || timestamp(last_est_time) > ECHO_DELAY_INTERVAL_MS)) {
        if (1 || !delay_estimator::silent(render_buf, FRAME_SAMPS, 2500)) {
          // copy data from |render_hist| and |capture_hist| 
          // to |far_est_buf| and |near_est_buf|.
          read_circular_buffer(render_hist, far_est_buf, EST_BUF_CAPACITY);
          read_circular_buffer(capture_hist, near_est_buf, EST_BUF_CAPACITY);

          far_est_size = near_est_size = EST_BUF_CAPACITY;
          if (dump_raw) {
            open_log_files();
            fwrite_samps(far_est_buf, fd_farend2, far_est_size);
            fwrite_samps(near_est_buf, fd_nearend2, near_est_size);
            close_log_files();
          }

          // start estimation thread
          delay_est_thrd_stopped = false;
          pthread_create(&delay_est_thrd, NULL, (void* (*)(void*))estimate_delay, (void*)1);
        }
      } else {
        // discard
        short tmp[FRAME_SAMPS];
        for (int i = 0, n = EST_BUF_CAPACITY / FRAME_SAMPS; i < n; ++i) {
          read_circular_buffer(render_hist, tmp, FRAME_SAMPS);
          read_circular_buffer(capture_hist, tmp, FRAME_SAMPS);
        }
      }
    }

    //
    // record
    //
    int64_t rec_proc_start = timestamp(0);
    int lack_cap_samps = timestamp(t0) * FRAME_SAMPS / FRAME_MS - captured_samps;
    if (lack_cap_samps >= FRAME_SAMPS * LOOP_FRAMES) {
      W("record overrun, lack_cap_samps %d", lack_cap_samps);
    }
    int loop_cnt = 0;
    while (lack_cap_samps >= FRAME_SAMPS) {
      ++loop_cnt;
      int samps = android_AudioIn(p,inbuffer,FRAME_SAMPS);
      if (samps == FRAME_SAMPS) {
        if (first_cap == 0) { 
          first_cap = timestamp(0);
          D("first time of capture at @%dms", (int)(first_cap - t0));
        }
        captured_samps += samps;
        lack_cap_samps -= samps;

        write_circular_buffer(capture_hist, inbuffer, samps);

        // adjust echo buffer, and read a frame for current AEC
        if (echo_delay2 >= 0 && echo_delay2 != echo_delay) {
          I("adjust echo buffer: %d=>%d at @%ds", echo_delay, echo_delay2,
              (int)(timestamp(t0) / 1000));
          if (echo_delay >= 0) {
            if (echo_delay2 > echo_delay) {
              for (int i = 0, n = echo_delay2 - echo_delay; i < n; ++i) {
                write_circular_buffer(echo_buf, silence, FRAME_SAMPS);
              }
            } else {
              // shift some frames
              for(int i = 0, n = echo_delay - echo_delay2; i < n; ++i) {
                read_circular_buffer(echo_buf, refbuf, samps);
              }
            }
          } else {
            for (int i = 0, n = echo_delay2; i < n; ++i) {
              write_circular_buffer(echo_buf, silence, FRAME_SAMPS);
            }
          }
          // reopen speex
          speex_ec_close();
          speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * SPEEX_FILTER_SIZE);
          echo_delay = echo_delay2;
        }
        int ref_samps = read_circular_buffer(echo_buf, refbuf, samps);

        short *out = inbuffer; // output(send) frame
        if (loop_idx < playback_delay + echo_delay || echo_delay <= 0 || ref_samps < samps) {
          // output as-is
        } else {
          // do AEC
          speex_echo_cancellation(st, inbuffer, refbuf, processedbuffer);
          speex_preprocess_run(den, processedbuffer);
          out = processedbuffer;
          stat_ec(inbuffer, out, samps);
        }
        // output
        if (samps != write_circular_buffer(nearend_buf, out, samps)) {
          E("nearend_buf full");
        }
        if (dump_raw) {
          fwrite_samps(inbuffer, fd_nearend, samps);
          if (ref_samps > 0)
            fwrite_samps(refbuf, fd_echo, ref_samps);
          fwrite_samps(out, fd_send, samps);
        }
      } else {
        D("record nothing at @%ds", (int)(timestamp(t0) / 1000));
        // reset |captured_samps| according real time.
        I("reset captured_samps at @%ds", (int)(timestamp(t0) / 1000));
        captured_samps += lack_cap_samps;
        lack_cap_samps = 0;
        break;
      }
      if (timestamp(t0) > FRAME_MS * LOOP_FRAMES / 2)
        break;
    }
    if (timestamp(rec_proc_start) >= FRAME_MS * LOOP_FRAMES) {
      W("record processing lasts too long: %"PRId64"ms for %d loops",
          timestamp(rec_proc_start), loop_cnt);
    }

    //
    // playback
    //
    if (first_cap > 0) {
      int64_t play_proc_start = timestamp(0);
      if (loop_idx < playback_delay) {
        playback(silence, FRAME_SAMPS, false);
        rendered_samps += FRAME_SAMPS;
      } else {
        int samps = read_circular_buffer(farend_buf, render_buf, FRAME_SAMPS);
        if (samps > 0) {
          playback(render_buf, samps, true);
          rendered_samps += samps;
        }
        // pad underrun with silence
        // |playback_delay| 可以显著减少这种情况的发生
        int lack_samps = timestamp(t0) * FRAME_SAMPS / FRAME_MS - rendered_samps;
        lack_samps += 2 * FRAME_SAMPS; // 不但不应该紧缺，还应该有点富余
        if (lack_samps > 0) {
          //D("playback underrun, lack of %d samps", lack_samps);
          if (samps > 0) {
            align_farend_buf(lack_samps, render_buf, samps);
          } else {
            align_farend_buf(lack_samps, silence, FRAME_SAMPS);
          }
          rendered_samps += lack_samps;
          if (lack_samps >= FRAME_SAMPS)
            memset(render_buf, 0, FRAME_SAMPS * 2);
        }
      }
      if (timestamp(play_proc_start) >= FRAME_MS * LOOP_FRAMES / 2) {
        W("playback processing lasts too long: %"PRId64"ms",
            timestamp(play_proc_start));
      }
    }

    // 平衡 |rendered_samps| 与物理时间。
    // (|captured_samps| 与物理时间的冲突通过在特定条件下重置|captured_samps|来
    // 解决。）
    if (first_cap == 0) {
      usleep(FRAME_MS * 1000);
    } else {
      int64_t total_ahead = (rendered_samps / FRAME_SAMPS * FRAME_MS) - timestamp(first_cap);
      if (total_ahead > max_ahead) {
        total_ahead = total_ahead - min_ahead;
        //D("idle: sleep %"PRId64"ms", total_ahead);
        usleep(1000 * total_ahead);
      } else if (total_ahead < -0 * FRAME_MS) {
        // 这个很严重，播放有破音，回声消除可能从此失效一段时间。
        // 发生这个情况的原因是循环体的本次运行中某个操作耗时太长，导致播放队列
        // 见底了。
        E("idle: total overrun for %dms! at @%dms", (int)(-total_ahead),
            (int)(timestamp(t0)));
      }
    }

    // log time elapse
    if (1) {
      int64_t curr_ahead = FRAME_MS * LOOP_FRAMES - timestamp(t1);
      if (curr_ahead >= 0) {
        //D("idle: curr %"PRId64"ms", curr_ahead);
      } else {
        W("idle: curr overrun for %"PRId64"ms!", -curr_ahead);
      }
    }
  }

  if (!delay_est_thrd_stopped) {
    pthread_join(delay_est_thrd, NULL);
  }

  cleanup();
}

int get_estimated_echo_delay()
{
  return echo_delay2 >= 0 ? echo_delay2 * FRAME_MS : -1;
}

jint get_playback_stream_type()
{
  return p ? openSLPlayQueryStreamType(p) : -1;
}

void set_playback_stream_type(jint stream_type)
{
  if (p) {
    openSLPlayClose(p);
    openSLPlayOpen(p, (SLint32)stream_type);
  }
}

void stop()
{
  on = 0;
}

void cleanup()
{
  android_CloseAudioDevice(p);
  free_circular_buffer(farend_buf);
  free_circular_buffer(echo_buf);
  free_circular_buffer(nearend_buf);
  free_circular_buffer(render_hist);
  free_circular_buffer(capture_hist);
  close_dump_files();
  speex_ec_close();

  if (delayEst) {
    delete delayEst;
    delayEst = NULL;
  }
  I("clean up");
}

int estimate_delay(int use_mem_data)
{
  static int cnt = -1;
  float cancel_ratio = 1;
  int result = -1;

  int pri = getpriority(PRIO_PROCESS, 0);
  setpriority(PRIO_PROCESS, 0, -8);
  I("estimate_delay, niceness: %d=>%d", pri, getpriority(PRIO_PROCESS, 0));

  I("start estimate_delay, #%d", ++cnt);

  int64_t t0 = timestamp(0);

  if (!use_mem_data) { // use file data
    FILE *fd_f = fopen("/mnt/sdcard/tmp/far2.raw", "r");
    FILE *fd_n = fopen("/mnt/sdcard/tmp/near2.raw", "r");
    if (!fd_f || !fd_n) {
      E("failed to open dump files at %d", __LINE__ );
      goto END;
    }
    far_est_size = fread(far_est_buf, 2, sizeof(far_est_buf) / 2, fd_f);
    near_est_size = fread(near_est_buf, 2, sizeof(near_est_buf) / 2, fd_n);
    fclose(fd_f);
    fclose(fd_n);
  }

  delayEst = new delay_estimator(SR, FRAME_SAMPS, MAX_DELAY, NEAREND_SIZE );
  result = delayEst->estimate(
      far_est_buf, far_est_size,
      near_est_buf, near_est_size,
      FRAME_SAMPS * 8,
      &cancel_ratio );
  if (result >= 0) {
    result /= FRAME_SAMPS; // convert from samples to frames
    if (curr_ec_ratio < 0 || cancel_ratio < curr_ec_ratio) {
      adjust_echo_delay2(result);
      recent_best_ec_ratio = cancel_ratio;
    }
  }

END:

  I("delay estimation done, result %d(%dms), cancellation ratio %0.2f, "
      "elapse %dms",
      result, result * FRAME_MS, cancel_ratio, (int)timestamp(t0));
  I("got echo_delay2 %d", echo_delay2);

  if (delayEst) {
    delete delayEst;
    delayEst = NULL;
  }

  far_est_p = near_est_p = NULL;
  delay_est_thrd_stopped = true;
  last_est_time = timestamp(0);
  return 0;
}

int estimate_delay_bak(int async)
{
  static int cnt = -1;

  // 全局变量 |on| 的初衷是用来控制 runNearendProcessing() 的，
  // estimate_delay() 既可以被 runNearendProcessing() 调用，也可以直接调用，
  // 仅在前一种情况中，它受 |on| 控制。
  bool watch_on = on;

  int pri = getpriority(PRIO_PROCESS, 0);
  setpriority(PRIO_PROCESS, 0, 0);
  I("estimate_delay, niceness: %d=>%d", pri, getpriority(PRIO_PROCESS, 0));

  I("start estimate_delay, #%d", ++cnt);

  short far[MAX_DELAY * FRAME_SAMPS];
  short near[FRAME_SAMPS];
  delayEst = new delay_estimator(SR, FRAME_SAMPS, MAX_DELAY, NEAREND_SIZE );

  FILE *fd_f = fopen("/mnt/sdcard/tmp/far2.raw", "r");
  FILE *fd_n = fopen("/mnt/sdcard/tmp/near2.raw", "r");
  if (!fd_f || !fd_n) {
      E("failed to open dump files at %d", __LINE__ );
      return 0;
  }

  int64_t t0 = timestamp(0);
  int result = -1;
  int i = 0;
  int skip_countdown = 0;
  // 描述 delay_estimator::process() 结果的惯性：
  // 成功就 +1；失败就 -1；交替就清零。
  int inertance = 0;
  bool echo_delay2_adjusted = false;
  fseek(fd_n, i * FRAME_SAMPS * 2, SEEK_SET);
  while(!watch_on || on)
  {
    if (FRAME_SAMPS != fread(far, 2, FRAME_SAMPS, fd_f))
      break;
    delayEst->add_far(far, FRAME_SAMPS);

    if (FRAME_SAMPS != fread(near, 2, FRAME_SAMPS, fd_n))
      break;

    delayEst->add_near(near, FRAME_SAMPS);

    // 减少搜索次数，希望不会影响结果的正确性
    if (--skip_countdown > 0) {
      continue;
    }

    int hint = delayEst->get_largest_delay() > 0
      ? delayEst->get_largest_delay() : echo_delay + DELAY_TOL;
    if (!async) {
      // sync call
      result = delayEst->process(hint);
      if (result < 0) {
        skip_countdown = 5;
        if (inertance >= 0) {
          inertance = -1;
        } else {
          --inertance;
        }
        // 如果连续失败好多次，而且之前已经有了不错的结果，就早点结束
        if (inertance < -2
            && estimation_not_bad(delayEst)) {
          I("estimate_delay, end early, case #2");
          break;
        }
        continue;
      } else {
        skip_countdown = 2;
        if (inertance <= 0) {
          inertance = 1;
        } else {
          ++inertance;
        }
      }
    } else {
      // async call
      if (delayEst->get_near_samps() > NEAREND_SIZE) {
        delayEst->process_async(hint);
      } else {
        D("near samps %d", delayEst->get_near_samps());
      }
      usleep(FRAME_MS * 1000);
    }

    if (result >= 0
        && estimation_not_bad(delayEst)) {
      // 得到了一个质量不太差的结果
      
      result = delayEst->get_best_delay();
      if (echo_delay2_desired) {
        // 刷新输出结果，但不放弃搜索
        I("estimate_delay, output early");
        echo_delay2 = result - DELAY_TOL;
        echo_delay2_desired = false;
      } else if (result >= echo_delay && result <= echo_delay + DELAY_TOL) {
        // 这个结果与 |echo_delay| 基本一致，那么二者都正确的可能性比较高，无需
        // 继续验证了。
        I("estimate_delay, end early");
        adjust_echo_delay2(result);
        echo_delay2_adjusted = true;
        break;
      }
    }
  }

  if (!echo_delay2_adjusted && estimation_not_bad(delayEst)) {
    result = delayEst->get_best_delay();
    adjust_echo_delay2(result);
  }

  I("delay estimation done, result %d, elapse %dms", result, (int)timestamp(t0));
  I("got echo_delay2 %d", echo_delay2);

  fclose(fd_f);
  fclose(fd_n);
  if (delayEst) {
    delete delayEst;
    delayEst = NULL;
  }
  delay_est_thrd_stopped = true;
  last_est_time = timestamp(0);
  return 0;
}

bool estimation_not_bad(delay_estimator *d)
{
  const int MIN_SUCC = 10; // <=0 表示无限
  const int MIN_HIT = 5;
  return (MIN_SUCC <= 0
      || delayEst->succ_times >= MIN_SUCC)
    && delayEst->get_best_hit() >= MIN_HIT;
}

int pull(JNIEnv *env, jshortArray buf)
{
  static int64_t last_pull = 0;
  static int64_t warn_elapse = 0;

  if (warn_elapse == 0) {
    warn_elapse = nearend_buf_size / FRAME_SAMPS * FRAME_MS / 2;
  }

  if(last_pull > 0 && timestamp(last_pull) > warn_elapse) {
    W("pull() too slow: %"PRId64"ms since last call", timestamp(last_pull));
  }
  last_pull = timestamp(0);

  if (!nearend_buf)
    return 0;
  jshort *_buf = env->GetShortArrayElements(buf, NULL);
  jsize samps = env->GetArrayLength(buf);
  int n = read_circular_buffer(nearend_buf, _buf, samps);
  env->ReleaseShortArrayElements(buf, _buf, 0);
  return n;
}

float *to_float(short *frame)
{
  static float buf[FRAME_SAMPS];
  for (int i = 0; i < FRAME_SAMPS; ++i)
    buf[i] = (float)frame[i] / 32768;
  return buf;
}

//
// @param samps - 不需要等于 FRAME_SAMPS
int playback(short *_farend, int samps, bool with_aec_analyze)
{
  // analyze
  if (with_aec_analyze && far_est_p) {
    if (EST_BUF_CAPACITY - far_est_size >= samps) {
      memcpy(far_est_p, _farend, samps * 2);
      far_est_p += samps;
      far_est_size += samps;
    }
    if (dump_raw)
      fwrite_samps(_farend, fd_farend2, samps);
  }

  // render
  write_circular_buffer(render_hist, _farend, samps);
  write_circular_buffer(echo_buf, _farend, samps);
  if (dump_raw) {
    fwrite_samps(_farend, fd_farend, samps);
  }
  return android_AudioOut(p,_farend,samps);
}

int push(JNIEnv *env, jshortArray farend)
{
  static int64_t last_push = 0;
  static int64_t warn_elapse = 0;

  if (warn_elapse == 0) {
    warn_elapse = farend_buf_size / FRAME_SAMPS * FRAME_MS / 2;
  }

  if(last_push > 0 && timestamp(last_push) > warn_elapse) {
    W("push() too slow: %"PRId64"ms since last call", timestamp(last_push));
  }
  last_push = timestamp(0);

  if (!farend_buf)
    return 0;

  jshort *_farend = env->GetShortArrayElements(farend, NULL);
  jsize samps = env->GetArrayLength(farend);
  int rtn = write_circular_buffer(farend_buf, _farend, samps);
  env->ReleaseShortArrayElements(farend, _farend, 0);
  return rtn;
}

void speex_ec_open (int sampleRate, int bufsize, int totalSize)
{
  //init
  st = speex_echo_state_init(bufsize, totalSize);
  den = speex_preprocess_state_init(bufsize, sampleRate);
  speex_echo_ctl(st, SPEEX_ECHO_SET_SAMPLING_RATE, &sampleRate);
  speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_ECHO_STATE, st);
  int value_on = 1;
  int value_off = 0;
  // 绝对不能打开 AGC，它会让消除（减弱）后的回声变得模糊、放大。
  speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_AGC, &value_off);
  // 暂时用不到 VAD
  speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_VAD, &value_off);
  // DENOISE 稍微有点用处
  speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_DENOISE, &value_on);
}

void speex_ec_close ()
{
  if (st) {
    speex_echo_state_destroy(st);
    st = NULL;
  }
  if (den) {
    speex_preprocess_state_destroy(den);
    den = NULL;
  }
}

// 统计输出/输入信号强度。
// 如果发现输出信号明显弱于输入信号，说明AEC在起作用，这时可以刷新
// |last_est_time|，从而避免重新估算回声延迟。
void stat_ec(const short *inbuffer, const short *outbuffer, int samps)
{
  const static int duration_sec = 4;
  static int sum1 = 0, sum2 = 0;
  static int stat_samps = 0;
  for (int i = 0; i < samps; ++i) {
    sum1 += abs(inbuffer[i]);
    sum2 += abs(outbuffer[i]);
  }
  stat_samps += samps;
  if (stat_samps > FRAME_SAMPS * FRAME_RATE * duration_sec) {
    bool touched = false;
    curr_ec_ratio = (float)sum2 / sum1;
    if (curr_ec_ratio < std::max(0.3f, recent_best_ec_ratio * 1.1f)) {
      // touch timestamp
      last_est_time = std::max(last_est_time,
          timestamp(0) - duration_sec * 1000);
      touched = true;
    }
    D("stat_ec, %0.2f @%ds %s", curr_ec_ratio, (int)(timestamp(t_start) / 1000),
        (touched ? "(valid)" : ""));
    stat_samps = 0;
    sum1 = sum2 = 0;
  }
}

void offline_process()
{
  const int NN = 160;
  const int TAIL = NN * 8;
   FILE *echo_fd, *ref_fd, *e_fd;
   short echo_buf[NN], ref_buf[NN], e_buf[NN];
   int sampleRate = 8000;

   ref_fd  = fopen("sdcard/tmp/near.raw", "rb");
   echo_fd = fopen("sdcard/tmp/echo.raw", "rb");
   e_fd    = fopen("sdcard/tmp/send.raw", "wb");

   speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * SPEEX_FILTER_SIZE);

   while (!feof(ref_fd) && !feof(echo_fd))
   {
      fread(ref_buf, sizeof(short), NN, ref_fd);
      fread(echo_buf, sizeof(short), NN, echo_fd);
      speex_echo_cancellation(st, ref_buf, echo_buf, e_buf);
      speex_preprocess_run(den, e_buf);
      fwrite(e_buf, sizeof(short), NN, e_fd);
   }
   speex_ec_close();
   fclose(e_fd);
   fclose(echo_fd);
   fclose(ref_fd);
}

static jint get_api_level(JNIEnv *env)
{
    if ((env)->ExceptionCheck())
        return false; // already got an exception pending

    bool success = true;

    // VERSION is a nested class within android.os.Build (hence "$" rather than "/")
    jclass versionClass = (env)->FindClass("android/os/Build$VERSION");
    if (NULL == versionClass)
        success = false;

    jfieldID sdkIntFieldID = NULL;
    if (success)
        success = (NULL != (sdkIntFieldID = (env)->GetStaticFieldID(versionClass, "SDK_INT", "I")));

    jint sdkInt = 0;
    if (success)
    {
        sdkInt = (env)->GetStaticIntField(versionClass, sdkIntFieldID);
        __android_log_print(ANDROID_LOG_VERBOSE, TAG, "sdkInt = %d", sdkInt);
    }

    // cleanup
    (env)->DeleteLocalRef(versionClass);

    return sdkInt;
}
