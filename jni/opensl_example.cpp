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

const int MAX_DELAY = 50;
const int NEAREND_SIZE = 50; // 太短可能会得到完全错误的结果（取决于实际输入）
const int DELAY_EST_MIN_SUCC = 15; // 负数表示无限
// 回声延迟（以frame为单位）的有效值 >= 0，下面是几个特殊的无效值
const int ECHO_DELAY_NULL = -1;
const int ECHO_DELAY_FAILED = -2;

// 远端信号缓冲区。由于无法保证上层调用push()的节奏，需要此缓冲区来为OpenSL层提
// 供稳定的音频流。
// 这个缓冲区对 echo delay 不会产生影响。
#define FAREND_BUFFER_SAMPS (FRAME_SAMPS * 100)

// 处理后的近端信号。
#define NEAREND_BUFFER_SAMPS (FRAME_SAMPS * 20)

#define TAG "aec" // log tag

#define DUMP_RAW 1
#if DUMP_RAW
#define DUMP_SAMPS(p,f,n) fwrite_samps(p,f,n)
#else
#define DUMP_SAMPS(p,f,n) (0)
#endif

//--------------------------------------------------------------------------------


//--------------------------------------------------------------------------------

void cleanup();
void fwrite_samps(short *frame, FILE *fd, int samps);
void speex_ec_open (int sampleRate, int bufsize, int totalSize);
void speex_ec_close ();
int playback(short *_farend, int samps, bool with_aec_analyze);
float *to_float(short *frame);

//--------------------------------------------------------------------------------

SpeexEchoState *st = NULL;
SpeexPreprocessState *den = NULL;

delay_estimator *delayEst = NULL;
pthread_t delay_est_thrd;
bool delay_est_thrd_stopped;

static int on;
OPENSL_STREAM  *p = NULL;

// 上层输入的远端信号经过 farend_buf 缓存后再输入给 audio sytem.
// 当 farend_buf 发生 underrun （入不敷出）时，会插入一定长度的静音。
circular_buffer *farend_buf = NULL;
// echo_buf 与 farend_buf 的区别在于：
// 它的开头有|echo_delay|个 slience frame；
circular_buffer *echo_buf = NULL;
// 消除了回声的近端信号，供上层pull
circular_buffer *nearend_buf = NULL;

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
int in_buffer_cnt = 0;
int out_buffer_cnt = 0;

//----------------------------------------------------------------------

// fill_with - a frame to fill with.
void align_farend_buf(int n, short *fill_with) {
  while (n > 0) {
    if (n <= FRAME_SAMPS) {
      playback(fill_with, n, true);
      n = 0;
    } else {
      playback(fill_with, FRAME_SAMPS, true);
      n -= FRAME_SAMPS;
    }
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

void open_dump_files(const char *mode)
{
  int r = mkdir("/mnt/sdcard/tmp", 755);
  if (r == 0 || errno == EEXIST) {
    fd_farend2 = fopen("/mnt/sdcard/tmp/far2.raw", mode);
    fd_nearend2 = fopen("/mnt/sdcard/tmp/near2.raw", mode);
  } else {
    fd_farend2 = fd_nearend2 = NULL;
  }
#if DUMP_RAW
  if (r == 0 || errno == EEXIST) {
    fd_farend = fopen("/mnt/sdcard/tmp/far.raw", mode);
    fd_nearend = fopen("/mnt/sdcard/tmp/near.raw", mode);
    fd_echo = fopen("/mnt/sdcard/tmp/echo.raw", mode);
    fd_send = fopen("/mnt/sdcard/tmp/send.raw", mode);
  } else {
    fd_farend = fd_nearend = fd_echo = fd_send = NULL;
  }
#endif
}

void close_dump_files()
{
  if (fd_farend2)
    fclose(fd_farend2);
  if (fd_nearend2)
    fclose(fd_nearend2);
#if DUMP_RAW
  if (fd_farend)
    fclose(fd_farend);
  if (fd_nearend)
    fclose(fd_nearend);
  if (fd_echo)
    fclose(fd_echo);
  if (fd_send)
    fclose(fd_send);
#endif
  fd_farend = fd_farend2 = fd_nearend = fd_nearend2 = fd_echo = fd_send = NULL;
  I("dump files closed");
}

void start(jint track_min_buf_size, jint record_min_buf_size,
    jint playback_delay_ms, jint echo_delay_ms)
{
  playback_delay = playback_delay_ms / FRAME_MS;
  echo_delay2_desired = echo_delay_ms < 0;
  if (echo_delay_ms < 0) {
    //
    // 根据硬件参数估算回声延迟
    //
    // 经过测试，在满足以下条件的设备上——
    // 1, track_min_buf_size = 870
    // 2, record_min_buf_size = 4096
    // 3, OpenSL buffer samples = 160
    // 4, 主循环的 ahead 范围为 (20,60)ms ［TODO why would this matter ???]
    // 5, [ in_buffer_cnt 和 out_buffer_cnt 对延迟没有影响 ]
    // ——，回声延时大约为 20x20=400ms(+-40)，而延时是与 out_bufferframes 成正比的（参见
    // AudioTrack::getMinFrameCount 的实现）
    echo_delay = 18 * track_min_buf_size / 870;
    // 延迟修正不足没关系，因为 speex AEC 模块有个 filter length 参数，初始化为
    // 8xframe，但是修正过度就很严重，将完全不能消除回声。
    echo_delay -= 2;
    echo_delay2 = ECHO_DELAY_NULL; // 要求动态评估
  } else {
    echo_delay = echo_delay_ms / FRAME_MS;
    echo_delay2 = echo_delay;
  }

  const int sample_size = sizeof(short);
  in_buffer_cnt = ceil((float)record_min_buf_size / sample_size / FRAME_SAMPS);
  out_buffer_cnt = ceil((float)track_min_buf_size / sample_size / FRAME_SAMPS);

  // 太小的 out_buffer_cnt （比如3） 不足以保证流畅的播放。
  const int min_buffer_cnt = 20;
  if (out_buffer_cnt < min_buffer_cnt) out_buffer_cnt = min_buffer_cnt;
  // 尚未观测到 in_buffer_cnt 太小的直接影响，不过为了简化主循环中控制 ahead 时
  // 间的逻辑，这里也强制 in_buffer_cnt 不小于一个阈值。
  if (in_buffer_cnt < min_buffer_cnt) in_buffer_cnt = min_buffer_cnt;

  I("AudioRecord min buf size %dB, AudioTrack min buf size %dB",
      record_min_buf_size, track_min_buf_size);
  I("OpenSL audio in-buffer samps %dx%d, out-buffer samps %dx%d, echo delay %d(x%d=%dms)",
      FRAME_SAMPS, in_buffer_cnt, FRAME_SAMPS, out_buffer_cnt,
      echo_delay, FRAME_MS, echo_delay * FRAME_MS);

  p = android_OpenAudioDevice(SR, 1, 1, FRAME_SAMPS, in_buffer_cnt, FRAME_SAMPS, out_buffer_cnt);
  if(p == NULL) return; 

  farend_buf = create_circular_buffer(FAREND_BUFFER_SAMPS);
  nearend_buf = create_circular_buffer(NEAREND_BUFFER_SAMPS);
  echo_buf = create_circular_buffer((MAX_DELAY + playback_delay + 4) * 2 * FRAME_SAMPS);

  speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 8);

  close_dump_files(); // 假如上次运行后没有干净地结束
  open_dump_files("w+");
  on = 1;
  t_start = timestamp(0);
  delay_est_thrd_stopped = true;

  memset(silence, 0, FRAME_SAMPS * sizeof(short));
  D("start at %" PRId64 "ms" , t_start); 
}

void runNearendProcessing() 
{
  short inbuffer[FRAME_SAMPS]; // record
  short refbuf[FRAME_SAMPS];
  short processedbuffer[FRAME_SAMPS];
  short render_buf[FRAME_SAMPS];

  // delay echo_buf (relative to farend_buf)
  if (echo_delay > 0) {
    for (int i = 0; i < echo_delay; ++i)
      write_circular_buffer(echo_buf, silence, FRAME_SAMPS);
  }

  //
  // 每次循环处理一个 FRAME
  //
  //
  // 我们并不试图让每一次循环的持续时间精确为 FRAME_MS，因为这不可能做到：尽管绝
  // 大多数情况下我们只需要比如说3ms就处理完了一个frame，但是偶尔（进程调度？）
  // 也会需要明显超过 FRAME_MS 的毫秒数。
  // 所以我们能做的是在一个更大的时间跨度上让实际流逝时间与音频时间大体上保持一致
  // ，这个偏差要在相关缓冲区的承受范围之内，具体来说：
  // A. 循环不能太快，否则从 OpenSL OPENSL_STREAM->outrb 可能读不到数据，以及
  // nearend_buf 中的未读数据会被覆盖；
  // B. 但也不能太慢，否则OpenSL OPENSL_STREAM->inrb 中的未读数据会被覆盖。
  //
  // 另外，AudioSystem record buffer overflow 好像也跟这个有关。
  //
  // 把时间提前量定义为
  //    ahead = 物理时长 - 已处理的音频的时长
  int max_ahead = out_buffer_cnt / 2 * FRAME_MS;
  int min_ahead = std::max(max_ahead / 2, 1);
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
  while(on) {
    t1 = timestamp(0);
    ++loop_idx;

    // estimate echo delay
    if (rendered_samps > FRAME_SAMPS * (playback_delay + MAX_DELAY * 5)) {
      // XXX 关闭文件，否则 estimate_delay() 会阻塞在 fopen() 上。 
      if (fd_farend2)
        fclose(fd_farend2);
      if (fd_nearend2)
        fclose(fd_nearend2);
      fd_farend2 = fd_nearend2 = NULL;

      if (echo_delay2 == ECHO_DELAY_NULL) {
        if (delay_est_thrd_stopped) {
          I("start estimate_delay ");
          delay_est_thrd_stopped = false;
          pthread_create(&delay_est_thrd, NULL, (void* (*)(void*))estimate_delay, 0);
        }
      }
    }

    //
    // playback
    //
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
        D("playback underrun, lack of %d samps", lack_samps);
        // 如果用静音帧来补充，会有破音，进而对录音和回声消除产生严重影响
        align_farend_buf(lack_samps, render_buf);
        rendered_samps += lack_samps;
      }
    }
    // 平衡 rendered_samps 与物理时间
    //int64_t total_ahead = (loop_idx * FRAME_MS) - timestamp(t0);
    int64_t total_ahead = (rendered_samps / FRAME_SAMPS * FRAME_MS) - timestamp(t0);
    if (total_ahead > max_ahead) {
      total_ahead = total_ahead - min_ahead;
      //D("idle: sleep %"PRId64"ms", total_ahead);
      usleep(1000 * total_ahead);
    } else if (total_ahead < -0 * FRAME_MS) {
      E("idle: total overrun for %"PRId64"ms!", -total_ahead);
    }

    //
    // record
    //
    int lack_cap_samps = timestamp(t0) * FRAME_SAMPS / FRAME_MS - captured_samps;
    if (lack_cap_samps > FRAME_SAMPS * 2) {
      E("record overrun, lack_cap_samps %d", lack_cap_samps);
    }
    while (lack_cap_samps >= FRAME_SAMPS) {
      int samps = android_AudioIn(p,inbuffer,FRAME_SAMPS);
      if (samps == FRAME_SAMPS) {
        if (captured_samps == 0) 
          D("first time of capture at @%"PRId64"ms", timestamp(t0));
        captured_samps += samps;
        lack_cap_samps -= samps;

        // adjust echo buffer, and read a frame for current AEC
        if (echo_delay2 >= 0 && echo_delay2 != echo_delay) {
          I("adjust echo buffer: %d=>%d", echo_delay, echo_delay2);
          if (echo_delay >= 0) {
            if (echo_delay2 > echo_delay + 2) {
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
          speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 8);
          echo_delay = echo_delay2;
        }
        read_circular_buffer(echo_buf, refbuf, samps);

        short *out = inbuffer; // output(send) frame
        if (loop_idx < playback_delay + echo_delay || echo_delay <= 0) {
          // output as-is
        } else {
          // do AEC
          speex_echo_cancellation(st, inbuffer, refbuf, processedbuffer);
          speex_preprocess_run(den, processedbuffer);
          out = processedbuffer;
          DUMP_SAMPS(inbuffer, fd_nearend, samps);
          DUMP_SAMPS(refbuf, fd_echo, samps);
          DUMP_SAMPS(out, fd_send, samps);
        }
        // output
        write_circular_buffer(nearend_buf, out, samps);
        if (captured_samps > playback_delay * FRAME_SAMPS && fd_nearend2)
          fwrite_samps(inbuffer, fd_nearend2, samps);
      } else {
        D("record nothing at @%"PRId64"ms", timestamp(t0));
        break;
      }
    }

    // log time elapse
    if (1) {
      int64_t curr_ahead = FRAME_MS - timestamp(t1);
      if (curr_ahead > 0) {
        D("idle: curr %"PRId64"ms", curr_ahead);
      } else {
        D("idle: curr overrun for %"PRId64"ms!", -curr_ahead);
      }
    }
  }

  cleanup();
}

int get_estimated_echo_delay()
{
  return echo_delay2 >= 0 ? echo_delay2 * FRAME_MS : -1;
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
  close_dump_files();
  speex_ec_close();

  if (delayEst) {
    delete delayEst;
    delayEst = NULL;
  }
  I("clean up");
}

int estimate_delay(int async)
{
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
  fseek(fd_n, i * FRAME_SAMPS * 2, SEEK_SET);
  for (; /*i < MAX_DELAY * 40*/; ++i)
  {
    if (FRAME_SAMPS != fread(far, 2, FRAME_SAMPS, fd_f))
      break;
    delayEst->add_far(far, FRAME_SAMPS);

    if (FRAME_SAMPS != fread(near, 2, FRAME_SAMPS, fd_n))
      break;

    delayEst->add_near(near, FRAME_SAMPS);

    // 减少搜索次数，希望不会影响结果的正确性
    if (i % 2 == 0)
      continue;

    int hint = delayEst->get_largest_delay() > 0
      ? delayEst->get_largest_delay() : echo_delay + 2;
    if (!async) {
      // sync call
      result = delayEst->process(hint);
    } else {
      // async call
      if (delayEst->get_near_samps() > NEAREND_SIZE) {
        delayEst->process_async(hint);
      } else {
        D("near samps %d", delayEst->get_near_samps());
      }
      usleep(FRAME_MS * 1000);
    }

    // 刷新输出结果，但不放弃搜索
    if (echo_delay2_desired && DELAY_EST_MIN_SUCC >= 0
        && (result >= 0 || (result = delayEst->get_best_delay()) >= 0)
        && delayEst->succ_times > DELAY_EST_MIN_SUCC) {
      echo_delay2 = result - 2;
    }
  }

  // 为保险起见，倾向于选择偏小一点的值
#if 0
  int r2 = delayEst->get_2nd_best_delay();
  if (r2 >= 0 && r2 >= result - 2)
    echo_delay2 = result - 2;
  else
    echo_delay2 = result;
#else
  echo_delay2 = result - 2;
#endif
  if (echo_delay2 < 0) echo_delay2 = ECHO_DELAY_FAILED;

  I("delay estimation done, result %d, elapse %dms", result, (int)timestamp(t0));
  I("got echo_delay2 %d", echo_delay2);

  fclose(fd_f);
  fclose(fd_n);
  if (delayEst) {
    delete delayEst;
    delayEst = NULL;
  }
  delay_est_thrd_stopped = true;
  return 0;
}

int pull(JNIEnv *env, jshortArray buf)
{
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
  if (with_aec_analyze) {
    fwrite_samps(_farend, fd_farend2, samps);
  }

  // render
  write_circular_buffer(echo_buf, _farend, samps);
  DUMP_SAMPS(_farend, fd_farend, samps);
  return android_AudioOut(p,_farend,samps);
}

int push(JNIEnv *env, jshortArray farend)
{
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
  int value_off = 1;
  speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_AGC, &value_on);
  speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_VAD, &value_on);
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

void offline_process()
{
  const int NN = 160;
  const int TAIL = NN * 8;
   FILE *echo_fd, *ref_fd, *e_fd;
   short echo_buf[NN], ref_buf[NN], e_buf[NN];
   SpeexEchoState *st;
   SpeexPreprocessState *den;
   int sampleRate = 8000;

   ref_fd  = fopen("sdcard/tmp/near.raw", "rb");
   echo_fd = fopen("sdcard/tmp/echo.raw", "rb");
   e_fd    = fopen("sdcard/tmp/send.raw", "wb");

   st = speex_echo_state_init(NN, TAIL);
   den = speex_preprocess_state_init(NN, sampleRate);
   speex_echo_ctl(st, SPEEX_ECHO_SET_SAMPLING_RATE, &sampleRate);
   speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_ECHO_STATE, st);

   while (!feof(ref_fd) && !feof(echo_fd))
   {
      fread(ref_buf, sizeof(short), NN, ref_fd);
      fread(echo_buf, sizeof(short), NN, echo_fd);
      speex_echo_cancellation(st, ref_buf, echo_buf, e_buf);
      speex_preprocess_run(den, e_buf);
      fwrite(e_buf, sizeof(short), NN, e_fd);
   }
   speex_echo_state_destroy(st);
   speex_preprocess_state_destroy(den);
   fclose(e_fd);
   fclose(echo_fd);
   fclose(ref_fd);
}
