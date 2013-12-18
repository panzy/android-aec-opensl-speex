/*
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
const int NEAREND_SIZE = 10;

// 远端信号缓冲区。由于无法保证上层调用push()的节奏，需要此缓冲区来为OpenSL层提
// 供稳定的音频流。
// 这个缓冲区对 echo delay 不会产生影响。
#define FAREND_BUFFER_SAMPS (FRAME_SAMPS * 100)

// 处理后的近端信号。
#define NEAREND_BUFFER_SAMPS (FRAME_SAMPS * 20)

#define TAG "aec" // log tag

#define DUMP_PCM true
#if DUMP_PCM
#define dump_audio(p,f,n) dump_audio_(p,f,n)
#else
#define dump_audio(p,f,n) do {} while (0);
#endif

//--------------------------------------------------------------------------------


//--------------------------------------------------------------------------------

void cleanup();
void dump_audio_(short *frame, FILE *fd, int samps);
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
FILE *fd_nearend = NULL;
FILE *fd_echo = NULL;
FILE *fd_send = NULL;

short silence[FRAME_SAMPS];

int playback_delay = 0; // samps
int echo_delay = 0; // samps, relative to playback
int echo_delay2 = -1; // samps, relative to playback, estimated by WebRtc 
int in_buffer_cnt = 0;
int out_buffer_cnt = 0;

//----------------------------------------------------------------------

void align_farend_buf(int n) {
  while (n > 0) {
    if (n <= FRAME_SAMPS) {
      playback(silence, n, true);
      n = 0;
    } else {
      playback(silence, FRAME_SAMPS, true);
      n -= FRAME_SAMPS;
    }
  }
}

void align_nearend_buf(int n) {
  while (n > 0) {
    if (n <= FRAME_SAMPS) {
      write_circular_buffer(nearend_buf, silence, n);
      dump_audio(silence, fd_nearend, n);
      n = 0;
    } else {
      write_circular_buffer(nearend_buf, silence, FRAME_SAMPS);
      dump_audio(silence, fd_nearend, FRAME_SAMPS);
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

void dump_audio_(short *frame, FILE *fd, int samps)
{
  if (fd) {
    fwrite(frame, samps, 2, fd);
  }
}

void open_dump_files(const char *mode)
{
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

void close_dump_files()
{
  if (fd_farend)
    fclose(fd_farend);
  if (fd_nearend)
    fclose(fd_nearend);
  if (fd_echo)
    fclose(fd_echo);
  if (fd_send)
    fclose(fd_send);
  fd_farend = fd_send = fd_nearend = NULL;
}

void start(jint track_min_buf_size, jint record_min_buf_size, jint playback_delay_ms)
{
  playback_delay = playback_delay_ms / FRAME_MS;

  //
  // 估算回声延迟
  //
  // XXX 以下方法适用于 Huawei U8860 和 MOTO XOOM，但不适用于 Xiaomi 1S/2S。
  //
  // 观测数据
  // ============================================================
  // device       track(B)  record(B)   echo_delay(ms)
  // ------------------------------------------------------------
  // xiaomi 1s    870       640         340
  // xiaomi 2s    1364      640         800
  // hw u8860     870       4096        400
  // xoom         1486      640         720
  // htc          1486      8192        900
  // ------------------------------------------------------------
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
  echo_delay -= 4;

  const int sample_size = sizeof(short);
  in_buffer_cnt = ceil((float)record_min_buf_size / sample_size / FRAME_SAMPS);
  out_buffer_cnt = ceil((float)track_min_buf_size / sample_size / FRAME_SAMPS);

  // 太小的 out_buffer_cnt （比如3） 不足以保证流畅的播放。
  const int min_buffer_cnt = 10;
  if (out_buffer_cnt < min_buffer_cnt) out_buffer_cnt = min_buffer_cnt;
  // 尚未观测到 in_buffer_cnt 太小的直接影响，不过为了简化主循环中控制 ahead 时
  // 间的逻辑，这里也强制 in_buffer_cnt 不小于一个阈值。
  if (in_buffer_cnt < min_buffer_cnt) in_buffer_cnt = min_buffer_cnt;

  D("AudioRecord min buf size %dB, AudioTrack min buf size %dB",
      record_min_buf_size, track_min_buf_size);
  D("OpenSL audio in-buffer samps %dx%d, out-buffer samps %dx%d, echo delay %d(x%d=%dms)",
      FRAME_SAMPS, in_buffer_cnt, FRAME_SAMPS, out_buffer_cnt,
      echo_delay, FRAME_MS, echo_delay * FRAME_MS);

  p = android_OpenAudioDevice(SR, 1, 1, FRAME_SAMPS, in_buffer_cnt, FRAME_SAMPS, out_buffer_cnt);
  if(p == NULL) return; 

  farend_buf = create_circular_buffer(FAREND_BUFFER_SAMPS);
  nearend_buf = create_circular_buffer(NEAREND_BUFFER_SAMPS);
  echo_buf = create_circular_buffer((echo_delay + playback_delay + 4) * 2 * FRAME_SAMPS);

  speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 8);

#if DUMP_PCM
  open_dump_files("w+");
#endif
  on = 1;
  t_start = timestamp(0);
  echo_delay2 = -1; // 每次都重新评估
  delay_est_thrd_stopped = true;

  memset(silence, 0, FRAME_SAMPS * sizeof(short));
  D("start at %" PRId64 "ms" , t_start); 
}

void runNearendProcessing() 
{
  short inbuffer[FRAME_SAMPS]; // record
  short refbuf[FRAME_SAMPS];
  short processedbuffer[FRAME_SAMPS];

  //if (echo_delay2 < 0 && !delayEst) {
  //  delayEst = new delay_estimator(SR, FRAME_SAMPS, MAX_DELAY, NEAREND_SIZE );
  //}

  // delay echo_buf (relative to farend_buf)
  for (int i = 0; i < echo_delay; ++i)
    write_circular_buffer(echo_buf, silence, FRAME_SAMPS);

  //
  // 每次循环处理一个 FRAME
  //
  int loop_idx = -1;
  int64_t t0 = timestamp(0), t1 = 0; // loop body begins
  int64_t total_ahead = 0; // 累计剩余时间
  int rendered_samps = 0;
  int captured_samps = 0;
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
  // XXX 另外，AudioSystem record buffer overflow 好像也跟这个有关。
  //
  // 把时间提前量定义为
  //    ahead = 物理时长 - 已处理的音频的时长
  int max_ahead = out_buffer_cnt / 2 * FRAME_MS;
  int min_ahead = std::max(max_ahead / 2, 1);
  // ……不管怎样，下面的硬编码的值的实际效果更好
  max_ahead = 3 * FRAME_MS;
  min_ahead = 1 * FRAME_MS;
  D("main loop time ahead target range: (%d,%d)ms, or (%d,%d)frames",
      min_ahead, max_ahead, min_ahead / FRAME_MS, max_ahead / FRAME_MS);
  while(on) {
    t1 = timestamp(0);
    ++loop_idx;

    // estimate echo delay
    if (echo_delay2 < 0 && loop_idx > playback_delay + MAX_DELAY * 5) {
      if (delay_est_thrd_stopped) {
        D("start estimate_delay ");
        close_dump_files(); // XXX 否则 estimate_delay() 会阻塞在 fopen() 上。 
        delay_est_thrd_stopped = false;
        pthread_create(&delay_est_thrd, NULL, (void* (*)(void*))estimate_delay, 0);
      }

      //if (delayEst->succ_times > 2) {
      //  echo_delay2 = delayEst->get_best_delay();
      //  D("got echo_delay2 %d", echo_delay2);
      //}
    }

    // playback
    if (loop_idx < playback_delay) {
      playback(silence, FRAME_SAMPS, true /* XXX test false */);
      rendered_samps += FRAME_SAMPS;
    } else {
      int samps = read_circular_buffer(farend_buf, inbuffer, FRAME_SAMPS);
      if (samps > 0) {
        playback(inbuffer, samps, true);
        rendered_samps += samps;
      }
      // pad underrun with silence
      // |playback_delay| 可以显著减少这种情况的发生
      int lack_samps = timestamp(t0) * FRAME_SAMPS / FRAME_MS - rendered_samps;
      if (lack_samps > 0) {
        D("playback underrun, lack of %d samps", lack_samps);
        align_farend_buf(lack_samps);
        rendered_samps += lack_samps;
      }
    }

    // record
    int samps = android_AudioIn(p,inbuffer,FRAME_SAMPS);
    if (samps == FRAME_SAMPS) {
      if (captured_samps == 0) 
        D("first time of capture at @%"PRId64"ms", timestamp(t0));
      captured_samps += samps;

      dump_audio(inbuffer, fd_nearend, samps);

      short *out = inbuffer; // output(send) frame
      read_circular_buffer(echo_buf, refbuf, samps);
      if (true || loop_idx < playback_delay + echo_delay) {
        // output as-is
      } else {
        // do AEC
        speex_echo_cancellation(st, inbuffer, refbuf, processedbuffer);
        speex_preprocess_run(den, processedbuffer);
        out = processedbuffer;
      }
      // output
      write_circular_buffer(nearend_buf, out, samps);
      dump_audio(out, fd_send, samps);
      dump_audio(refbuf, fd_echo, samps);
    } else {
      D("record nothing at @%"PRId64"ms", timestamp(t0));
    }

    //
    // idle
    //
    // log current idle
    if (0) {
      int64_t curr_ahead = FRAME_MS - timestamp(t1);
      if (curr_ahead > 0) {
        D("idle: %"PRId64"ms", curr_ahead);
      } else {
        D("idle: overrun for %"PRId64"ms!", -curr_ahead);
      }
    }
    // balance total idle
    total_ahead = (loop_idx * FRAME_MS) - timestamp(t0);
    if (total_ahead > max_ahead) {
      total_ahead = total_ahead - min_ahead;
      //D("idle: sleep %"PRId64"ms", total_ahead);
      usleep(1000 * total_ahead);
    }
  }

  cleanup();
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
#if DUMP_PCM
  close_dump_files();
#endif
  speex_ec_close();

  if (delayEst) {
    delete delayEst;
    delayEst = NULL;
  }
}

int estimate_delay(int async)
{
  short far[MAX_DELAY * FRAME_SAMPS];
  short near[FRAME_SAMPS];
  delayEst = new delay_estimator(SR, FRAME_SAMPS, MAX_DELAY, NEAREND_SIZE );

  FILE *fd_f = fopen("/mnt/sdcard/tmp/far.raw", "r");
  FILE *fd_n = fopen("/mnt/sdcard/tmp/near.raw", "r");
  if (!fd_f || !fd_n) {
      E("failed to open dump files at %d", __LINE__ );
      return 0;
  }

  int64_t t0 = timestamp(0);
  int i = 0;
  int result = -1;
  fseek(fd_n, i * FRAME_SAMPS * 2, SEEK_SET);
  for (; /*i < MAX_DELAY * 40*/; ++i)
  {
    if(FRAME_SAMPS != fread(far, 2, FRAME_SAMPS, fd_f))
      break;
    delayEst->add_far(far, FRAME_SAMPS);

    fread(near, 2, FRAME_SAMPS, fd_n);

    delayEst->add_near(near, FRAME_SAMPS);

    if (!async) {
      // sync call
      result = delayEst->process(12);
    } else {
      // async call
      if (delayEst->get_near_samps() > NEAREND_SIZE) {
        delayEst->process_async(12);
        //usleep(100 * 1000); // TODO need to fix
      } else {
        D("near samps %d", delayEst->get_near_samps());
      }
    }

    usleep(FRAME_MS * 1000);

    if ((result >= 0 || (result = delayEst->get_best_delay()) >= 0) && delayEst->succ_times > 1) {
      break;
    }
  }

  I("delay esitmation done, result %d, elapse %dms", result, (int)timestamp(t0));

  fclose(fd_f);
  fclose(fd_n);
  if (delayEst) {
    delete delayEst;
    delayEst = NULL;
  }

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
    write_circular_buffer(echo_buf, _farend, samps);
    //if (delayEst) {
    //  delayEst->add_far(_farend, samps);
    //}
  }

  // render
  dump_audio(_farend, fd_farend, samps);
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

double getTimestamp()
{
  return android_GetTimestamp(p);
}

void speex_ec_open (int sampleRate, int bufsize, int totalSize)
{
  //init
  st = speex_echo_state_init(bufsize, totalSize);
  den = speex_preprocess_state_init(bufsize, sampleRate);
  speex_echo_ctl(st, SPEEX_ECHO_SET_SAMPLING_RATE, &sampleRate);
  speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_ECHO_STATE, st);
  int value = 1;
  //speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_AGC, &value);
  //speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_VAD, &value);
  speex_preprocess_ctl(den, SPEEX_PREPROCESS_SET_DENOISE, &value);
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

