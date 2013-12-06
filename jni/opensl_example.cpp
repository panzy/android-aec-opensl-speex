/*
opensl_example.c:
OpenSL example module 
Copyright (c) 2012, Victor Lazzarini
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <jni.h>
#include <android/log.h>
#include <sys/time.h>
#include <assert.h>
#include <queue>
#include <stdio.h>
#include <inttypes.h>
#include <unistd.h>
#include "opensl_io2.h"

#include "speex/speex_echo.h"
#include "speex/speex_preprocess.h"

#include "opensl_example.h"

// from <inttypes.h>
#define	PRId64			"lld"		/* int64_t */

#define SR 8000 // sample rate

// AEC frame, 10ms
#define FRAME_SAMPS 160
#define FRAME_MS (1000 * FRAME_SAMPS / SR)

// 远端信号缓冲区。由于无法保证上层调用push()的节奏，需要此缓冲区来为OpenSL层提
// 供稳定的音频流。
#define FAREND_BUFFER_SAMPS (FRAME_SAMPS * 150)
#define FAREND_BUFFER_SAMPS_DELAY (FRAME_SAMPS * 70)

// 提交给 OpenSL player buffer queue 的 buffer 的长度，
#define OPENSL_BUFFER_SAMPS (FRAME_SAMPS * 40)

// 回声信号缓冲区，其长度要能容纳这个延迟：声音从被播放到被录音。
#define ECHO_BUFFER_SAMPS (FRAME_SAMPS * 150)

// 处理后的近端信号。
#define NEAREND_BUFFER_SAMPS (FRAME_SAMPS * 10)

#define TAG "aec" // log tag

//--------------------------------------------------------------------------------

void speex_ec_open (int sampleRate, int bufsize, int totalSize);
void speex_ec_close ();
int push_helper(short *_farend, int samps);

//--------------------------------------------------------------------------------

SpeexEchoState *st;
SpeexPreprocessState *den;

static int on;
OPENSL_STREAM  *p = NULL;

circular_buffer *farend_buf = NULL;
circular_buffer *nearend_buf = NULL;
circular_buffer *echo_buf = NULL;

void *aecm = NULL;

int64_t t_start;

int farend_duration = 0; // duration of pushed audio in ms

// dump audio data
FILE *fd_farend = NULL;
FILE *fd_send = NULL;
FILE *fd_nearend = NULL;

short silence[FRAME_SAMPS];

//----------------------------------------------------------------------

// get timestamp of today in ms.
int64_t timestamp(int64_t base)
{
  timeval t;
  gettimeofday(&t, NULL);
  return ((int64_t)t.tv_sec * 1000 + (int64_t)(t.tv_usec / 1000)) - base;
}

void dump_audio(short *frame, FILE *fd, int samps)
{
  fwrite(frame, samps, 2, fd);
}

void start()
{
  farend_buf = create_circular_buffer(FAREND_BUFFER_SAMPS);
  nearend_buf = create_circular_buffer(NEAREND_BUFFER_SAMPS);
  echo_buf = create_circular_buffer(ECHO_BUFFER_SAMPS);
  speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 8);
  p = android_OpenAudioDevice(SR,1,1,OPENSL_BUFFER_SAMPS);

  if(p == NULL) return; 

  on = 1;
  fd_farend = fopen("/mnt/sdcard/tmp/far.dat", "w+");
  fd_send = fopen("/mnt/sdcard/tmp/send.dat", "w+");
  fd_nearend = fopen("/mnt/sdcard/tmp/near.dat", "w+");
  t_start = timestamp(0);
  farend_duration = 0;
  memset(silence, 0, FRAME_SAMPS * sizeof(short));
  __android_log_print(ANDROID_LOG_DEBUG, TAG, "start, %" PRId64 , t_start); 
}

void runNearendProcessing() 
{
  short inbuffer[FRAME_SAMPS]; // record
  short refbuf[FRAME_SAMPS];
  short processedbuffer[FRAME_SAMPS];

  // delay between play and rec in frames
  int delay = // (FAREND_BUFFER_SAMPS_DELAY / FRAME_SAMPS) // farend buffer 引入的播放延迟
    + 3 * (OPENSL_BUFFER_SAMPS / FRAME_SAMPS) // opensl rec buffer 引入的录音延迟
    + 12 // 硬件延迟
    ;
  if (delay >= ECHO_BUFFER_SAMPS / FRAME_SAMPS) {
    __android_log_print(ANDROID_LOG_ERROR, TAG, "echo buffer is insufficiency, actual delay %d frames", delay);
  }

  //
  // 每次循环处理一个 FRAME
  //
  int loop_idx = 0;
  int64_t t0 = timestamp(0), t1 = 0; // loop body begins
  int64_t total_ahead = 0; // 累计剩余时间
  int max_ahead = NEAREND_BUFFER_SAMPS / FRAME_SAMPS * FRAME_MS;
  int min_ahead = max_ahead / 2;
  while(on) {
    t1 = timestamp(0);
    ++loop_idx;

    // playback
    if (loop_idx < FAREND_BUFFER_SAMPS_DELAY / FRAME_SAMPS) {
      push_helper(silence, FRAME_SAMPS);
    } else {
      int samps = read_circular_buffer(farend_buf, inbuffer, FRAME_SAMPS);
      if (samps > 0) {
        push_helper(inbuffer, samps);
      }
      // pad underrun with silence
      if (FRAME_SAMPS - samps > 0) {
        push_helper(silence, FRAME_SAMPS -samps);
      }
    }

    // record
    int samps = android_AudioIn(p,inbuffer,FRAME_SAMPS);
    if (samps == FRAME_SAMPS) {
      if (loop_idx < delay) {
        // echo does not occur yet
        write_circular_buffer(nearend_buf, inbuffer, FRAME_SAMPS);
        dump_audio(inbuffer, fd_send, FRAME_SAMPS);
      } else {
        // do AEC
        dump_audio(inbuffer, fd_nearend, FRAME_SAMPS);
        read_circular_buffer(echo_buf, refbuf, FRAME_SAMPS);
        speex_echo_cancellation(st, inbuffer, refbuf, processedbuffer);
        speex_preprocess_run(den, processedbuffer);
        write_circular_buffer(nearend_buf, processedbuffer, FRAME_SAMPS);
        dump_audio(processedbuffer, fd_send, FRAME_SAMPS);
      }
    }

    //
    // idle
    //
    // log current idle
    if (0) {
      int64_t curr_ahead = FRAME_MS - timestamp(t1);
      if (curr_ahead > 0) {
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "idle: %"PRId64"ms", curr_ahead);
      } else {
        __android_log_print(ANDROID_LOG_DEBUG, TAG, "idle: overrun for %"PRId64"ms!", -curr_ahead);
      }
    }
    // balance total idle
    total_ahead = (loop_idx * FRAME_MS) - timestamp(t0);
    if (total_ahead > max_ahead) {
      total_ahead = total_ahead - min_ahead;
      //__android_log_print(ANDROID_LOG_DEBUG, TAG, "idle: sleep %"PRId64"ms", total_ahead);
      usleep(1000 * total_ahead);
    }
  }

  android_CloseAudioDevice(p);
}

void runUnderrunCompensation()
{
  int time_slice = FRAME_MS;
  int slice_samps = FRAME_SAMPS;
  while (on) {
    int64_t elapsed = timestamp(t_start);
    int64_t delta = elapsed - farend_duration;
    if (delta > FRAME_MS) {
      __android_log_print(ANDROID_LOG_WARN, TAG, "playback underrun, time elapsed: %" PRId64 "ms, audio duration: %dms, gap %" PRId64 "ms", 
          elapsed, farend_duration, delta);
      farend_duration += time_slice;
      android_AudioOut(p, silence, slice_samps);
      write_circular_buffer(echo_buf, silence, slice_samps);
      dump_audio(silence, fd_farend, slice_samps);
    }
    usleep(time_slice * 1000);
  }
}

void stop()
{
  on = 0;
  free_circular_buffer(farend_buf);
  free_circular_buffer(echo_buf);
  free_circular_buffer(nearend_buf);
  fclose(fd_farend);
  fclose(fd_send);
  fclose(fd_nearend);
  fd_farend = fd_send = fd_nearend = NULL;
  speex_ec_close();
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

int push_helper(short *_farend, int samps)
{
  // TODO thread-safe
  if (!echo_buf)
    return 0;

  farend_duration += samps * FRAME_MS / FRAME_SAMPS;

  int rtn = samps;

  // analyze
  write_circular_buffer(echo_buf, _farend, samps);
  dump_audio(_farend, fd_farend, samps);

  // render
  rtn = android_AudioOut(p,_farend,samps);

  return rtn;
}

int push(JNIEnv *env, jshortArray farend)
{
  if (!echo_buf)
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

