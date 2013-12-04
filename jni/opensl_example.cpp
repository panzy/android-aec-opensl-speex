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
#include <unistd.h>
#include "opensl_io2.h"

#include "speex/speex_echo.h"
#include "speex/speex_preprocess.h"

#include "opensl_example.h"

#define SR 8000 // sample rate
#define FRAME_SAMPS 160
#define FRAME_MS (1000 * FRAME_SAMPS / SR)
#define BUFFERFRAMES (FRAME_SAMPS * 20) // queue size in samples

#define TAG "aec" // log tag

typedef long long int64;

//--------------------------------------------------------------------------------

void speex_ec_open (int sampleRate, int bufsize, int totalSize);
void speex_ec_close ();
int push_helper(short *_farend);

//--------------------------------------------------------------------------------

SpeexEchoState *st;
SpeexPreprocessState *den;

static int on;
OPENSL_STREAM  *p = NULL;

circular_buffer *recbuf = NULL;
circular_buffer *playbuf = NULL;

void *aecm = NULL;

int64 t_start;
std::queue<int64> t_render;
std::queue<int64> t_analyze;

int farend_duration = 0; // duration of pushed audio in ms
int64 t_first_push = 0;

// dump audio data
FILE *fd_farend = NULL;
FILE *fd_send = NULL;
FILE *fd_nearend = NULL;

short silence[FRAME_SAMPS];

//----------------------------------------------------------------------

// get timestamp of today in ms.
int64 timestamp(int64 base)
{
  timeval t;
  gettimeofday(&t, NULL);
  return ((int64)t.tv_sec * 1000 + (int64)(t.tv_usec / 1000)) - base;
}

void dump_audio(short *frame, FILE *fd)
{
  //static float buf[FRAME_SAMPS];
  //for (int i = 0; i < FRAME_SAMPS; ++i) {
  //  buf[i] = frame[i] / 32768.0;
  //}
  //fwrite(buf, FRAME_SAMPS, 4, fd);

  fwrite(frame, FRAME_SAMPS, 2, fd);
}

void init()
{
  recbuf = create_circular_buffer(BUFFERFRAMES);
  playbuf = create_circular_buffer(FRAME_SAMPS * 100);

  speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 8);

  p = android_OpenAudioDevice(SR,1,1,BUFFERFRAMES);

  if(p == NULL) return; 
  fd_farend = fopen("/mnt/sdcard/tmp/far.dat", "w+");
  fd_send = fopen("/mnt/sdcard/tmp/send.dat", "w+");
  fd_nearend = fopen("/mnt/sdcard/tmp/near.dat", "w+");
  t_start = timestamp(0) - 2000;
  farend_duration = 0;
  t_first_push = 0;
  memset(silence, 0, FRAME_SAMPS * sizeof(short));
  on = 1;
}

void run() 
{
  short inbuffer[FRAME_SAMPS]; // record
  short refbuf[FRAME_SAMPS];
  short processedbuffer[FRAME_SAMPS];

  // delay between play and rec in samples
  int delay = (BUFFERFRAMES / FRAME_SAMPS) /* recorder buffer queue */
    + 2 /* play latency */
    //+ 80 /* extra play latency */
    ;
  delay = 60; // 在 xoom 上测试得到延迟大约为 1250ms。
  int loopIdx = 0;
  while(on) {
    int samps = android_AudioIn(p,inbuffer,FRAME_SAMPS);

    if (samps != FRAME_SAMPS) continue;

    if (loopIdx++ < delay) {
      // echo does not occur yet
      write_circular_buffer(recbuf, inbuffer, FRAME_SAMPS);
      dump_audio(inbuffer, fd_send);
    } else {
      // do AEC
      dump_audio(inbuffer, fd_nearend);
      read_circular_buffer(playbuf, refbuf, FRAME_SAMPS);
      speex_echo_cancellation(st, inbuffer, refbuf, processedbuffer);
      speex_preprocess_run(den, processedbuffer);
      write_circular_buffer(recbuf, processedbuffer, FRAME_SAMPS);
      dump_audio(processedbuffer, fd_send);
    }
  }  

  android_CloseAudioDevice(p);
}

void startUnderrunCompensate()
{
  while (on) {
    int64 now = timestamp(0);
    int64 delta;
    if (t_first_push > 0 && (delta = now - t_first_push - farend_duration) > 0) {
      __android_log_print(ANDROID_LOG_WARN, TAG, "playback underrun, time elapsed: %lldms, audio duration: %dms, gap %lldms", 
          now - t_first_push, farend_duration, delta);
      int n = delta / FRAME_MS;
      for (int j = 0; j < n; ++j)
        push_helper(silence);
    }
    sleep(FRAME_MS / 2);
  }
}

void close()
{
  on = 0;
  fclose(fd_farend);
  fclose(fd_send);
  fclose(fd_nearend);
  fd_farend = fd_send = fd_nearend = NULL;
  speex_ec_close();
}

int pull(JNIEnv *env, jshortArray buf)
{
  if (!recbuf)
    return 0;
  jshort *_buf = env->GetShortArrayElements(buf, NULL);
  int n = read_circular_buffer(recbuf, _buf, FRAME_SAMPS);
  env->ReleaseShortArrayElements(buf, _buf, 0);
  return n;
}

int push_helper(short *_farend)
{
  // TODO thread-safe
  if (!playbuf)
    return 0;

  if (t_first_push == 0) {
    t_first_push = timestamp(0);
  }
  farend_duration += FRAME_MS;

  int rtn = FRAME_SAMPS;

  // analyze
  t_analyze.push(timestamp(t_start));
  write_circular_buffer(playbuf, _farend, FRAME_SAMPS);
  dump_audio(_farend, fd_farend);

  // render
  t_render.push(timestamp(t_start) + BUFFERFRAMES / FRAME_SAMPS * FRAME_MS);
  if (android_AudioOut(p,_farend,FRAME_SAMPS) != FRAME_SAMPS)
    rtn = 0; 

  return rtn;
}

int push(JNIEnv *env, jshortArray farend)
{
  if (!playbuf)
    return 0;

  jshort *_farend = env->GetShortArrayElements(farend, NULL);
  jsize samps = env->GetArrayLength(farend);
  int rtn = samps == FRAME_SAMPS ? push_helper(_farend) : 0;
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

