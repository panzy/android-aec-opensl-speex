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
#include "opensl_io.h"

#include "speex/speex_echo.h"
#include "speex/speex_preprocess.h"

#include "opensl_example.h"

#define SR 8000 // sample rate
#define FRAME_SAMPS 160
#define FRAME_MS (1000 * FRAME_SAMPS / SR)
#define BUFFERFRAMES (FRAME_SAMPS * 50) // queue size in samples

//--------------------------------------------------------------------------------

void speex_ec_open (int sampleRate, int bufsize, int totalSize);
void speex_ec_close ();

//--------------------------------------------------------------------------------

SpeexEchoState *st;
SpeexPreprocessState *den;

typedef long long int64;

static int on;
OPENSL_STREAM  *p = NULL;

circular_buffer *recbuf = NULL;
circular_buffer *playbuf = NULL;

void *aecm = NULL;

int64 t_start;
std::queue<int64> t_render;
std::queue<int64> t_analyze;

FILE *fd_out = NULL;
FILE *fd_mic = NULL;

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
  playbuf = create_circular_buffer(BUFFERFRAMES * 3);

  speex_ec_open(SR, FRAME_SAMPS, FRAME_SAMPS * 8);

  p = android_OpenAudioDevice(SR,1,1,BUFFERFRAMES);

  if(p == NULL) return; 
  fd_out = fopen("/mnt/sdcard/tmp/out.dat", "w+");
  fd_mic = fopen("/mnt/sdcard/tmp/mic.dat", "w+");
  t_start = timestamp(0) - 2000;
  on = 1;
}

void run() 
{
  int samps, i, j;
  short inbuffer[FRAME_SAMPS]; // record
  short refbuf[FRAME_SAMPS];
  short processedbuffer[FRAME_SAMPS];

  // delay between play and rec in samples
  int delay = (BUFFERFRAMES / FRAME_SAMPS) /* recorder buffer queue */
    + 2 /* play latency */
    + 10 /* extra play latency */
    ;
  i = 0;
  while(on) {
    samps = android_AudioIn(p,inbuffer,FRAME_SAMPS);

    // discard head frames
    if (i++ < delay) continue;

    dump_audio(inbuffer, fd_mic);
    read_circular_buffer(playbuf, refbuf, FRAME_SAMPS);
    speex_echo_cancellation(st, inbuffer, refbuf, processedbuffer);
    speex_preprocess_run(den, processedbuffer);
    dump_audio(processedbuffer, fd_out);
    write_circular_buffer(recbuf, processedbuffer, FRAME_SAMPS);
  }  

  android_CloseAudioDevice(p);
}

void close()
{
  on = 0;
  fclose(fd_out);
  fclose(fd_mic);
  speex_ec_close();
}

int pull(JNIEnv *env, jshortArray buf)
{
  jshort *_buf = env->GetShortArrayElements(buf, NULL);
  int n = read_circular_buffer(recbuf, _buf, FRAME_SAMPS);
  env->ReleaseShortArrayElements(buf, _buf, 0);
  return n;
}

int push(JNIEnv *env, jshortArray farend)
{
  jshort *_farend = env->GetShortArrayElements(farend, NULL);
  jsize samps = env->GetArrayLength(farend);
  int rtn = samps;

  // analyze
  t_analyze.push(timestamp(t_start));
  write_circular_buffer(playbuf, _farend, samps);

  // render
  t_render.push(timestamp(t_start) + BUFFERFRAMES / FRAME_SAMPS * FRAME_MS);
  if (android_AudioOut(p,_farend,samps) != samps)
    rtn = 0; 

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
  speex_echo_state_destroy(st);
  speex_preprocess_state_destroy(den);
}

