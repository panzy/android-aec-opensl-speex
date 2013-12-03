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
#include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"
#include "webrtc/modules/audio_processing/ns/ns_core.h"
#include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"

#define VECSAMPS_MONO 80 // frame size in samples
#define BUFFERFRAMES (VECSAMPS_MONO * 50) // queue size in samples
#define SR 8000 // sample rate

typedef long long int64;

static int on;
OPENSL_STREAM  *p = NULL;

circular_buffer *recbuf = NULL;

void *aecm = NULL;
NsxHandle *ns_inst;

int64 t_start;
std::queue<int64> t_render;
std::queue<int64> t_analyze;

// dump audio data
FILE *fd_farend = NULL;
FILE *fd_nearend = NULL;
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
  static float buf[VECSAMPS_MONO];
  for (int i = 0; i < VECSAMPS_MONO; ++i) {
    buf[i] = frame[i] / 32768.0;
  }
  fwrite(buf, VECSAMPS_MONO, 4, fd);
}

void init()
{
  recbuf = create_circular_buffer(BUFFERFRAMES);

  WebRtcNsx_Create(&ns_inst);
  WebRtcNsx_Init(ns_inst, SR);
  WebRtcNsx_set_policy(ns_inst, 2);

  WebRtcAecm_Create(&aecm);
  WebRtcAecm_Init(aecm, SR);

  p = android_OpenAudioDevice(SR,1,1,BUFFERFRAMES);
  if(p == NULL) return; 
  fd_farend = fopen("/mnt/sdcard/tmp/in.dat", "w+");
  fd_nearend = fopen("/mnt/sdcard/tmp/out.dat", "w+");
  fd_mic = fopen("/mnt/sdcard/tmp/mic.dat", "w+");
  t_start = timestamp(0) - 2000;
  on = 1;
}

void run() 
{
  int samps, i, j;
  short inbuffer[VECSAMPS_MONO]; // record
  short processedbuffer[VECSAMPS_MONO];
  short processedbuffer2[VECSAMPS_MONO];

  while(on) {

    samps = android_AudioIn(p,inbuffer,VECSAMPS_MONO);
    int64 t_capture = timestamp(t_start) - (BUFFERFRAMES / VECSAMPS_MONO * 10);

    WebRtcNsx_Process(ns_inst, inbuffer, NULL, processedbuffer, NULL);

    if (!t_render.empty()) 
    {
      assert(!t_analyze.empty());
      int64 t_process = timestamp(t_start);
      int delay = (t_render.front() - t_analyze.front()) + (t_process - t_capture);
      WebRtcAecm_Process(
          aecm, inbuffer, processedbuffer, processedbuffer2, VECSAMPS_MONO, delay);
      __android_log_print(ANDROID_LOG_DEBUG, "webrtc", "aecm process, delay=%d = (%lld - %lld) + (%lld - %lld)",
          delay, t_render.front(), t_analyze.front(), t_process, t_capture);
      //write_circular_buffer(recbuf, processedbuffer, VECSAMPS_MONO);
      dump_audio(processedbuffer, fd_mic);
      dump_audio(processedbuffer2, fd_nearend);
      t_render.pop();
      t_analyze.pop();
    }
    else
    {
      //__android_log_print(ANDROID_LOG_DEBUG, "webrtc", "aecm process skipped");
      //write_circular_buffer(recbuf, processedbuffer, VECSAMPS_MONO);
      dump_audio(processedbuffer, fd_mic);
      dump_audio(processedbuffer, fd_nearend);
    }
  }  

  android_CloseAudioDevice(p);
  WebRtcAecm_Free(aecm);
  WebRtcNsx_Free(ns_inst);
}

void close()
{
  on = 0;
  fclose(fd_farend);
  fclose(fd_nearend);
  fclose(fd_mic);
}

int pull(JNIEnv *env, jshortArray buf)
{
  jshort *_buf = env->GetShortArrayElements(buf, NULL);
  int n = read_circular_buffer(recbuf, _buf, VECSAMPS_MONO);
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
  WebRtcAecm_BufferFarend(aecm, _farend, VECSAMPS_MONO);
  dump_audio(_farend, fd_farend);

  // render
  t_render.push(timestamp(t_start) + BUFFERFRAMES / VECSAMPS_MONO * 10);
  if (android_AudioOut(p,_farend,samps) != samps)
    rtn = 0; 

  env->ReleaseShortArrayElements(farend, _farend, 0);
  return rtn;
}

double getTimestamp()
{
  return android_GetTimestamp(p);
}
