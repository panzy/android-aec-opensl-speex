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
#include "opensl_io.h"
#include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"
#include "webrtc/modules/audio_processing/ns/ns_core.h"
#include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"

#define VECSAMPS_MONO 80 // frame size in samples
#define BUFFERFRAMES (VECSAMPS_MONO * 50) // queue size in samples
#define SR 8000 // sample rate
#define PLAY_ON_MAIN_PROC 0 // playback on main thread or push thread?

static int on;
OPENSL_STREAM  *p = NULL;

circular_buffer *playbuf = NULL;
circular_buffer *recbuf = NULL;

void *aecm = NULL;
NsxHandle *ns_inst;

void start_process() {
  int samps, i, j;
  short inbuffer[VECSAMPS_MONO];
  short outbuffer[VECSAMPS_MONO];
  short processedbuffer[VECSAMPS_MONO];

  playbuf = create_circular_buffer(BUFFERFRAMES);
  recbuf = create_circular_buffer(BUFFERFRAMES);

  WebRtcNsx_Create(&ns_inst);
  WebRtcNsx_Init(ns_inst, SR);
  WebRtcNsx_set_policy(ns_inst, 2);

  WebRtcAecm_Create(&aecm);
  WebRtcAecm_Init(aecm, SR);

  p = android_OpenAudioDevice(SR,1,1,BUFFERFRAMES);
  if(p == NULL) return; 

  on = 1;
  while(on) {
    samps = android_AudioIn(p,inbuffer,VECSAMPS_MONO);

    int n = read_circular_buffer(playbuf, outbuffer, VECSAMPS_MONO);
    if (n == VECSAMPS_MONO) {
#if PLAY_ON_MAIN_PROC
      android_AudioOut(p,outbuffer,samps); 
#endif
      //write_circular_buffer(recbuf, inbuffer, VECSAMPS_MONO);
      
      WebRtcNsx_Process(ns_inst, inbuffer, NULL, processedbuffer, NULL);

      //WebRtcAecm_BufferFarend(aecm, outbuffer, VECSAMPS_MONO);
      //WebRtcAecm_Process(
      //    aecm, inbuffer, NULL, processedbuffer, VECSAMPS_MONO, 500);
      
      write_circular_buffer(recbuf, processedbuffer, VECSAMPS_MONO);
    } else {
      write_circular_buffer(recbuf, inbuffer, VECSAMPS_MONO);
    }
  }  

  android_CloseAudioDevice(p);
  free_circular_buffer(playbuf);
  WebRtcAecm_Free(aecm);
  WebRtcNsx_Free(ns_inst);
}

void stop_process(){
  on = 0;
}

int pull(JNIEnv *env, jshortArray buf) {
  jshort *_buf = (*env)->GetShortArrayElements(env, buf, NULL);
  int n = read_circular_buffer(recbuf, _buf, VECSAMPS_MONO);
  (*env)->ReleaseShortArrayElements(env, buf, _buf, 0);
  return n;
}

int push(JNIEnv *env, jshortArray farend) {
  jshort *_farend = (*env)->GetShortArrayElements(env, farend, NULL);
  jsize samps = (*env)->GetArrayLength(env, farend);
  int rtn = samps;
  if (write_circular_buffer(playbuf, _farend, samps) != samps)
    rtn = 0;
#if !PLAY_ON_MAIN_PROC
  if (android_AudioOut(p,_farend,samps) != samps)
    rtn = 0; 
#endif
  (*env)->ReleaseShortArrayElements(env, farend, _farend, 0);
  return rtn;
}

double getTimestamp() {
  return android_GetTimestamp(p);
}
