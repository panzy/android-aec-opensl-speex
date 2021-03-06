/*
opensl_io.c:
Android OpenSL input/output module header
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

#ifndef OPENSL_IO
#define OPENSL_IO

#include <SLES/OpenSLES.h>
#include <SLES/OpenSLES_Android.h>
#include <SLES/OpenSLES_AndroidConfiguration.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include "common.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct _circular_buffer {
  short *buffer;
  int  wp;
  int rp;
  int size;
} circular_buffer;

typedef struct opensl_stream {
  
  // engine interfaces
  SLObjectItf engineObject;
  SLEngineItf engineEngine;

  // output mix interfaces
  SLObjectItf outputMixObject;

  // buffer queue player interfaces
  SLObjectItf bqPlayerObject;
  SLPlayItf bqPlayerPlay;
  SLAndroidSimpleBufferQueueItf bqPlayerBufferQueue;
  SLEffectSendItf bqPlayerEffectSend;
  SLint32 bqPlayerStreamType;
  // 回调函数要读写缓冲区，主线程关闭播放器时要销毁缓冲区，
  // 这两个动作需要加锁。
  pthread_mutex_t playerLock;

  // recorder interfaces
  SLObjectItf recorderObject;
  SLRecordItf recorderRecord;
  SLAndroidSimpleBufferQueueItf recorderBufferQueue;
  
  // buffers
  short *outputBuffer;
  short *inputBuffer;
  short *recBuffer;
  short *playBuffer;

  circular_buffer *outrb;
  circular_buffer *inrb;

  // size of buffers
  int outBufSamples;
  int inBufSamples;

  double time;
  int inchannels;
  int outchannels;
  int   sr;

  int os_api_level;
  int is_aec_available; // built-in Acoustic Echo Cancellation

} OPENSL_STREAM;

  /*
  Open the audio device with a given sampling rate (sr), input and output channels and IO buffer size
  in frames. Returns a handle to the OpenSL stream
  */
  OPENSL_STREAM* android_OpenAudioDevice(int sr, int inchannels, int outchannels,
          int in_buffer_frames, int in_buffer_cnt,
          int out_buffer_frames, int out_buffer_cnt,
          int os_api_level, int is_aec_available);
  /* 
  Close the audio device 
  */
  void android_CloseAudioDevice(OPENSL_STREAM *p);
  // opens the OpenSL ES device for output
  //
  // streamType - Audio playback stream type, see SL_ANDROID_STREAM_* constants in
  //              <SLES/OpenSLES_AndroidConfiguration.h>, eg,
  //              SL_ANDROID_STREAM_VOICE,
  //              SL_ANDROID_STREAM_MEDIA.
  SLresult openSLPlayOpen(OPENSL_STREAM *p, SLint32 streamType);
  void openSLPlayClose(OPENSL_STREAM *p);

  /* 
  Read a buffer from the OpenSL stream *p, of size samples. Returns the number of samples read.
  */
  int android_AudioIn(OPENSL_STREAM *p, short *buffer,int size);
  /*
  Write a buffer to the OpenSL stream *p, of size samples. Returns the number of samples written.
  */
  int android_AudioOut(OPENSL_STREAM *p, short *buffer,int size);
  /*
  Get the current IO block time in seconds
  */
  double android_GetTimestamp(OPENSL_STREAM *p);
  
  circular_buffer* create_circular_buffer(int shorts);
  int checkspace_circular_buffer(circular_buffer *p, int writeCheck);
  int read_circular_buffer(circular_buffer *p, short *out, int shorts);
  int write_circular_buffer(circular_buffer *p, const short *in, int shorts);
  void free_circular_buffer (circular_buffer *p);
#ifdef __cplusplus
};
#endif

#endif // #ifndef OPENSL_IO
