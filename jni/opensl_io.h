#ifndef OPENSL_IO
#define OPENSL_IO

#include <SLES/OpenSLES.h>
#include <SLES/OpenSLES_Android.h>
#include <pthread.h>
#include <stdlib.h>

typedef struct threadLock_{
  pthread_mutex_t m;
  pthread_cond_t  c;
  unsigned char   s;
} threadLock;

#ifdef __cplusplus
extern "C" {
#endif

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

  // recorder interfaces
  SLObjectItf recorderObject;
  SLRecordItf recorderRecord;
  SLAndroidSimpleBufferQueueItf recorderBufferQueue;

  // buffer indexes
  int currentInputIndex;
  int currentOutputIndex;

  // current buffer half (0, 1)
  int currentOutputBuffer;
  int currentInputBuffer;
  
  // buffers
  short *outputBuffer[2];
  short *inputBuffer[2];

  // size of buffers
  int outBufSamples;
  int inBufSamples;

  // locks
  void*  inlock;
  void*  outlock;

  double time;
  int inchannels;
  int outchannels;
  int   sr;

} OPENSL_STREAM;

  /*
  Open the audio device with a given sampling rate (sr), input and output channels and IO buffer size
  in frames. Returns a handle to the OpenSL stream
  */
  OPENSL_STREAM* android_OpenAudioDevice(int sr, int inchannels, int outchannels, int bufferframes);
  /* 
  Close the audio device 
  */
  void android_CloseAudioDevice(OPENSL_STREAM *p);
  /* 
  Read a buffer from the OpenSL stream *p, of size samples. Returns the number of samples read.
  */
  int android_AudioIn(OPENSL_STREAM *p, float *buffer,int size);
  /*
  Write a buffer to the OpenSL stream *p, of size samples. Returns the number of samples written.
  */
  int android_AudioOut(OPENSL_STREAM *p, float *buffer,int size);
  /*
  Get the current IO block time in seconds
  */
  double android_GetTimestamp(OPENSL_STREAM *p);
  
#ifdef __cplusplus
};
#endif

#endif // #ifndef OPENSL_IO
