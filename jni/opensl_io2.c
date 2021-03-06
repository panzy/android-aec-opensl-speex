/*
  opensl_io.c:
  Android OpenSL input/output module
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

#include "opensl_io2.h"

#define TAG "aec_opensl"

static SLuint32 openSLChooseInputDevice(OPENSL_STREAM *p);
static void bqPlayerCallback(SLAndroidSimpleBufferQueueItf bq, void *context);
static void bqRecorderCallback(SLAndroidSimpleBufferQueueItf bq, void *context);
circular_buffer* create_circular_buffer(int count);
int checkspace_circular_buffer(circular_buffer *p, int writeCheck);
void free_circular_buffer (circular_buffer *p);

// creates the OpenSL ES audio engine
static SLresult openSLCreateEngine(OPENSL_STREAM *p)
{
  SLresult result;
  // create engine
  result = slCreateEngine(&(p->engineObject), 0, NULL, 0, NULL, NULL);
  if(result != SL_RESULT_SUCCESS) goto  engine_end;

  // realize the engine 
  result = (*p->engineObject)->Realize(p->engineObject, SL_BOOLEAN_FALSE);
  if(result != SL_RESULT_SUCCESS) goto engine_end;

  // get the engine interface, which is needed in order to create other objects
  result = (*p->engineObject)->GetInterface(p->engineObject, SL_IID_ENGINE, &(p->engineEngine));
  if(result != SL_RESULT_SUCCESS) goto  engine_end;

 engine_end:
  return result;
}

// opens the OpenSL ES device for output
//
// streamType - Audio playback stream type, see SL_ANDROID_STREAM_* constants in
//              <SLES/OpenSLES_AndroidConfiguration.h>, eg,
//              SL_ANDROID_STREAM_VOICE,
//              SL_ANDROID_STREAM_MEDIA.
SLresult openSLPlayOpen(OPENSL_STREAM *p, SLint32 streamType)
{
  SLresult result = SL_RESULT_SUCCESS;
  SLuint32 sr = p->sr;
  SLuint32  channels = p->outchannels;

  if (0 == pthread_mutex_trylock(&p->playerLock)) {
    SLDataLocator_AndroidSimpleBufferQueue loc_bufq = {SL_DATALOCATOR_ANDROIDSIMPLEBUFFERQUEUE, 2};
    if(channels) {
      // configure audio source

      switch(sr){

        case 8000:
          sr = SL_SAMPLINGRATE_8;
          break;
        case 11025:
          sr = SL_SAMPLINGRATE_11_025;
          break;
        case 16000:
          sr = SL_SAMPLINGRATE_16;
          break;
        case 22050:
          sr = SL_SAMPLINGRATE_22_05;
          break;
        case 24000:
          sr = SL_SAMPLINGRATE_24;
          break;
        case 32000:
          sr = SL_SAMPLINGRATE_32;
          break;
        case 44100:
          sr = SL_SAMPLINGRATE_44_1;
          break;
        case 48000:
          sr = SL_SAMPLINGRATE_48;
          break;
        case 64000:
          sr = SL_SAMPLINGRATE_64;
          break;
        case 88200:
          sr = SL_SAMPLINGRATE_88_2;
          break;
        case 96000:
          sr = SL_SAMPLINGRATE_96;
          break;
        case 192000:
          sr = SL_SAMPLINGRATE_192;
          break;
        default:
          result = SL_RESULT_INTERNAL_ERROR;
      }
    } else {
      result = SL_RESULT_INTERNAL_ERROR;
    }

    if (result == SL_RESULT_SUCCESS) {
      const SLInterfaceID ids[] = {SL_IID_VOLUME};
      const SLboolean req[] = {SL_BOOLEAN_FALSE};
      result = (*p->engineEngine)->CreateOutputMix(p->engineEngine, &(p->outputMixObject), 1, ids, req);
    }

    if (result == SL_RESULT_SUCCESS) {

      // realize the output mix
      result = (*p->outputMixObject)->Realize(p->outputMixObject, SL_BOOLEAN_FALSE);
    }

    if (result == SL_RESULT_SUCCESS) {
      int speakers;
      if(channels > 1) 
        speakers = SL_SPEAKER_FRONT_LEFT | SL_SPEAKER_FRONT_RIGHT;
      else speakers = SL_SPEAKER_FRONT_CENTER;
      SLDataFormat_PCM format_pcm = {SL_DATAFORMAT_PCM,channels, sr,
        SL_PCMSAMPLEFORMAT_FIXED_16, SL_PCMSAMPLEFORMAT_FIXED_16,
        speakers, SL_BYTEORDER_LITTLEENDIAN};

      SLDataSource audioSrc = {&loc_bufq, &format_pcm};

      // configure audio sink
      SLDataLocator_OutputMix loc_outmix = {SL_DATALOCATOR_OUTPUTMIX, p->outputMixObject};
      SLDataSink audioSnk = {&loc_outmix, NULL};

      // create audio player
      const SLInterfaceID ids1[] = {SL_IID_ANDROIDSIMPLEBUFFERQUEUE, SL_IID_ANDROIDCONFIGURATION, SL_IID_VOLUME};
      const SLboolean req1[] = {SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE};
      result = (*p->engineEngine)->CreateAudioPlayer(p->engineEngine, &(p->bqPlayerObject), &audioSrc, &audioSnk,
          sizeof(ids1) / sizeof(SLInterfaceID), ids1, req1);
    }

    if (result == SL_RESULT_SUCCESS) {
      // set stream type
      p->bqPlayerStreamType = streamType;
      SLAndroidConfigurationItf playerConfig;
      result = (*p->bqPlayerObject)->GetInterface(p->bqPlayerObject,
          SL_IID_ANDROIDCONFIGURATION, &playerConfig);
      assert(SL_RESULT_SUCCESS == result);
      if (SL_RESULT_SUCCESS == result) {
        result = (*playerConfig)->SetConfiguration(playerConfig,
            SL_ANDROID_KEY_STREAM_TYPE, &streamType, sizeof(SLint32));
        assert(SL_RESULT_SUCCESS == result);
      }

      // realize the player
      result = (*p->bqPlayerObject)->Realize(p->bqPlayerObject, SL_BOOLEAN_FALSE);
    }

    if (result == SL_RESULT_SUCCESS) {
      // get the play interface
      result = (*p->bqPlayerObject)->GetInterface(p->bqPlayerObject, SL_IID_PLAY, &(p->bqPlayerPlay));
    }
    if (result == SL_RESULT_SUCCESS) {
      // get the buffer queue interface
      result = (*p->bqPlayerObject)->GetInterface(p->bqPlayerObject, SL_IID_ANDROIDSIMPLEBUFFERQUEUE,
          &(p->bqPlayerBufferQueue));
    }
    if (result == SL_RESULT_SUCCESS) {
      // register callback on the buffer queue
      result = (*p->bqPlayerBufferQueue)->RegisterCallback(p->bqPlayerBufferQueue, bqPlayerCallback, p);
    }
    if (result == SL_RESULT_SUCCESS) {
      SLVolumeItf volumeIf;
      result = (*p->bqPlayerObject)->GetInterface(p->bqPlayerObject, SL_IID_VOLUME, &volumeIf);
      if (result == SL_RESULT_SUCCESS) {
        SLmillibel maxVolume, volume;
        (*volumeIf)->GetMaxVolumeLevel(volumeIf, &maxVolume);
        //(*volumeIf)->GetVolumeLevel(volumeIf, &volume);
        //I("audio player volume, max %dmB, curr %dmB", maxVolume, volume);
        // 在某些设备上，音量太大时录音会失真（波峰被截断了，可能是受音频格式PCM16
        // 限制？），导致speex AEC失败。
        volume = -300; // mB, 1dB=100mB
        (*volumeIf)->SetVolumeLevel(volumeIf, volume);
        (*volumeIf)->GetVolumeLevel(volumeIf, &volume);
        I("audio player volume, max %dmB, curr %dmB", maxVolume, volume);
      }

      // set the player's state to playing
      result = (*p->bqPlayerPlay)->SetPlayState(p->bqPlayerPlay, SL_PLAYSTATE_PLAYING);
    }

    if (result == SL_RESULT_SUCCESS) {
      if((p->playBuffer = (short *) calloc(p->outBufSamples, sizeof(short))) == NULL) {
        result = SL_RESULT_INTERNAL_ERROR;
      }
    }

    pthread_mutex_unlock(&p->playerLock);

    if (result == SL_RESULT_SUCCESS) {
      (*p->bqPlayerBufferQueue)->Enqueue(p->bqPlayerBufferQueue, 
          p->playBuffer,p->outBufSamples*sizeof(short));
    }
  }
  return result;
}

void openSLPlayClose(OPENSL_STREAM *p){

  // destroy buffer queue audio player object, and invalidate all associated interfaces
  if (p && 0 == pthread_mutex_trylock(&p->playerLock)) {
    if (p->bqPlayerObject != NULL && p->bqPlayerPlay != NULL) {
      SLuint32 state = SL_PLAYSTATE_PLAYING;
      (*p->bqPlayerPlay)->SetPlayState(p->bqPlayerPlay, SL_PLAYSTATE_STOPPED);
      while(state != SL_PLAYSTATE_STOPPED)
        (*p->bqPlayerPlay)->GetPlayState(p->bqPlayerPlay, &state);
      (*p->bqPlayerObject)->Destroy(p->bqPlayerObject);
      p->bqPlayerObject = NULL;
      p->bqPlayerPlay = NULL;
      p->bqPlayerBufferQueue = NULL;
      p->bqPlayerEffectSend = NULL;
    }
    pthread_mutex_unlock(&p->playerLock);
  }
}

// Open the OpenSL ES device for input
static SLresult openSLRecOpen(OPENSL_STREAM *p){

  SLresult result;
  SLuint32 sr = p->sr;
  SLuint32 channels = p->inchannels;

  if(channels){

    switch(sr){

    case 8000:
      sr = SL_SAMPLINGRATE_8;
      break;
    case 11025:
      sr = SL_SAMPLINGRATE_11_025;
      break;
    case 16000:
      sr = SL_SAMPLINGRATE_16;
      break;
    case 22050:
      sr = SL_SAMPLINGRATE_22_05;
      break;
    case 24000:
      sr = SL_SAMPLINGRATE_24;
      break;
    case 32000:
      sr = SL_SAMPLINGRATE_32;
      break;
    case 44100:
      sr = SL_SAMPLINGRATE_44_1;
      break;
    case 48000:
      sr = SL_SAMPLINGRATE_48;
      break;
    case 64000:
      sr = SL_SAMPLINGRATE_64;
      break;
    case 88200:
      sr = SL_SAMPLINGRATE_88_2;
      break;
    case 96000:
      sr = SL_SAMPLINGRATE_96;
      break;
    case 192000:
      sr = SL_SAMPLINGRATE_192;
      break;
    default:
      return -1;
    }

    SLuint32 mic_deviceId = openSLChooseInputDevice(p);
    
    // configure audio source
    SLDataLocator_IODevice loc_dev = {SL_DATALOCATOR_IODEVICE, SL_IODEVICE_AUDIOINPUT,
				      mic_deviceId, NULL};
    SLDataSource audioSrc = {&loc_dev, NULL};

    // configure audio sink
    int speakers;
    if(channels > 1) 
      speakers = SL_SPEAKER_FRONT_LEFT | SL_SPEAKER_FRONT_RIGHT;
    else speakers = SL_SPEAKER_FRONT_CENTER;
    SLDataLocator_AndroidSimpleBufferQueue loc_bq = {SL_DATALOCATOR_ANDROIDSIMPLEBUFFERQUEUE, 2};
    SLDataFormat_PCM format_pcm = {SL_DATAFORMAT_PCM, channels, sr,
				   SL_PCMSAMPLEFORMAT_FIXED_16, SL_PCMSAMPLEFORMAT_FIXED_16,
				   speakers, SL_BYTEORDER_LITTLEENDIAN};
    SLDataSink audioSnk = {&loc_bq, &format_pcm};

    // create audio recorder
    // (requires the RECORD_AUDIO permission)
    const SLInterfaceID id[] = {SL_IID_ANDROIDSIMPLEBUFFERQUEUE, SL_IID_ANDROIDCONFIGURATION};
    const SLboolean req[] = {SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE};
    result = (*p->engineEngine)->CreateAudioRecorder(p->engineEngine, &(p->recorderObject), &audioSrc,
						     &audioSnk, sizeof(id) / sizeof(SLInterfaceID), id, req);
    if (SL_RESULT_SUCCESS != result) goto end_recopen;

    // set mic volume.
    // 这部分代码应该是没问题的，不过还没遇到哪个手机实现了这个功能。
    SLDeviceVolumeItf deviceVolumeItf;
    result = (*p->engineObject)->GetInterface(p->engineObject, SL_IID_DEVICEVOLUME, &deviceVolumeItf);
    if (result == SL_RESULT_SUCCESS) {
      SLint32 vol;
      (*deviceVolumeItf)->GetVolume(deviceVolumeItf, mic_deviceId, &vol);
      I("audio recorder, curr vol %d", (int)vol);
      // WebRTC 使用如下公式计算 mic volume:
      // vol = ((volume * (_maxSpeakerVolume - _minSpeakerVolume)
      //        + (int) (255 / 2)) / (255)) + _minSpeakerVolume;
      //
      // http://webrtc.googlecode.com/svn-history/r214/trunk/src/modules/audio_device/main/source/Android/audio_device_android_opensles.cc
      (*deviceVolumeItf)->SetVolume(deviceVolumeItf, mic_deviceId, -300);
      (*deviceVolumeItf)->GetVolume(deviceVolumeItf, mic_deviceId, &vol);
      I("audio recorder, curr vol %d", (int)vol);
    } else if (result == SL_RESULT_FEATURE_UNSUPPORTED) {
      W("failed to set mic volume, result %d, SL_RESULT_FEATURE_UNSUPPORTED", (int)result);
    }
    else {
      W("failed to set mic volume, result %d", (int)result);
    }

    // set recording preset
    SLAndroidConfigurationItf recorderConfig;   
    result = (*p->recorderObject)->GetInterface(p->recorderObject, SL_IID_ANDROIDCONFIGURATION, &recorderConfig);
    if(result == SL_RESULT_SUCCESS) {
      // 关于几种 SL_ANDROID_RECORDING_PRESET_*:
      //
      // SL_ANDROID_RECORDING_PRESET_VOICE_COMMUNICATION 
      //
      //    此配置会启用Android系统内置的回声消除功能（若设备支持的话）。
      //
      //    若设备未实现AEC功能，此配置通常比
      //    VOICE_RECOGNITION 更糟糕， 因为前者的近
      //    端可能严重失真因而不利于 Speex。
      //
      //    为了帮助内置AEC取得最佳效果，对播放也要进行一些设置：
      //
      //    1, stream type
      //
      //      设置为 AudioManager.STREAM_VOICE_CALL，
      //
      //      这个一般是在初始化播放器（不论是 OpenSL audio player，还是 Android
      //      Framework 中的 MediaPlayer，SoundPool，或 AudioTrack）时设置。
      //
      //    2, audio mode 
      //      
      //      设置为 AudioManager.MODE_IN_COMMUNICATION。
      //
      //      就目前所知，这个只能通过在 Java 代码中调用
      //      android.media.AudioManager.setMode()
      //      来设置，OpenSL 中没有等价 API。
      //
      // SL_ANDROID_RECORDING_PRESET_VOICE_RECOGNITION 
      //
      //    按照定义，它不会自动做回声消除，而实际上，它录制的近端信号通常比
      //    GENERIC 录制的要更加失真一些，因为也不利于 Speex 回声消除。
      //
      // SL_ANDROID_RECORDING_PRESET_GENERIC
      //    
      //    录制的近端信号中的回声最接近远端信号，因而最有利于用 Speex AEC.
      //
      // 结论：
      // 1, 对 Speex AEC 的友好性比较
      //    VOICE_COMMUNICATION <= VOICE_RECOGNITION <= GENERIC
      // 2, 如果设备真的内置 AEC，那么它的效果一般远好于我们自己用 Speex AEC。
      //
      SLint32 streamType = SL_ANDROID_RECORDING_PRESET_GENERIC;
      if(p->os_api_level >= 11 && p->is_aec_available){
        // SL_ANDROID_RECORDING_PRESET_VOICE_COMMUNICATION = 4
        // 由于我们目前采用的NDK是android-9，头文件中没有这个常量。
        streamType = 4;
        I("config recorder, android-%d, stream type=SL_ANDROID_RECORDING_PRESET_VOICE_COMMUNICATION", p->os_api_level); 
      } else {
        streamType = SL_ANDROID_RECORDING_PRESET_GENERIC;
        I("config recorder, android-%d, stream type=SL_ANDROID_RECORDING_PRESET_GENERIC", p->os_api_level); 
      }
      result = (*recorderConfig)->SetConfiguration(recorderConfig, SL_ANDROID_KEY_RECORDING_PRESET, &streamType, sizeof(SLint32));
      if (result != SL_RESULT_SUCCESS) {
        E("failed to config recorder");
      }
    }

    // realize the audio recorder
    result = (*p->recorderObject)->Realize(p->recorderObject, SL_BOOLEAN_FALSE);
    if (SL_RESULT_SUCCESS != result) goto end_recopen;

    // get the record interface
    result = (*p->recorderObject)->GetInterface(p->recorderObject, SL_IID_RECORD, &(p->recorderRecord));
    if (SL_RESULT_SUCCESS != result) goto end_recopen;
 
    // get the buffer queue interface
    result = (*p->recorderObject)->GetInterface(p->recorderObject, SL_IID_ANDROIDSIMPLEBUFFERQUEUE,
						&(p->recorderBufferQueue));
    if (SL_RESULT_SUCCESS != result) goto end_recopen;

    // register callback on the buffer queue
    result = (*p->recorderBufferQueue)->RegisterCallback(p->recorderBufferQueue, bqRecorderCallback,
							 p);
    if (SL_RESULT_SUCCESS != result) goto end_recopen;
    result = (*p->recorderRecord)->SetRecordState(p->recorderRecord, SL_RECORDSTATE_RECORDING);

    if((p->recBuffer = (short *) calloc(p->inBufSamples, sizeof(short))) == NULL) {
      return -1;
    }

    (*p->recorderBufferQueue)->Enqueue(p->recorderBufferQueue, 
				       p->recBuffer, p->inBufSamples*sizeof(short));

  end_recopen: 
    return result;
  }
  else return SL_RESULT_SUCCESS;


}

static SLuint32 openSLChooseInputDevice(OPENSL_STREAM *p) {

  SLuint32 mic_deviceId = SL_DEFAULTDEVICEID_AUDIOINPUT;
  SLresult result;
  SLAudioIODeviceCapabilitiesItf AudioIODeviceCapabilitiesItf;
  int i;
  result = (*p->engineObject)->GetInterface(p->engineObject, SL_IID_AUDIOIODEVICECAPABILITIES,
      &AudioIODeviceCapabilitiesItf);
  if (result == SL_RESULT_SUCCESS) {
    const int MAX_NUMBER_INPUT_DEVICES = 5;
    SLint32 numInputs = MAX_NUMBER_INPUT_DEVICES;
    SLuint32 InputDeviceIDs[MAX_NUMBER_INPUT_DEVICES];
    result = (*AudioIODeviceCapabilitiesItf)->GetAvailableAudioInputs(AudioIODeviceCapabilitiesItf,
        &numInputs, InputDeviceIDs);
    D("query mic devices: result %d, count %d", (int)result, (int)numInputs);
    if (result == SL_RESULT_SUCCESS && numInputs > 0) {
      SLAudioInputDescriptor AudioInputDescriptor;
      for (i = 0; i < numInputs; ++i) {
        result = (*AudioIODeviceCapabilitiesItf)->QueryAudioInputCapabilities(
            AudioIODeviceCapabilitiesItf, InputDeviceIDs[i], &AudioInputDescriptor); 
        D("see mic device id: %d", (int)InputDeviceIDs[i]);
        if((AudioInputDescriptor.deviceConnection ==
              SL_DEVCONNECTION_ATTACHED_WIRED)&&
            (AudioInputDescriptor.deviceScope == SL_DEVSCOPE_USER)&&
            (AudioInputDescriptor.deviceLocation ==
             SL_DEVLOCATION_HEADSET))
        {
          mic_deviceId = InputDeviceIDs[i];
          break;
        }
        else if((AudioInputDescriptor.deviceConnection ==
              SL_DEVCONNECTION_INTEGRATED)&&
            (AudioInputDescriptor.deviceScope ==
             SL_DEVSCOPE_USER)&&
            (AudioInputDescriptor.deviceLocation ==
             SL_DEVLOCATION_HANDSET))
        {
          mic_deviceId = InputDeviceIDs[i];
          break;
        }
      }
    }
  } else {
    W("failed to get interface SL_IID_AUDIOIODEVICECAPABILITIES, result %d", (uint)result);
  }
  D("choose mic device id: 0x%08x", (uint)mic_deviceId);
  return mic_deviceId;
}

// close the OpenSL IO and destroy the audio engine
static void openSLDestroyEngine(OPENSL_STREAM *p){

  // destroy buffer queue audio player object, and invalidate all associated interfaces
  openSLPlayClose(p);

  // destroy audio recorder object, and invalidate all associated interfaces
  if (p->recorderObject != NULL) {
   SLuint32 state = SL_PLAYSTATE_PLAYING;
    (*p->recorderRecord)->SetRecordState(p->recorderRecord, SL_RECORDSTATE_STOPPED);
    while(state != SL_RECORDSTATE_STOPPED)
      (*p->recorderRecord)->GetRecordState(p->recorderRecord, &state);
    (*p->recorderObject)->Destroy(p->recorderObject);
    p->recorderObject = NULL;
    p->recorderRecord = NULL;
    p->recorderBufferQueue = NULL;
  }

  // destroy output mix object, and invalidate all associated interfaces
  if (p->outputMixObject != NULL) {
    (*p->outputMixObject)->Destroy(p->outputMixObject);
    p->outputMixObject = NULL;
  }

  // destroy engine object, and invalidate all associated interfaces
  if (p->engineObject != NULL) {
    (*p->engineObject)->Destroy(p->engineObject);
    p->engineObject = NULL;
    p->engineEngine = NULL;
  }

}


// open the android audio device for input and/or output
OPENSL_STREAM *android_OpenAudioDevice(int sr, int inchannels, int outchannels,
        int in_buffer_frames, int in_buffer_cnt,
        int out_buffer_frames, int out_buffer_cnt,
        int os_api_level,
        int is_aec_available){
  
  OPENSL_STREAM *p;
  p = (OPENSL_STREAM *) malloc(sizeof(OPENSL_STREAM));
  memset(p, 0, sizeof(OPENSL_STREAM));
  p->inchannels = inchannels;
  p->outchannels = outchannels;
  p->sr = sr;
  p->os_api_level = os_api_level;
  p->is_aec_available = is_aec_available;
  pthread_mutex_init(&p->playerLock, NULL);
 
  if((p->outBufSamples  =  out_buffer_frames*outchannels) != 0) {
    if((p->outputBuffer = (short *) calloc(p->outBufSamples, sizeof(short))) == NULL) {
      android_CloseAudioDevice(p);
      return NULL;
    }
  }

  if((p->inBufSamples  =  in_buffer_frames*inchannels) != 0){
    if((p->inputBuffer = (short *) calloc(p->inBufSamples, sizeof(short))) == NULL){
      android_CloseAudioDevice(p);
      return NULL; 
    }
  }

  if((p->outrb = create_circular_buffer(p->outBufSamples*out_buffer_cnt)) == NULL) {
      android_CloseAudioDevice(p);
      return NULL; 
  }
 if((p->inrb = create_circular_buffer(p->inBufSamples*out_buffer_cnt)) == NULL) {
      android_CloseAudioDevice(p);
      return NULL; 
  }

  if(openSLCreateEngine(p) != SL_RESULT_SUCCESS) {
    android_CloseAudioDevice(p);
    return NULL;
  }

  if(openSLRecOpen(p) != SL_RESULT_SUCCESS) {
    android_CloseAudioDevice(p);
    return NULL;
  } 

  if(openSLPlayOpen(p, SL_ANDROID_STREAM_VOICE) != SL_RESULT_SUCCESS) {
    android_CloseAudioDevice(p);
    return NULL;
  }  

  p->time = 0.;
  return p;
}

// close the android audio device
void android_CloseAudioDevice(OPENSL_STREAM *p){

  if (p == NULL)
    return;

  openSLDestroyEngine(p);

  if (p->outputBuffer != NULL) {
    free(p->outputBuffer);
    p->outputBuffer= NULL;
  }

  if (p->inputBuffer != NULL) {
    free(p->inputBuffer);
    p->inputBuffer = NULL;
  }

  if (p->playBuffer != NULL) {
    free(p->playBuffer);
    p->playBuffer = NULL;
  }

  if (p->recBuffer != NULL) {
    free(p->recBuffer);
    p->recBuffer = NULL;
  }

  free_circular_buffer(p->inrb);
  free_circular_buffer(p->outrb);

  pthread_mutex_destroy(&p->playerLock);
  free(p);
}

// returns timestamp of the processed stream
double android_GetTimestamp(OPENSL_STREAM *p){
  return p->time;
}


// this callback handler is called every time a buffer finishes recording
void bqRecorderCallback(SLAndroidSimpleBufferQueueItf bq, void *context)
{
  OPENSL_STREAM *p = (OPENSL_STREAM *) context;
  int count = p->inBufSamples;
  if (write_circular_buffer(p->inrb, p->recBuffer,count) < count) {
    E("record buffer overflow");
  }
  (*p->recorderBufferQueue)->Enqueue(p->recorderBufferQueue,p->recBuffer,
          count*sizeof(short));
}
 
// gets a buffer of size samples from the device
int android_AudioIn(OPENSL_STREAM *p,short *buffer,int size){
  short *inBuffer;
  int i;
  if(p == NULL ||  p->inBufSamples ==  0) return 0;
  size = read_circular_buffer(p->inrb,p->inputBuffer,size);
  for(i=0; i < size; i++){
    buffer[i] =  p->inputBuffer[i];
  }
  if(p->outchannels == 0) p->time += (double) size/(p->sr*p->inchannels);
  return size;
}

// this callback handler is called every time a buffer finishes playing
void bqPlayerCallback(SLAndroidSimpleBufferQueueItf bq, void *context)
{
  OPENSL_STREAM *p = (OPENSL_STREAM *) context;
  if (p && 0 == pthread_mutex_trylock(&p->playerLock)) {
    int count = read_circular_buffer(p->outrb,p->playBuffer,p->outBufSamples);
    if (count < p->outBufSamples) {
      memset(p->playBuffer + count * sizeof(short), 0,
          (p->outBufSamples - count) * sizeof(short));
      count = p->outBufSamples;
    }
    (*p->bqPlayerBufferQueue)->Enqueue(p->bqPlayerBufferQueue,
        p->playBuffer,count*sizeof(short));
    pthread_mutex_unlock(&p->playerLock);
  }
}

// return: 0 or count.
int read_circular_buffer(circular_buffer *p, short *out, int count){
  int remaining;
  int size = p->size;
  int i=0, rp = p->rp;
  short *buffer = p->buffer;
  if ((remaining = checkspace_circular_buffer(p, 0)) < count) {
    return 0;
  }
  for(i=0; i < count; i++){
    out[i] = buffer[rp++];
    if(rp == size) rp = 0;
  }
  p->rp = rp;
  return count;
}

int write_circular_buffer(circular_buffer *p, const short *in, int count){
  int remaining;
  int countwrite, size = p->size;
  int i=0, wp = p->wp;
  short *buffer = p->buffer;
  if ((remaining = checkspace_circular_buffer(p, 1)) == 0) {
    return 0;
  }
  countwrite = count > remaining ? remaining : count;
  for(i=0; i < countwrite; i++){
    buffer[wp++] = in[i];
    if(wp == size) wp = 0;
  }
  p->wp = wp;
  return countwrite;
}

// puts a buffer of size samples to the device.
// non-block.
int android_AudioOut(OPENSL_STREAM *p, short *buffer,int size){

  short *outBuffer, *inBuffer;
  int i, count = size;
  if(p == NULL  ||  p->outBufSamples ==  0)  return 0;
  for(i=0; i < size; i++){
    p->outputBuffer[i] = buffer[i];
  }
  count = write_circular_buffer(p->outrb, p->outputBuffer,count);
  p->time += (double) size/(p->sr*p->outchannels);
  return count;
}

circular_buffer* create_circular_buffer(int count){
  circular_buffer *p;
  if ((p = calloc(1, sizeof(circular_buffer))) == NULL) {
    return NULL;
  }
  p->size = count;
  p->wp = p->rp = 0;
   
  if ((p->buffer = calloc(count, sizeof(short))) == NULL) {
    free (p);
    return NULL;
  }
  return p;
}

int checkspace_circular_buffer(circular_buffer *p, int writeCheck){
  int wp = p->wp, rp = p->rp, size = p->size;
  if(writeCheck){
    if (wp > rp) return rp - wp + size - 1;
    else if (wp < rp) return rp - wp - 1;
    else return size - 1;
  }
  else {
    if (wp > rp) return wp - rp;
    else if (wp < rp) return wp - rp + size;
    else return 0;
  }	
}

void
free_circular_buffer (circular_buffer *p){
  if(p == NULL) return;
  free(p->buffer);
  free(p);
}

