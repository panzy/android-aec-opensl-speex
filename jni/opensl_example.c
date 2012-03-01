#include <android/log.h>
#include "opensl_io.h"

#define BUFFERFRAMES 1024
#define VECSAMPS_MONO 64
#define VECSAMPS_STEREO 128
#define SR 44100

static int on;
void start_process() {
  OPENSL_STREAM  *p;
  int samps, i, j;
  float  inbuffer[VECSAMPS_MONO], outbuffer[VECSAMPS_STEREO];
  p = android_OpenAudioDevice(SR,1,2,BUFFERFRAMES);
  if(p == NULL) return; 
  on = 1;
  while(on) {
   samps = android_AudioIn(p,inbuffer,VECSAMPS_MONO);
   for(i = 0, j=0; i < samps; i++, j+=2)
     outbuffer[j] = outbuffer[j+1] = inbuffer[i];
   android_AudioOut(p,outbuffer,VECSAMPS_STEREO); 
  }  
  android_CloseAudioDevice(p);
}

void stop_process(){
  on = 0;
}
