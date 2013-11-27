LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libwebrtc_audio_preprocessing
#LOCAL_EXPORT_C_INCLUDES := /home/panzy/workspace/webrtc-read-only/obj/local
#LOCAL_SRC_FILES := /home/panzy/workspace/webrtc-read-only/obj/local/armeabi/libwebrtc_audio_preprocessing.so
LOCAL_SRC_FILES := /home/panzy/workspace/webrtc-read-only/obj/local/$(TARGET_ARCH_ABI)/libwebrtc_audio_preprocessing.so
include $(PREBUILT_SHARED_LIBRARY)


include $(CLEAR_VARS)
LOCAL_MODULE   := opensl_example
LOCAL_C_INCLUDES := $(LOCAL_PATH) \
	/home/panzy/workspace/webrtc-read-only
LOCAL_CFLAGS := -O3 
LOCAL_CPPFLAGS :=$(LOCAL_CFLAGS)
###

LOCAL_SRC_FILES := opensl_example.cpp  \
opensl_io.c\
java_interface_wrap.cpp 

LOCAL_SHARED_LIBRARIES := libwebrtc_audio_preprocessing

LOCAL_LDLIBS := -llog -lOpenSLES

include $(BUILD_SHARED_LIBRARY)


