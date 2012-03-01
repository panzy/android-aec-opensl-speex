#!/bin/sh

export ANDROID_NDK_ROOT=$HOME/work/android-ndk-r7
export NDK_MODULE_PATH=../android

rm -rf src/opensl_example
mkdir -p src/opensl_example

swig -java -package opensl_example -includeall -verbose -outdir src/opensl_example -c++ -I/usr/local/include -I/System/Library/Frameworks/JavaVM.framework/Headers -I./jni -o jni/java_interface_wrap.cpp opensl_example_interface.i

$ANDROID_NDK_ROOT/ndk-build TARGET_PLATFORM=android-9 V=1

#cp libs/armeabi-v7a/libecho.so ../android/workspace/echo/libs/armeabi-v7a/libecho.so



