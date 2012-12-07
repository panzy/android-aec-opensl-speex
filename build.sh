#!/bin/sh

export ANDROID_NDK_ROOT=$HOME/src/android-ndk-r8c

rm -rf src/opensl_example
mkdir -p src/opensl_example

swig -java -package opensl_example -includeall -verbose -outdir src/opensl_example -c++ -I/usr/local/include -I/System/Library/Frameworks/JavaVM.framework/Headers -I./jni -o jni/java_interface_wrap.cpp opensl_example_interface.i

$ANDROID_NDK_ROOT/ndk-build TARGET_PLATFORM=android-14 V=1




