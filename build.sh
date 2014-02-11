#!/bin/sh

export ANDROID_NDK_ROOT=/opt/android-ndk-r9b

PKG=cn.com.cybertech.audio
DIR=cn/com/cybertech/audio
rm -rf src/$DIR
mkdir -p src/$DIR

swig -java -package $PKG -includeall -verbose -outdir src/$DIR -c++ -I/usr/local/include -I/System/Library/Frameworks/JavaVM.framework/Headers -I./jni -o jni/java_interface_wrap.cpp cybertech_aec_interface.i

$ANDROID_NDK_ROOT/ndk-build TARGET_PLATFORM=android-9 V=1




