#ifndef _COMMON_H_
#define _COMMON_H_

#include <android/log.h>
#include <bits/stl_algobase.h>

#define I(fmt, ...)  __android_log_print(ANDROID_LOG_INFO, TAG, fmt, ##__VA_ARGS__);
#define W(fmt, ...)  __android_log_print(ANDROID_LOG_WARN, TAG, fmt, ##__VA_ARGS__);
#define E(fmt, ...)  __android_log_print(ANDROID_LOG_ERROR, TAG, fmt, ##__VA_ARGS__);

#if 0
#define D(fmt, ...)  __android_log_print(ANDROID_LOG_DEBUG, TAG, fmt, ##__VA_ARGS__);
#else
#define D(fmt, ...) (0)
#endif

#endif
