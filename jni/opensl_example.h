/*
opensl_example.h:
OpenSL example module header, used for SWIG wrapping
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

/* 开始播放和录音。
 *
 * 音频参数：
 *  Sample Rate = 8kHz
 *  Sample Foramt = signed 16-bit PCM
 *  Byte order = Little-endian
 *  Channels = 1
 *
 * @param playback_delay_ms 指示延迟播放
 * @param echo_delay_ms 提供一个关于回声延迟的参考
 *
 * 延迟播放
 *
 * 如果等到完全填满 FAREND_BUFFER 才开始播放（在这之前只播放静音），这个播放延迟
 * 就长到无法忍受了，所以我们在播放 farend 之前只缓存 playback_delay_ms 那么多。
 *
 * 这么做的副作用是，如果上层在填充 playback_delay_ms 与FAREND_BUFFER_SAMPS 之间的
 * 数据时发生了严重的阻塞，播放的声音会有卡顿现象。不过这仍然不会影响 AEC，因为
 * AEC 主线程遇到 FAREND_BUFFER underrun 会自动补充静音帧，AEC 模块无需区分输入
 * 给它的 farend 是原样的还是经过underrun修正的。
 * */
void start(jint audio_track_min_buf_size, jint audio_record_min_buf_size,
    jint playback_delay_ms, jint echo_delay_ms);
/* 持续处理录音。*/
void runNearendProcessing();
/* 结束播放和录音。*/
void stop();
/* 添加一段声音到播放队列。
 * 不会阻塞，添加太快会覆盖缓冲区。
 * return: 0 or buf.len */
int push(JNIEnv *env, jshortArray buf);
/* 获取处理好的录音。
 * buf: a frame of samples.
 * return: 0 or buf.len */
int pull(JNIEnv *env, jshortArray buf);
int estimate_delay(int async);
/* 获取动态评估的回声延迟。
 * 请在 runNearendProcessing() 结束后调用。
 * return echo delay in ms, -1 if the value is unavailable.
 * */
int get_estimated_echo_delay();
/* 处理离线文件：
 * near.raw + echo.raw => send.raw
 */
void offline_process();
