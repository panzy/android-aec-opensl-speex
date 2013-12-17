/*
AudiotestActivity.java:
Opensl example Application
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

package com.audiotest;

import android.annotation.TargetApi;
import android.content.Context;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioRecord;
import android.media.AudioTrack;
import android.os.Build;
import android.util.Log;
import opensl_example.opensl_example;
import android.app.Activity;
import android.os.Bundle;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import static android.os.Build.VERSION_CODES.*;

public class AudiotestActivity extends Activity {
    private static final String TAG = "java";
    private static final int SR = 8000; // sample rate
    private static final int FRAME_SAMPS = 320; // samples per frame
    private static final int FRAME_MS = 1000 * FRAME_SAMPS / SR; // ms
    private static final int playback_delay = 300; // ms

    /** Called when the activity is first created. */
	Thread thread;
    Thread thread2;
    Thread thread3;
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        int track_minbufsz = AudioTrack.getMinBufferSize(8000, AudioFormat.CHANNEL_OUT_MONO, AudioFormat.ENCODING_PCM_16BIT);
        Log.d(TAG, "AudioTrack.getMinBufferSize " + track_minbufsz);
        int record_minbufsz = AudioRecord.getMinBufferSize(8000, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT);
        Log.d(TAG, "AudioRecord.getMinBufferSize " + record_minbufsz);

        thread = new Thread() {
            public void run() {
                setPriority(Thread.MAX_PRIORITY);
                opensl_example.runNearendProcessing();
            }
        };

        thread2 = new Thread() {
            public void run() {
                try {
                    //FileInputStream fis = new FileInputStream("/mnt/sdcard/tmp/speaker.raw");
                    InputStream fis = AudiotestActivity.this.getAssets().open("speaker.raw");
                    short[] buf = new short[FRAME_SAMPS];
                    byte[] bytes = new byte[FRAME_SAMPS * 2];
                    if (fis != null) {
                        long t0 = System.currentTimeMillis();
                        int loopIdx = 0;
                        while (thread != null && fis.read(bytes) > 0) {
                            ++loopIdx;
                            ByteBuffer.wrap(bytes)
                                    .order(ByteOrder.LITTLE_ENDIAN)
                                    .asShortBuffer().get(buf);

                            // 有两种方式控制 push 的节奏：
                            // A) 控制真实流逝时间和音频流时间的差距
                            // B) 检测到队列满了，就 sleep
                            if (false) {
                                if (opensl_example.push(buf) != FRAME_SAMPS)
                                    Log.d(TAG, "push failed");

                                // 如果写得太快，需要暂停一会，否则会覆盖底层的缓冲区
                                long ahead = (loopIdx * FRAME_MS) - (System.currentTimeMillis() - t0);
                                if (ahead > 200) {
                                    sleep(ahead - 200);
                                }
                            } else {
                                while (opensl_example.push(buf) != FRAME_SAMPS) {
                                    sleep(FRAME_MS);
                                }
                            }

                            // 模拟网络阻塞或Java GC造成的随机延迟
                            if (loopIdx % 32 == 0) {
                                Thread.sleep(FRAME_MS * 32);
                            }
                        }

                        ++loopIdx;
                        long ahead = (loopIdx * FRAME_MS) - (System.currentTimeMillis() - t0);
                        if (ahead > 0) {
                            sleep(ahead);
                        }
                    }
                    fis.close();

                    // exit app
                    sleep(playback_delay);
                    AudiotestActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            AudiotestActivity.this.finish();
                        }
                    });
                } catch (FileNotFoundException e) {
                } catch (IOException e) {
                } catch (InterruptedException e) {
                }
            }
        };

        thread3 = new Thread() {
            public void run() {
                try {
                    short[] buf = new short[FRAME_SAMPS];
                    byte[] bytes = new byte[FRAME_SAMPS * 2];
                    FileOutputStream fos = new FileOutputStream("/mnt/sdcard/tmp/out.raw");
                    while(thread != null) {
                        int n = opensl_example.pull(buf);
                        if (n > 0) {
                            ByteBuffer.wrap(bytes)
                                    .order(ByteOrder.LITTLE_ENDIAN)
                                    .asShortBuffer().put(buf);
                            fos.write(bytes);
                        } else {
                        }
                        sleep(FRAME_MS);
                    }
                    fos.close();
                } catch (FileNotFoundException e) {
                } catch (IOException e) {
                } catch (InterruptedException e) {
                }
            }
        };

        if (true) {
            opensl_example.start(track_minbufsz, record_minbufsz, playback_delay);
            thread.start();
            thread2.start();
            //thread3.start();
        } else {
            opensl_example.estimate_delay(0);
        }

        checkAudioLowLatencyFeature();

        querySampleRate();
    }

    @TargetApi(JELLY_BEAN_MR1)
    private void querySampleRate() {
        if (Integer.valueOf(Build.VERSION.SDK) < 17)
            return;
        AudioManager am = (AudioManager) this.getSystemService(Context.AUDIO_SERVICE);
        String fpb = am.getProperty(AudioManager.PROPERTY_OUTPUT_FRAMES_PER_BUFFER);
        String sr = am.getProperty(AudioManager.PROPERTY_OUTPUT_SAMPLE_RATE);
        Log.d(TAG, "AudioManager output frame per buf: " + fpb + ", sample rate: " + sr);
    }

    /**
     * false on Moto Xoom
     */
    private void checkAudioLowLatencyFeature() {
        PackageManager pm = this.getPackageManager();
        boolean claimsFeature = pm.hasSystemFeature(PackageManager.FEATURE_AUDIO_LOW_LATENCY);
        Log.d(TAG, "has feature audio low latency: " + claimsFeature);
    }

    public void onDestroy(){
    	
    	super.onDestroy();
    	opensl_example.stop();
    	try {
            thread2.interrupt();
            thread3.interrupt();
			thread.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    	thread = null;
        thread2 = null;
    }
}