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
import android.media.AudioManager;
import android.os.Build;
import android.util.Log;
import opensl_example.opensl_example;
import android.app.Activity;
import android.os.Bundle;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import static android.os.Build.VERSION_CODES.*;

public class AudiotestActivity extends Activity {
    private static final String TAG = "java";
    private static final int SR = 8000; // sample rate
    private static final int FRAME_SAMPS = 80; // 80 samples per frame
    private static final int FRAME_MS = 10; // 10ms

    /** Called when the activity is first created. */
	Thread thread;
    Thread thread2;
    Thread thread3;
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        thread = new Thread() {
            public void run() {
                setPriority(Thread.MAX_PRIORITY);
                opensl_example.run();
            }
        };

        thread2 = new Thread() {
            public void run() {
                try {
                    FileInputStream fis = new FileInputStream("/mnt/sdcard/tmp/speaker.dat");
                    short[] buf = new short[FRAME_SAMPS];
                    byte[] bytes = new byte[FRAME_SAMPS * 2];
                    if (fis != null) {
                        long t = System.currentTimeMillis();
                        long t0 = t;
                        while (thread != null && fis.read(bytes) > 0) {
                            ByteBuffer.wrap(bytes)
                                    .order(ByteOrder.LITTLE_ENDIAN)
                                    .asShortBuffer().get(buf);

                            // push() will block if internal queue is full
                            if (opensl_example.push(buf) != FRAME_SAMPS)
                                Log.d(TAG, "push failed");
                            long t2 = System.currentTimeMillis();
                            Log.d(TAG, "push elapse " + (t2 - t));
                            t = t2;
                        }
                        Log.d(TAG, "audio timestamp by opensl " + opensl_example.getTimestamp());
                        Log.d(TAG, "audio timestamp by java " + (System.currentTimeMillis() - t0));
                    }
                    fis.close();
                } catch (FileNotFoundException e) {
                } catch (IOException e) {
                }
            }
        };

        thread3 = new Thread() {
            public void run() {
                try {
                    short[] buf = new short[FRAME_SAMPS];
                    byte[] bytes = new byte[FRAME_SAMPS * 2];
                    FileOutputStream fos = new FileOutputStream("/mnt/sdcard/tmp/out.dat");
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

        opensl_example.init();
		thread.start();
        thread2.start();
        //thread3.start();


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
    	opensl_example.close();
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