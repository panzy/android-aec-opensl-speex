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
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioRecord;
import android.media.AudioTrack;
import android.media.audiofx.AcousticEchoCanceler;
import android.os.Build;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import opensl_example.opensl_example;
import android.app.Activity;
import android.os.Bundle;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import static android.os.Build.VERSION_CODES.*;

public class AudiotestActivity extends Activity implements View.OnClickListener {
    private static final String output_dir = Environment.getExternalStorageDirectory()
            + "/tmp";
    private static final String far_filename = output_dir + "/far.raw";
    private static final String near_filename = output_dir + "/near.raw";
    private static final String echo_filename = output_dir + "/echo.raw";
    private static final String send_filename = output_dir + "/out.raw";

    private static final String TAG = "java";
    private static final int SR = 8000; // sample rate
    private static final int FRAME_SAMPS = 320; // samples per frame
    private static final int FRAME_MS = 1000 * FRAME_SAMPS / SR; // ms
    private static final int playback_delay = 300; // ms
    private static final int dump_raw = 0;

	Thread thread;
    Thread thread2;
    Thread thread3;

    private boolean isPlaying = false;

    int sampleRate = 8000;
    int managerBufferSize = 2000;
    // frame_size is the amount of data (in samples) you want to
    // process at once and filter_length is the length (in samples)
    // of the echo cancelling filter you want to use (also known as
    // tail length). It is recommended to use a frame size in the
    // order of 20 ms (or equal to the codec frame size) and make
    // sure it is easy to perform an FFT of that size (powers of two
    // are better than prime sizes). The recommended tail length is
    // approximately the third of the room reverberation time. For
    // example, in a small room, reverberation time is in the order
    // of 300 ms, so a tail length of 100 ms is a good choice (800
    // samples at 8000 Hz sampling rate).
    final static int frameMs = 20; // frame duration
    final static int frameRate = 1000 / frameMs;
    int frameSize = sampleRate * 20 / 1000;
    int filterLength = frameSize * 16;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        checkAudioLowLatencyFeature();

        querySampleRate();

        findViewById(R.id.btn_play_record).setOnClickListener(this);
        findViewById(R.id.btn_play_far).setOnClickListener(this);
        findViewById(R.id.btn_play_near).setOnClickListener(this);
        findViewById(R.id.btn_play_send).setOnClickListener(this);
        findViewById(R.id.btn_offline_aec).setOnClickListener(this);
        findViewById(R.id.btn_offline_est).setOnClickListener(this);
    }

    private void play_record_aec_start() {
        int track_minbufsz = AudioTrack.getMinBufferSize(8000,
                AudioFormat.CHANNEL_OUT_MONO, AudioFormat.ENCODING_PCM_16BIT);
        Log.d(TAG, "AudioTrack.getMinBufferSize " + track_minbufsz);
        int record_minbufsz = AudioRecord.getMinBufferSize(8000,
                AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT);
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
                                    sleep(FRAME_MS / 2);
                                }
                            }

                            // 模拟网络阻塞或Java GC造成的随机延迟
                            if (loopIdx % 32 == 0) {
                                Thread.sleep(FRAME_MS * 32);
                            }
                        }

                        ++loopIdx;
                        long ahead = (loopIdx * FRAME_MS) - (System.currentTimeMillis() - t0);
                        ahead -= 100;
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
                            play_record_aec_stop();
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
                    FileOutputStream fos = new FileOutputStream(send_filename);
                    while(thread != null) {
                        int n = opensl_example.pull(buf);
                        if (n > 0) {
                            ByteBuffer.wrap(bytes)
                                    .order(ByteOrder.LITTLE_ENDIAN)
                                    .asShortBuffer().put(buf);
                            fos.write(bytes);
                        } else {
                            sleep(FRAME_MS / 2);
                        }
                    }
                    fos.close();
                } catch (FileNotFoundException e) {
                } catch (IOException e) {
                } catch (InterruptedException e) {
                }
            }
        };

        int echo_delay_ms = getSharedPreferences("aec", 0).getInt("echo_delay_ms", -1);
        int aec= 0;
        if (Build.VERSION.SDK_INT >= 16) {
            if(AcousticEchoCanceler.isAvailable())
                aec = 1;
        }
        opensl_example.start(track_minbufsz, record_minbufsz, playback_delay, echo_delay_ms, dump_raw, aec);
        thread.start();
        thread2.start();
        thread3.start();
    }

    private void play_record_aec_stop() {
        saveDelay();

        opensl_example.stop();
        try {
            if (thread2 != null && thread2.isAlive()) {
                thread2.interrupt();
            }
            if (thread3 != null && thread3.isAlive()) {
                thread3.interrupt();
            }
            if (thread != null && thread.isAlive()) {
                thread.join();
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        thread = null;
        thread2 = null;
        thread3 = null;
    }

    private void saveDelay() {
        int echo_delay_ms = opensl_example.get_estimated_echo_delay();
        SharedPreferences.Editor edt = getSharedPreferences("aec", 0).edit();
        edt.putInt("echo_delay_ms", echo_delay_ms);
        edt.commit();
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

        if (Build.VERSION.SDK_INT >= 16) {
            Log.d(TAG, "Checks if the device implements acoustic echo cancellation: " +
                AcousticEchoCanceler.isAvailable());
        }
    }

    public void onDestroy(){
    	super.onDestroy();
        play_record_aec_stop();
    }

    @Override
    public void onClick(View view) {
        switch(view.getId()) {
            case R.id.btn_play_record:
                if (thread == null) {
                    play_record_aec_start();
                } else {
                    play_record_aec_stop();
                }
                break;
            case R.id.btn_play_far:
                onPlay((Button)(findViewById(R.id.btn_play_far)), far_filename);
                break;
            case R.id.btn_play_near:
                onPlay((Button)(findViewById(R.id.btn_play_near)), near_filename);
                break;
            case R.id.btn_play_send:
                onPlay((Button)(findViewById(R.id.btn_play_send)), send_filename);
                break;
            case R.id.btn_offline_est:
                opensl_example.estimate_delay(0);
                saveDelay();
                break;
            case R.id.btn_offline_aec:
            {
                long t = System.currentTimeMillis();
                opensl_example.offline_process();
                Log.i(TAG, "offline process, elapse " + (System.currentTimeMillis() - t) + "ms");
            }
                break;
        }
    }

    private void play(String filename) {
        AudioTrack player = new AudioTrack(AudioManager.STREAM_MUSIC,
                sampleRate,
                AudioFormat.CHANNEL_OUT_MONO, AudioFormat.ENCODING_PCM_16BIT,
                managerBufferSize, AudioTrack.MODE_STREAM);
        player.play();

        try {
            FileInputStream fis = new FileInputStream(filename);
            byte[] buf = new byte[managerBufferSize];
            while(isPlaying) {
                int n = fis.read(buf);
                if (n <= 0)
                    break;
                player.write(buf, 0, n);
            };
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void onPlay(final Button btn, final String filename) {
        if (isPlaying) {
            isPlaying = false;
            return;
        }

        final String origTxt = btn.getText().toString();
        btn.setText("Playing ...");
        isPlaying = true;
        new Thread(new Runnable() {
            @Override
            public void run() {
                play(filename);
                isPlaying = false;
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        btn.setText(origTxt);
                    }
                });
            }
        }).start();
    }

}