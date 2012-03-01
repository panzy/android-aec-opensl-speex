package com.audiotest;

import opensl_example.opensl_example;
import android.app.Activity;
import android.os.Bundle;

public class AudiotestActivity extends Activity {
    /** Called when the activity is first created. */
	Thread thread;
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        thread = new Thread() {
			public void run() {
				setPriority(Thread.MAX_PRIORITY);
				opensl_example.start_process();
			}
		};
		thread.start();   
    }
    public void onDestroy(){
    	
    	super.onDestroy();
    	opensl_example.stop_process();
    	try {
			thread.join();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    	thread = null;
    	
    }
}