package com.example.smartstripv10;

import androidx.appcompat.app.AppCompatActivity;

import android.app.ProgressDialog;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Vibrator;
import android.view.View;

public class TeachInActivity extends AppCompatActivity {

    Vibrator vibrator;
    int outlet;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_teach_in);

        Context context = getApplicationContext();
        vibrator = (Vibrator) context.getSystemService(Context.VIBRATOR_SERVICE);

        Intent intent = getIntent();
        String OutletStr = intent.getStringExtra(MainActivity.OUTLETMESSAGE);
        outlet = Integer.parseInt(OutletStr);


    }

    public void onOnSampleButton(View view) {
        vibrator.vibrate(50);
        byte [] modeOnMsg = {'m', 1};
        MainActivity.sendCharacteristic(modeOnMsg);
        final ProgressDialog dialog = ProgressDialog.show(TeachInActivity.this, "", "Sending training example...", true, true);

        final Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if(MainActivity.atmegaOK) {
                    MainActivity.atmegaOK = false;
                    dialog.cancel();
                    handler.removeCallbacks(this);
                    return;
                }
                handler.postDelayed(this, 500); // run every 1/2 second
            }
        }, 500);
    }

    public void onOffSampleButton(View view) {
        vibrator.vibrate(50);
        byte [] modeOffMsg = {'m', 0};
        MainActivity.sendCharacteristic(modeOffMsg);

        final ProgressDialog dialog = ProgressDialog.show(TeachInActivity.this, "", "Sending training example...", true, true);

        final Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if(MainActivity.atmegaOK) {
                    MainActivity.atmegaOK = false;
                    dialog.cancel();
                    handler.removeCallbacks(this);
                    return;
                }
                handler.postDelayed(this, 500); // run every 1/2 second
            }
        }, 500);
    }

    public void onFinishButton(View view) {
        byte[] teachEndMsg = {'t', (byte)outlet, 0};
        MainActivity.sendCharacteristic(teachEndMsg);

        vibrator.vibrate(50);
        this.finish();
    }

}
