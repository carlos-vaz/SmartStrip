package com.example.smartstripv10;

import androidx.appcompat.app.AppCompatActivity;
import androidx.constraintlayout.widget.ConstraintLayout;
import androidx.constraintlayout.widget.ConstraintSet;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Vibrator;
import android.util.Log;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.TranslateAnimation;
import android.widget.Button;
import android.widget.TextView;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    // Image and animations
    public ConstraintLayout stripImageLayout;
    public ConstraintLayout topLayout;
    public ConstraintLayout outletInfoLayout;
    public ConstraintSet constraintSetImageRight;
    public ConstraintSet constraintSetImageCenter;
    public TranslateAnimation anim_to_right;
    public TranslateAnimation anim_to_center;
    volatile int imageRighted;
    int pixToSlide = 200;
    int animDuration = 500;

    // Override buttons
    Drawable defaultSmartButton;
    Drawable defaultOnButton;
    Drawable defaultOffButton;
    int LIGHTBLUE = Color.rgb(50,50,255);

    volatile boolean longClick;
    Vibrator vibrator;
    volatile int multiSelectArray[];
    volatile int multiSelectLength;

    // Device Array
    volatile ArrayList<DeviceInfo> deviceInfoArrayList;

    int zi_teachInOutlet = -1;

    volatile boolean waitingForResponse = false;
    volatile String globalResponse;

    BluetoothAdapter bluetoothAdapter;
    BluetoothLeService bluetoothLeService;
    public static BluetoothGatt gatt;
    private Handler mHandler;
    BluetoothAdapter.LeScanCallback scanCallback;
    private String deviceAddress;
    static boolean mConnected = false;
    private int REQUEST_ENABLE_BT = 0;
    private static final String TAG = "MainActivity";
    public static final String OUTLETMESSAGE = "SMARTSTRIP.OUTLET.MESSAGE";

    public static UUID SS_SERVICE_UUID = convertFromInteger(0xFFE0);
    public static UUID SS_CHARACTERISTIC_UUID = convertFromInteger(0xFFE1);

    public static boolean atmegaOK = false;

    public static DebugConsole debug;

    public int PROX_ON_THRESH = -60;
    public int PROX_OFF_THRESH = -75;
    public boolean prox_reset = false;

    // Code to manage Service lifecycle.
    private final ServiceConnection mServiceConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName componentName, IBinder service) {
            bluetoothLeService = ((BluetoothLeService.LocalBinder) service).getService();
            if (!bluetoothLeService.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
                finish();
            }
            // Automatically connects to the device upon successful start-up initialization.
            gatt = bluetoothLeService.connectSmartStrip(deviceAddress);
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            bluetoothLeService = null;
        }
    };


    //private final BroadcastReceiver mGattUpdateReceiver = new MyBroadcastReceiver();

    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            //Log.i(TAG, "RECEIVED A BROADCAST... Action = " + action);
            //debug.putText("RECEIVED A BROADCAST... Action = " + action);
            if (BluetoothLeService.ACTION_GATT_CONNECTED.equals(action)) {
                mConnected = true;
                Log.i(TAG, "BCast Receiver: GATT CONNECTED");
                debug.putText("BCast Receiver: GATT CONNECTED");
                startRssiMonitor();
            } else if (BluetoothLeService.ACTION_GATT_DISCONNECTED.equals(action)) {
                mConnected = false;
                Log.i(TAG, "BCast Receiver: GATT DISCONNECTED");
                debug.putText("BCast Receiver: GATT DISCONNECTED");
                unbindService(mServiceConnection);
                scanLeDevice(true);
            } else if (BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {
                Log.i(TAG, "BCast Receiver: GATT SERVICES DISCOVERED");
                debug.putText("BCast Receiver: GATT SERVICES DISCOVERED");
                List<BluetoothGattService> services =  gatt.getServices();
                for(BluetoothGattService service : services) {
                    Log.i(TAG, "Available Service: " + service.getUuid());
                    debug.putText("Available Service: " + service.getUuid());
                    if(service.getUuid().equals(SS_SERVICE_UUID)) {
                        gatt.setCharacteristicNotification(service.getCharacteristic(SS_CHARACTERISTIC_UUID), true);
                        Log.i(TAG, "Enabled notifications on Response Characteristic");
                        debug.putText("Enabled notifications on Response Characteristic");
                    }
                }
            } else if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {
                String msgFromAtmega = intent.getStringExtra(BluetoothLeService.EXTRA_DATA);
                handleAtmegaResponse(msgFromAtmega);
                /*if(waitingForResponse) {
                    globalResponse = msgFromAtmega;
                    waitingForResponse = false;
                }*/
                Log.i(TAG, "BCast Receiver: DATA AVAILABLE");
                debug.putText("ATMEGA: " + msgFromAtmega);
            } else if (BluetoothLeService.ACTION_RSSI_AVAILABLE.equals(action)) {
                //Log.i(TAG, "BCast Receiver: RSSI AVAILABLE");
                //debug.putText("BCast Receiver: RSSI AVAILABLE");
                int rssi = Integer.parseInt(intent.getStringExtra(BluetoothLeService.EXTRA_DATA));
                    if(rssi > PROX_ON_THRESH && prox_reset) {
                        debug.putText("PROX ON!");
                        byte[] proximityMsg = {'o', -1, 0, 0};
                        sendCharacteristic(proximityMsg);
                        prox_reset = false;
                    }
                    else if (rssi < PROX_OFF_THRESH) {
                        prox_reset = true;
                    }
            }
        }
    };

    public void bindToBleService() {
        Log.i(TAG, "TRYING TO BIND");

        //ComponentName myservice = startService(new Intent(this, BluetoothLeService.class));

        Intent gattServiceIntent = new Intent(this, BluetoothLeService.class);
        bindService(gattServiceIntent, mServiceConnection, BIND_AUTO_CREATE);
    }

    public void registerMyBroadcastReceiver() {
        IntentFilter filter = new IntentFilter(BluetoothLeService.ACTION_GATT_CONNECTED);
        filter.addAction(BluetoothLeService.ACTION_GATT_DISCONNECTED);
        filter.addAction(BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED);
        filter.addAction(BluetoothLeService.ACTION_DATA_AVAILABLE);
        filter.addAction(BluetoothLeService.ACTION_RSSI_AVAILABLE);
        this.registerReceiver(mGattUpdateReceiver, filter);
    }


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "Executing onCreate()");

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Make green circles invisible
        makeCirclesInvisible();

        stripImageLayout = findViewById(R.id.stripLayout);
        topLayout = findViewById(R.id.topLayout);
        outletInfoLayout = findViewById(R.id.outletInfoLayout);
        constraintSetImageRight = new ConstraintSet();
        constraintSetImageRight.clone(topLayout);
        constraintSetImageRight.setTranslationX(R.id.stripLayout, pixToSlide);
        constraintSetImageRight.setVisibility(R.id.outletInfoLayout, ConstraintSet.VISIBLE);
        constraintSetImageCenter = new ConstraintSet();
        constraintSetImageCenter.clone(topLayout);
        constraintSetImageCenter.setTranslationX(R.id.stripLayout, 0);

        anim_to_right = new TranslateAnimation(0, pixToSlide, 0, 0);
        anim_to_right.setDuration(animDuration);
        anim_to_right.setAnimationListener(new TranslateAnimation.AnimationListener() {
            @Override
            public void onAnimationStart(Animation animation) { }
            @Override
            public void onAnimationRepeat(Animation animation) { }
            @Override
            public void onAnimationEnd(Animation animation) {
                stripImageLayout.clearAnimation();
                constraintSetImageRight.applyTo(topLayout);
            }
        });

        anim_to_center = new TranslateAnimation(0,-1*pixToSlide, 0, 0);
        anim_to_center.setDuration(animDuration);
        anim_to_center.setAnimationListener(new TranslateAnimation.AnimationListener() {
            @Override
            public void onAnimationStart(Animation animation) { }
            @Override
            public void onAnimationRepeat(Animation animation) { }
            @Override
            public void onAnimationEnd(Animation animation) {
                stripImageLayout.clearAnimation();
                constraintSetImageCenter.applyTo(topLayout);
            }
        });

        imageRighted = 0;

        defaultSmartButton = findViewById(R.id.smartButton).getBackground();
        defaultOnButton = findViewById(R.id.onButton).getBackground();
        defaultOffButton = findViewById(R.id.offButton).getBackground();

        // Initialize device array
        loadFromDisk();

        if(deviceInfoArrayList == null) {
            // init blank array list of size 6
            deviceInfoArrayList = new ArrayList<>();
            deviceInfoArrayList.add(new DeviceInfo());
            deviceInfoArrayList.add(new DeviceInfo());
            deviceInfoArrayList.add(new DeviceInfo());
            deviceInfoArrayList.add(new DeviceInfo());
            deviceInfoArrayList.add(new DeviceInfo());
            deviceInfoArrayList.add(new DeviceInfo());
        }


        // From device array, load statuses and draw them on strip icon
        updateDeviceStatusColors();

        longClick = false;
        Context context = getApplicationContext();
        vibrator = (Vibrator) context.getSystemService(Context.VIBRATOR_SERVICE);

        // Multi-select array of devices
        multiSelectArray = new int[2];
        multiSelectLength = 0;

        // Button on long click listeners
        Button outlet1Button = findViewById(R.id.outlet1button);
        Button outlet2Button = findViewById(R.id.outlet2button);
        Button outlet3Button = findViewById(R.id.outlet3button);
        Button outlet4Button = findViewById(R.id.outlet4button);
        Button outlet5Button = findViewById(R.id.outlet5button);
        Button outlet6Button = findViewById(R.id.outlet6button);

        outlet1Button.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View view) {
                longClick = false;
                clearMultiSelect();
                onOutlet1ButtonClick(view);
                longClick = true;
                vibrator.vibrate(50);
                return false;
            }
        });
        outlet2Button.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View view) {
                longClick = false;
                clearMultiSelect();
                onOutlet2ButtonClick(view);
                longClick = true;
                vibrator.vibrate(50);
                return false;
            }
        });
        outlet3Button.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View view) {
                longClick = false;
                clearMultiSelect();
                onOutlet3ButtonClick(view);
                longClick = true;
                vibrator.vibrate(50);
                return false;
            }
        });
        outlet4Button.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View view) {
                longClick = false;
                clearMultiSelect();
                onOutlet4ButtonClick(view);
                longClick = true;
                vibrator.vibrate(50);
                return false;
            }
        });
        outlet5Button.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View view) {
                longClick = false;
                clearMultiSelect();
                onOutlet5ButtonClick(view);
                longClick = true;
                vibrator.vibrate(50);
                return false;
            }
        });
        outlet6Button.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View view) {
                longClick = false;
                clearMultiSelect();
                onOutlet6ButtonClick(view);
                longClick = true;
                vibrator.vibrate(50);
                return false;
            }
        });

        mHandler = new Handler();

        // Initializes Bluetooth adapter.
        final BluetoothManager bluetoothManager =
                (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = bluetoothManager.getAdapter();

        // Ensures Bluetooth is available on the device and it is enabled. If not,
        // displays a dialog requesting user permission to enable Bluetooth.
        if (bluetoothAdapter == null || !bluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }

        scanCallback =
                new BluetoothAdapter.LeScanCallback() {
                    @Override
                    public void onLeScan(final BluetoothDevice device, int rssi,
                                         byte[] scanRecord) {
                        Log.i(TAG, "Found device with name " + device.getName() + " and address " + device.getAddress());
                        String devname = device.getName();
                        if(devname != null && devname.equals("SmartStrip")) {
                            deviceAddress = device.getAddress();
                            Log.i(TAG, "ENTERED EQUALS");
                            scanLeDevice(false);

                            // Bind to the Bluetooth service
                            bindToBleService();

                            // Register the Broadcast Receiver
                            registerMyBroadcastReceiver();

                        } else {
                            Log.i(TAG, "ENTERED NOT EQUALS");
                        }
                    }
                };

        Log.i(TAG, "Scanning for Bluetooth Devices...");
        debug = new DebugConsole();
        debug.putText("Scanning for Bluetooth Devices...");
        scanLeDevice(true);

    }

    @Override
    protected void onPause() {
        Log.i("MY LOG: ", "EXECUTING METHOD ONPAUSE()");
        super.onPause();
        saveToDisk();
    }

    @Override
    protected void onResume() {
        Log.i("MY LOG: ", "EXECUTING METHOD ONRESUME()");
        super.onResume();
        loadFromDisk();
    }


    @Override
    public void onSaveInstanceState(Bundle savedInstanceState) {
        super.onSaveInstanceState(savedInstanceState);
        // Save UI state changes to the savedInstanceState.
        // This bundle will be passed to onCreate if the process is
        // killed and restarted.
        savedInstanceState.putParcelableArrayList("deviceInfoArrayList", deviceInfoArrayList);
        saveToDisk();
    }

    @Override
    public void onRestoreInstanceState(Bundle savedInstanceState) {
        super.onRestoreInstanceState(savedInstanceState);
        // Restore UI state from the savedInstanceState.
        // This bundle has also been passed to onCreate.
        deviceInfoArrayList = savedInstanceState.getParcelableArrayList("deviceInfoArrayList");
    }

    public void onOutlet1ButtonClick(View view) {
        int thisAnimDuration = 0;

        if(imageRighted == 0) {
            stripImageLayout.startAnimation(anim_to_right);
            thisAnimDuration = animDuration/2;
            imageRighted = 1;
        }

        deliverDeviceInfo(1, thisAnimDuration);

        if(longClick && multiSelectLength<2) {
            addMultiSelect(1);
        } else {
            makeCirclesInvisible();
            clearMultiSelect();
            longClick = false;
        }
        if(multiSelectLength == 2) {
            offerSwapAction();
        }
        findViewById(R.id.circle1).setVisibility(View.VISIBLE);
    }

    public void onOutlet2ButtonClick(View view) {
        int thisAnimDuration = 0;

        if(imageRighted == 0) {
            stripImageLayout.startAnimation(anim_to_right);
            thisAnimDuration = animDuration/2;
            imageRighted = 1;
        }

        deliverDeviceInfo(2, thisAnimDuration);

        if(longClick && multiSelectLength<2) {
            addMultiSelect(2);
        } else {
            makeCirclesInvisible();
            clearMultiSelect();
            longClick = false;
        }
        if(multiSelectLength == 2) {
            offerSwapAction();
        }
        findViewById(R.id.circle2).setVisibility(View.VISIBLE);
    }

    public void onOutlet3ButtonClick(View view) {
        int thisAnimDuration = 0;

        if(imageRighted == 0) {
            stripImageLayout.startAnimation(anim_to_right);
            thisAnimDuration = animDuration/2;
            imageRighted = 1;
        }

        deliverDeviceInfo(3, thisAnimDuration);

        if(longClick && multiSelectLength<2) {
            addMultiSelect(3);
        } else {
            makeCirclesInvisible();
            clearMultiSelect();
            longClick = false;
        }
        if(multiSelectLength == 2) {
            offerSwapAction();
        }
        findViewById(R.id.circle3).setVisibility(View.VISIBLE);
    }

    public void onOutlet4ButtonClick(View view) {
        int thisAnimDuration = 0;

        if(imageRighted == 0) {
            stripImageLayout.startAnimation(anim_to_right);
            thisAnimDuration = animDuration/2;
            imageRighted = 1;
        }

        deliverDeviceInfo(4, thisAnimDuration);

        if(longClick && multiSelectLength<2) {
            addMultiSelect(4);
        } else {
            makeCirclesInvisible();
            clearMultiSelect();
            longClick = false;
        }
        if(multiSelectLength == 2) {
            offerSwapAction();
        }
        findViewById(R.id.circle4).setVisibility(View.VISIBLE);
    }

    public void onOutlet5ButtonClick(View view) {
        int thisAnimDuration = 0;

        if(imageRighted == 0) {
            stripImageLayout.startAnimation(anim_to_right);
            thisAnimDuration = animDuration/2;
            imageRighted = 1;
        }

        deliverDeviceInfo(5, thisAnimDuration);

        if(longClick && multiSelectLength<2) {
            addMultiSelect(5);
        } else {
            makeCirclesInvisible();
            clearMultiSelect();
            longClick = false;
        }
        if(multiSelectLength == 2) {
            offerSwapAction();
        }
        findViewById(R.id.circle5).setVisibility(View.VISIBLE);
    }

    public void onOutlet6ButtonClick(View view) {
        int thisAnimDuration = 0;

        if(imageRighted == 0) {
            stripImageLayout.startAnimation(anim_to_right);
            thisAnimDuration = animDuration/2;
            imageRighted = 1;
        }

        deliverDeviceInfo(6, thisAnimDuration);

        if(longClick && multiSelectLength<2) {
            addMultiSelect(6);
        } else {
            makeCirclesInvisible();
            clearMultiSelect();
            longClick = false;
        }
        if(multiSelectLength == 2) {
            offerSwapAction();
        }
        findViewById(R.id.circle6).setVisibility(View.VISIBLE);
    }

    private void deliverDeviceInfo(final int outlet, int delay) {
        if(multiSelectLength==1)
            return;

        final DeviceInfo thisDevice = deviceInfoArrayList.get(outlet-1);

        Handler handler = new Handler();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {
                TextView foundTextView = findViewById(R.id.deviceName);
                foundTextView.setText(thisDevice.Name);
                foundTextView = findViewById(R.id.outletNumber);
                foundTextView.setText(Integer.toString(outlet));
                ConstraintLayout foundLayout = findViewById(R.id.swapActionLayout);
                foundLayout.setVisibility(View.GONE);
                foundLayout = findViewById(R.id.outletInfoLayout);
                foundLayout.setVisibility(View.VISIBLE);

                foundTextView = findViewById(R.id.thresholdValue);
                if(thisDevice.Threshold != -1) {
                    foundTextView.setText(Integer.toString(thisDevice.Threshold));
                } else {
                    foundTextView.setText("");
                }

                foundTextView = findViewById(R.id.onoffCountTextView);
                foundTextView.setText(Integer.toString(thisDevice.OnCount) + " / " + Integer.toString(thisDevice.OffCount));


                switch(thisDevice.Status) {
                    case 0:
                        findViewById(R.id.offButton).setBackgroundColor(Color.RED);
                        findViewById(R.id.onButton).setBackground(defaultOnButton);
                        findViewById(R.id.smartButton).setBackground(defaultSmartButton);
                        findViewById(R.id.teachButton).setVisibility(View.INVISIBLE);
                        break;
                    case 1:
                        findViewById(R.id.offButton).setBackground(defaultOffButton);
                        findViewById(R.id.onButton).setBackgroundColor(LIGHTBLUE);
                        findViewById(R.id.smartButton).setBackground(defaultSmartButton);
                        findViewById(R.id.teachButton).setVisibility(View.INVISIBLE);
                        break;
                    case 2:
                        findViewById(R.id.offButton).setBackground(defaultOffButton);
                        findViewById(R.id.onButton).setBackground(defaultOnButton);
                        findViewById(R.id.smartButton).setBackgroundColor(Color.GREEN);
                        findViewById(R.id.teachButton).setVisibility(View.VISIBLE);
                        break;
                }

            }
        }, delay);
    }

    public void onBackgroundClick(View view) {
        ConstraintLayout foundLayout = findViewById(R.id.outletInfoLayout);
        foundLayout.setVisibility(View.GONE);
        foundLayout = findViewById(R.id.swapActionLayout);
        foundLayout.setVisibility(View.GONE);

        if(imageRighted == 1) {
            stripImageLayout.startAnimation(anim_to_center);
            imageRighted = 0;
        }

        makeCirclesInvisible();

        longClick = false;
        clearMultiSelect();
    }

    private void makeCirclesInvisible() {
        findViewById(R.id.circle1).setVisibility(View.INVISIBLE);
        findViewById(R.id.circle2).setVisibility(View.INVISIBLE);
        findViewById(R.id.circle3).setVisibility(View.INVISIBLE);
        findViewById(R.id.circle4).setVisibility(View.INVISIBLE);
        findViewById(R.id.circle5).setVisibility(View.INVISIBLE);
        findViewById(R.id.circle6).setVisibility(View.INVISIBLE);
    }

    private void updateDeviceStatusColors() {
        for(int i=0; i<6; i++) {
            int color = Color.GRAY;

            switch(deviceInfoArrayList.get(i).Status) {
                case 0:
                    color = Color.RED;
                    break;
                case 1:
                    color = Color.rgb(0, 0, 255);
                    break;
                case 2:
                    color = Color.GREEN;
                    break;
            }

            TextView view = findViewById(R.id.outlet1Label);
            switch(i) {
                case 0:
                    view = findViewById(R.id.outlet1Label);
                   break;
                case 1:
                    view = findViewById(R.id.outlet2Label);
                    break;
                case 2:
                    view = findViewById(R.id.outlet3Label);
                    break;
                case 3:
                    view = findViewById(R.id.outlet4Label);
                    break;
                case 4:
                    view = findViewById(R.id.outlet5Label);
                    break;
                case 5:
                    view = findViewById(R.id.outlet6Label);
                    break;
            }
            view.setTextColor(color);
        }

    }

    public void onSmartButtonClick(View view) {
        TextView outletTextView = findViewById(R.id.outletNumber);
        int zi_outlet = Integer.parseInt(outletTextView.getText().toString()) - 1;

        //Log.i(TAG, "ONSMARTBUTTONCLICK: Outlet number = " + zi_outlet);
        byte[] smartMsg = {'o', (byte)zi_outlet, 0, 1};
        sendCharacteristic(smartMsg);

        deviceInfoArrayList.get(zi_outlet).Status = 2;

        view.setBackgroundColor(Color.GREEN);
        findViewById(R.id.onButton).setBackground(defaultOnButton);
        findViewById(R.id.offButton).setBackground(defaultOffButton);

        updateDeviceStatusColors();
        deliverDeviceInfo(zi_outlet + 1,0);
    }

    public void onOnButtonClick(View view) {
        TextView outletTextView = findViewById(R.id.outletNumber);
        int zi_outlet = Integer.parseInt(outletTextView.getText().toString()) - 1;
        //Log.i(TAG, "ONSMARTBUTTONCLICK: Outlet number = " + zi_outlet);

        byte[] onMsg = {'o', (byte)zi_outlet, 1, 0};
        sendCharacteristic(onMsg);
        /*waitingForResponse = true;

        while(waitingForResponse);

        if(globalResponse.equals("k")) { */

            deviceInfoArrayList.get(zi_outlet).Status = 1;

            view.setBackgroundColor(LIGHTBLUE);
            findViewById(R.id.smartButton).setBackground(defaultSmartButton);
            findViewById(R.id.offButton).setBackground(defaultOffButton);

            updateDeviceStatusColors();
            deliverDeviceInfo(zi_outlet + 1,0);

        // }
    }

    public void onOffButtonClick(View view) {
        TextView outletTextView = findViewById(R.id.outletNumber);
        int zi_outlet = Integer.parseInt(outletTextView.getText().toString()) - 1;
        //Log.i(TAG, "ONSMARTBUTTONCLICK: Outlet number = " + zi_outlet);

        byte[] onMsg = {'o', (byte)zi_outlet, 0, 0};
        sendCharacteristic(onMsg);
        /*waitingForResponse = true;

        while(waitingForResponse);

        if(globalResponse.equals("k")) {*/

            deviceInfoArrayList.get(zi_outlet).Status = 0;

            view.setBackgroundColor(Color.RED);
            findViewById(R.id.smartButton).setBackground(defaultSmartButton);
            findViewById(R.id.onButton).setBackground(defaultOnButton);

            updateDeviceStatusColors();

            deliverDeviceInfo(zi_outlet + 1,0);

        //}
    }

    private void addMultiSelect(int outlet) {
        if(multiSelectLength<2)
            multiSelectArray[multiSelectLength++] = outlet;
    }

    private void clearMultiSelect() {
        multiSelectArray[0] = 0;
        multiSelectArray[1] = 0;
        multiSelectLength = 0;
    }

    private void offerSwapAction() {
        ConstraintLayout foundLayout = findViewById(R.id.outletInfoLayout);
        foundLayout.setVisibility(View.GONE);
        foundLayout = findViewById(R.id.swapActionLayout);
        foundLayout.setVisibility(View.VISIBLE);

        Log.i("LOG: ", "Value of multiSelectArray = [" + multiSelectArray[0] + ", " + multiSelectArray[1] + "]");
        TextView textView = findViewById(R.id.swapDev1Label);
        textView.setText("Outlet " + multiSelectArray[0] + ": " + deviceInfoArrayList.get(multiSelectArray[0]-1).Name);
        textView = findViewById(R.id.swapDev2Label);
        textView.setText("Outlet " + multiSelectArray[1] + ": " + deviceInfoArrayList.get(multiSelectArray[1]-1).Name);
        textView = findViewById(R.id.swapActionTitle);
        textView.setText("Swap Outlets");
    }

    public void onSwapButtonClick(View view) {
        Collections.swap(deviceInfoArrayList, multiSelectArray[0]-1, multiSelectArray[1]-1);
        updateDeviceStatusColors();
    }

    public void onAddDeviceButton(View view) {
        Intent intent = new Intent(this, AddDeviceActivity.class);
        startActivityForResult(intent, 10);
    }

    public void onTeachInButton(View view) {
        TextView outletTextView = findViewById(R.id.outletNumber);
        int zi_outlet = Integer.parseInt(outletTextView.getText().toString()) - 1;
        zi_teachInOutlet = zi_outlet;

        if(deviceInfoArrayList.get(zi_outlet).Status == 2) {

            byte[] teachMsg = {'t', (byte) zi_outlet, 1};
            sendCharacteristic(teachMsg);

            Intent intent = new Intent(this, TeachInActivity.class);
            intent.putExtra(OUTLETMESSAGE, Integer.toString(zi_outlet));
            startActivity(intent);
        }

    }

    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == 10) {
            int type = data.getIntExtra("type", -1);

            if(type == 0) {
                // Collect name string from "add new device" activity
                String newName = data.getStringExtra("newDeviceName");

                // Get which outlet
                TextView outletTextView = findViewById(R.id.outletNumber);
                int outlet = Integer.parseInt(outletTextView.getText().toString());

                // Set DeviceInfo for this outlet to the name
                DeviceInfo newDeviceInfo = new DeviceInfo();
                newDeviceInfo.Name = newName;
                DeviceInfo oldDeviceInfo = deviceInfoArrayList.get(outlet - 1);
                deviceInfoArrayList.set(outlet - 1, newDeviceInfo);
                deliverDeviceInfo(outlet, 0);

                // Save new device to permanent storage
                saveToDisk();

                // Store oldDevice into permanent storage
                storePastDevicePermanent(oldDeviceInfo);
            }else if(type == 1) {
                int oldDevIndex = data.getIntExtra("oldDeviceIndex", -1);

                ArrayList<DeviceInfo> pastDeviceInfoArrayList = null;

                try {
                    FileInputStream inputStream = openFileInput("PastDeviceData.data");
                    ObjectInputStream inobj = new ObjectInputStream(inputStream);
                    pastDeviceInfoArrayList = (ArrayList<DeviceInfo>) inobj.readObject();
                    inputStream.close();
                } catch (Exception e) {
                    Log.i(TAG, "Error Reading PastDeviceData.data");
                    e.printStackTrace();
                }
                if(pastDeviceInfoArrayList != null) {
                    DeviceInfo incomingExistingDev = pastDeviceInfoArrayList.get(oldDevIndex);

                    // Get which outlet
                    TextView outletTextView = findViewById(R.id.outletNumber);
                    int outlet = Integer.parseInt(outletTextView.getText().toString());

                    // Set DeviceInfo for this outlet to the name
                    DeviceInfo oldDeviceInfo = deviceInfoArrayList.get(outlet - 1);
                    deviceInfoArrayList.set(outlet - 1, incomingExistingDev);
                    deliverDeviceInfo(outlet, 0);

                    // Save new device to permanent storage
                    saveToDisk();

                    // Store oldDevice into permanent storage
                    storePastDevicePermanent(oldDeviceInfo);
                }

            }
        }
    }

    // Saves global var "deviceInfoArrayList" to a file
    public void saveToDisk() {
        try {
            Context context = getApplicationContext();
            File outCurrent = new File(context.getFilesDir(), "CurrentDeviceData.data");
            Log.i("MY LOG: ", "FILE DIR = " + context.getFilesDir());
            if(!outCurrent.exists()) {
                Log.i("MY LOG: ", "CREATING NEW FILE in dir = " + context.getFilesDir());
                outCurrent.createNewFile();
                Log.i("MY LOG: ", "DONE CREATING NEW FILE");
            }

            FileOutputStream outFileStream  = openFileOutput("CurrentDeviceData.data", Context.MODE_PRIVATE);
            ObjectOutputStream outobj = new ObjectOutputStream(outFileStream);
            outobj.writeObject(deviceInfoArrayList);
            outFileStream.close();

        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    // Loads from file into global var "deviceInfoArrayList"
    public void loadFromDisk() {
            try {

                Context context = getApplicationContext();
                File outCurrent = new File(context.getFilesDir(), "CurrentDeviceData.data");
                Log.i("MY LOG: ", "FILE DIR = " + context.getFilesDir());

                // If array file does not exist, create new one
                if(!outCurrent.exists()) {
                    deviceInfoArrayList = new ArrayList<>();
                    for(int i=0; i<6; i++) {
                        deviceInfoArrayList.add(new DeviceInfo());
                    }
                } else {
                // Otherwise, fill from file
                    FileInputStream inputStream = openFileInput("CurrentDeviceData.data");
                    ObjectInputStream inobj = new ObjectInputStream(inputStream);
                    deviceInfoArrayList = (ArrayList<DeviceInfo>) inobj.readObject();
                    inputStream.close();
                }

            } catch (Exception e) {
                e.printStackTrace();
            }

    }

    public void storePastDevicePermanent(DeviceInfo oldDev) {
        try {
            Context context = getApplicationContext();
            File out = new File(context.getFilesDir(), "PastDeviceData.data");
            if (!out.exists()) {
                out.createNewFile();
            }

            ArrayList<DeviceInfo> pastDeviceInfoArrayList = null;

            try {
                FileInputStream inputStream = openFileInput("PastDeviceData.data");
                ObjectInputStream inobj = new ObjectInputStream(inputStream);
                pastDeviceInfoArrayList = (ArrayList<DeviceInfo>) inobj.readObject();
                inputStream.close();
            } catch (Exception e) {
                Log.i(TAG, "Error Reading PastDeviceData.data");
                e.printStackTrace();
            }

                if(pastDeviceInfoArrayList == null) {
                    pastDeviceInfoArrayList = new ArrayList<>();
                }

                boolean exists = false;
                for(DeviceInfo dev : pastDeviceInfoArrayList) {
                    if(dev.DevID == oldDev.DevID) {
                        exists = true;
                    }
                }

                if(! exists && ! oldDev.Name.equals("Blank Outlet")) {
                    pastDeviceInfoArrayList.add(oldDev);
                }



            FileOutputStream outFileStream = openFileOutput("PastDeviceData.data", Context.MODE_PRIVATE);
            ObjectOutputStream outobj = new ObjectOutputStream(outFileStream);
            outobj.writeObject(pastDeviceInfoArrayList);
            outFileStream.close();
        } catch (Exception e) {e.printStackTrace();}

    }

    public void startRssiMonitor() {
        mHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                gatt.readRemoteRssi();
                mHandler.postDelayed(this, 500); // run every 1/2 second
            }
        }, 2000);
    }

    Runnable backgroundScan;

    private void scanLeDevice(final boolean enable) {
        if(enable) {
            final Handler innerHandler = new Handler();

            mHandler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    if(! mConnected) {

                        backgroundScan = new Runnable() {
                            @Override
                            public void run() {
                                bluetoothAdapter.stopLeScan(scanCallback);
                            }
                        };

                        innerHandler.postDelayed(backgroundScan, 1000); // Scan for 1 second

                        bluetoothAdapter.startLeScan(scanCallback);

                        mHandler.postDelayed(this, 5000); // run every 5 seconds
                    }
                }
            }, 1000);
        } else {
            bluetoothAdapter.stopLeScan(scanCallback);
        }
    }

 /*   private void scanLeDevice(final boolean enable) {
        if (enable) {
            // scan for 1 second, pause for 10 seconds, then repeat
            bluetoothAdapter.startLeScan(scanCallback);

            mHandler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    bluetoothAdapter.stopLeScan(scanCallback);
                }
            }, 1000); // Scan for 1 second

            bluetoothAdapter.stopLeScan(scanCallback);

            mHandler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    scanLeDevice(true);
                }
            }, 10000); // wait for 10 seconds

        } else {
            bluetoothAdapter.stopLeScan(scanCallback);
        }
    } */


    public static void sendCharacteristic(byte[] data) {
        if(mConnected) {
            BluetoothGattService service = gatt.getService(SS_SERVICE_UUID);
            if(service == null)
                return;
            BluetoothGattCharacteristic characteristic = service.getCharacteristic(SS_CHARACTERISTIC_UUID);
            characteristic.setValue(data);
            gatt.writeCharacteristic(characteristic);

            final StringBuilder hexData = new StringBuilder(data.length);
            for (byte byteChar : data)
                hexData.append(String.format("%02X ", byteChar));
            debug.putText(new String(data) + "\n" + hexData.toString());
        } else {
            debug.putText("Attempted to send characteristic when BT disconnected");
        }
    }


    public static UUID convertFromInteger(int i) {
        final long MSB = 0x0000000000001000L;
        final long LSB = 0x800000805f9b34fbL;
        long value = i & 0xFFFFFFFF;
        return new UUID(MSB | (value << 32), LSB);
    }

    public void onDebugButton(View view) {
        Intent intent = new Intent(this, DebugConsoleActivity.class);
        startActivity(intent);
    }

    public void handleAtmegaResponse(String atmegaResp) {


        char[] respBytes = atmegaResp.toCharArray();

        if(respBytes[0] == 'k' ) { //&& respBytes[1] == 0) {
            // OK message received
            Log.i(TAG, "RECEIVED OK FROM ATMEGA");
            atmegaOK = true;
        }

        char[] thresholdBytes = atmegaResp.toCharArray();

        if(thresholdBytes[0] == 'I' && thresholdBytes[1] == 'D') {
            // Threshold message received

            /*int val = (((int) thresholdBytes[3]) << 8) & ((int) thresholdBytes[2]);
            int onval = (((int) thresholdBytes[5]) << 8) & ((int) thresholdBytes[4]);
            int offval = (((int) thresholdBytes[7]) << 8) & ((int) thresholdBytes[6]);
            int oncount = (((int) thresholdBytes[9]) << 8) & ((int) thresholdBytes[8]);
            int offcount = (((int) thresholdBytes[11]) << 8) & ((int) thresholdBytes[10]);
            int minon = (((int) thresholdBytes[13]) << 8) & ((int) thresholdBytes[12]);
            int maxoff = (((int) thresholdBytes[15]) << 8) & ((int) thresholdBytes[14]);
            int suffex = (int) thresholdBytes[16];*/

            int val = (((int) thresholdBytes[3]) * 256) + ((int) thresholdBytes[2]);
            int onval = (((int) thresholdBytes[5]) * 256) + ((int) thresholdBytes[4]);
            int offval = (((int) thresholdBytes[7]) * 256) + ((int) thresholdBytes[6]);
            int oncount = (((int) thresholdBytes[9]) * 256) + ((int) thresholdBytes[8]);
            int offcount = (((int) thresholdBytes[11]) * 256) + ((int) thresholdBytes[10]);
            int minon = (((int) thresholdBytes[13]) * 256) + ((int) thresholdBytes[12]);
            int maxoff = (((int) thresholdBytes[15]) * 256) + ((int) thresholdBytes[14]);
            int suffex = (int) thresholdBytes[16];

            deviceInfoArrayList.get(zi_teachInOutlet).Threshold = val;
            deviceInfoArrayList.get(zi_teachInOutlet).OnVal = onval;
            deviceInfoArrayList.get(zi_teachInOutlet).OffVal = offval;
            deviceInfoArrayList.get(zi_teachInOutlet).OnCount = oncount;
            deviceInfoArrayList.get(zi_teachInOutlet).OffCount = offcount;
            deviceInfoArrayList.get(zi_teachInOutlet).MinOn = minon;
            deviceInfoArrayList.get(zi_teachInOutlet).MaxOff = maxoff;
            deviceInfoArrayList.get(zi_teachInOutlet).Suffex = suffex;

            deliverDeviceInfo(zi_teachInOutlet + 1, 0);

            debug.putText("Put threshold val = " + val + " into outlet = " + zi_teachInOutlet);

        }
    }

}
