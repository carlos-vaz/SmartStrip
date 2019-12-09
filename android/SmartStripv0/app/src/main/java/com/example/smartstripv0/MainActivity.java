package com.example.smartstripv0;

import androidx.appcompat.app.AppCompatActivity;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.Switch;
import android.widget.TextView;

import java.util.List;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    BluetoothAdapter bluetoothAdapter;
    BluetoothLeService bluetoothLeService;
    BluetoothGatt gatt;
    private Handler mHandler;
    BluetoothAdapter.LeScanCallback scanCallback;
    private String deviceAddress;
    boolean mConnected = false;
    private int REQUEST_ENABLE_BT = 0;
    private static final String TAG = "MainActivity";
    //public UUID SS_SERVICE_UUID = UUID.fromString("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    //public UUID SS_QUERY_CHARACTERISTIC_UUID = UUID.fromString("cf403526-dc8b-11e9-8a34-2a2ae2dbcce4");
    //public UUID SS_RESPONSE_CHARACTERISTIC_UUID = UUID.fromString("beb5483e-36e1-4688-b7f5-ea07361b26a8");
    public UUID SS_SERVICE_UUID = convertFromInteger(0xFFE0);
    public UUID SS_CHARACTERISTIC_UUID = convertFromInteger(0xFFE1);


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
            Log.i(TAG, "RECEIVED A BROADCAST... Action = " + action);
            if (BluetoothLeService.ACTION_GATT_CONNECTED.equals(action)) {
                mConnected = true;
                Log.i(TAG, "BCast Receiver: GATT CONNECTED");
                startRssiMonitor();
            } else if (BluetoothLeService.ACTION_GATT_DISCONNECTED.equals(action)) {
                mConnected = false;
                Log.i(TAG, "BCast Receiver: GATT DISCONNECTED");
                // TODO: re-connect to the GATT server
            } else if (BluetoothLeService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {
                Log.i(TAG, "BCast Receiver: GATT SERVICES DISCOVERED");
                List<BluetoothGattService> services =  gatt.getServices();
                for(BluetoothGattService service : services) {
                    Log.i(TAG, "Available Service: " + service.getUuid());
                    if(service.getUuid().equals(SS_SERVICE_UUID)) {
                        gatt.setCharacteristicNotification(service.getCharacteristic(SS_CHARACTERISTIC_UUID), true);
                        Log.i(TAG, "Enabled notifications on Response Characteristic");
                    }
                }
            } else if (BluetoothLeService.ACTION_DATA_AVAILABLE.equals(action)) {
                Log.i(TAG, "BCast Receiver: DATA AVAILABLE");
                displayCharacteristic(intent.getStringExtra(BluetoothLeService.EXTRA_DATA));
            } else if (BluetoothLeService.ACTION_RSSI_AVAILABLE.equals(action)) {
                Log.i(TAG, "BCast Receiver: RSSI AVAILABLE");
                int rssi = Integer.parseInt(intent.getStringExtra(BluetoothLeService.EXTRA_DATA));
                TextView foundTextView = findViewById(R.id.foundTextView);
                foundTextView.setText("RSSI: " + intent.getStringExtra(BluetoothLeService.EXTRA_DATA));

                Switch prox_switch = findViewById(R.id.prox_switch);
                if(prox_switch.isChecked()) {
                    if(rssi > -65) {
                        sendCharacteristic("on".getBytes());
                    }
                    else {
                        sendCharacteristic("off".getBytes());
                    }
                }

            }
        }
    };

    public void bindToBleService() {
        Log.i(TAG, "TRYING TO BIND");
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
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
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
                            TextView foundTextView = findViewById(R.id.foundTextView);
                            foundTextView.setText("RSSI: " + String.valueOf(rssi));
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
    }

    public void onFindStripButtonClick(View view) {
        TextView foundTextView = findViewById(R.id.foundTextView);
        foundTextView.setText("Scanning...");
        scanLeDevice(true);
    }

    public void onSendCommandButtonClick(View view) {
        EditText input = (EditText)findViewById(R.id.commandInput);
        sendCharacteristic(input.getText().toString().getBytes());
        //boolean ok = gatt.readRemoteRssi();
        //Log.i(TAG, "Tried to read remote RSSI... ok = " + ok);
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

    private void scanLeDevice(final boolean enable) {
        if (enable) {
            // Stops scanning after a pre-defined scan period.
            mHandler.postDelayed(new Runnable() {
                @Override
                public void run() {
                    bluetoothAdapter.stopLeScan(scanCallback);
                }
            }, 10000); // Scan for 10 seconds

            bluetoothAdapter.startLeScan(scanCallback);
        } else {
            bluetoothAdapter.stopLeScan(scanCallback);
        }
    }


    private void sendCharacteristic(byte[] data) {
        BluetoothGattService service =  gatt.getService(SS_SERVICE_UUID);
        BluetoothGattCharacteristic characteristic = service.getCharacteristic(SS_CHARACTERISTIC_UUID);
        characteristic.setValue(data);
        gatt.writeCharacteristic(characteristic);
    }

    private void displayCharacteristic(String data) {
        TextView foundTextView = findViewById(R.id.dispData);
        foundTextView.setText(data);
    }

    public UUID convertFromInteger(int i) {   final long MSB = 0x0000000000001000L;
        final long LSB = 0x800000805f9b34fbL;
        long value = i & 0xFFFFFFFF;
        return new UUID(MSB | (value << 32), LSB);}

}



