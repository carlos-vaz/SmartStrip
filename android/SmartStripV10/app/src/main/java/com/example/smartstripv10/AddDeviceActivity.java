package com.example.smartstripv10;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;
import android.view.ViewGroup.LayoutParams;

import java.io.File;
import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.ArrayList;

public class AddDeviceActivity extends AppCompatActivity {

    public int i;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_add_device);

        // Load old device names from disk
        Context context = getApplicationContext();
        File in = new File(context.getFilesDir(), "PastDeviceData.data");
        if (in.exists()) {

            try {
                FileInputStream inputStream = openFileInput("PastDeviceData.data");
                ObjectInputStream inobj = new ObjectInputStream(inputStream);
                final ArrayList<DeviceInfo> pastDeviceInfoArrayList = (ArrayList<DeviceInfo>) inobj.readObject();
                inputStream.close();

                Log.i("ADD-DEVICE-ACTIVITY", "pastDeviceInfoArrayList size = " + pastDeviceInfoArrayList.size());

                //ScrollView sv = findViewById(R.id.existingDeviceScrollView);
                LinearLayout linearLayout = findViewById(R.id.existingDeviceslinearLayout);

                // Create buttons for each old device
                i=0;
                for(DeviceInfo di : pastDeviceInfoArrayList) {
                    Button btn = new Button(this);
                    btn.setLayoutParams(new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT));
                    btn.setText(di.Name);
                    btn.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            Intent intent = new Intent();
                            intent.putExtra("type", 1);
                            intent.putExtra("oldDeviceIndex", i);
                            setResult(RESULT_OK, intent);
                            getParent().finish();
                        }

                    });
                    linearLayout.addView(btn);
                    i++;
                    // TODO: Add onClickListener() that swaps devices and re-writes to PastDeviceData.data
                }

            } catch (Exception e) {
                Log.i("ADD-DEVICE-ACTIVITY: ", "Error reading PastDeviceData.data or creating BUTTONS");
                e.printStackTrace();
            }
        }

    }

    public void onAddNewDeviceButton(View view) {
        Intent intent = new Intent(this, AddNewDeviceActivity.class);
        startActivityForResult(intent, 10);
    }

    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Collect name string from AddNewDeviceActivity name console
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode == 10) {
            if(resultCode == RESULT_OK) {
                String newName = data.getStringExtra("newDeviceName");

                // Send back new device name to parent activity
                Intent intent = new Intent();
                intent.putExtra("type", 0);
                intent.putExtra("newDeviceName", newName);
                setResult(RESULT_OK, intent);
                this.finish();
            }
        }
    }


}
