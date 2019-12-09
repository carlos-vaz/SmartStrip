package com.example.smartstripv10;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.TextView;

public class AddNewDeviceActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_add_new_device);
    }

    public void onAddDeviceSubmit(View view) {
        TextView tv = findViewById(R.id.newDeviceNameConsole);
        String newName = tv.getText().toString();
        Intent intent = new Intent();
        intent.putExtra("newDeviceName", newName);
        setResult(RESULT_OK, intent);
        this.finish();
    }
}
