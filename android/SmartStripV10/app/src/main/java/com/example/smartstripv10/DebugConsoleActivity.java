package com.example.smartstripv10;

import androidx.appcompat.app.AppCompatActivity;

import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Handler;
import android.text.method.ScrollingMovementMethod;
import android.view.View;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.TextView;

public class DebugConsoleActivity extends AppCompatActivity {

    Handler handler;

    Drawable defaultBottomButton;

    int lastTextSize = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_debug_console);

        defaultBottomButton =  findViewById(R.id.bottomButton).getBackground();

        handler = new Handler();
        final int delay = 1000; //milliseconds

        handler.postDelayed(new Runnable(){
            public void run(){
                TextView console = findViewById(R.id.debugConsoleTextView);
                console.setText(MainActivity.debug.text);

                int totalHeight = console.getLayout().getLineTop(console.getLineCount()) - console.getHeight();
                if(console.getScrollY() < totalHeight && MainActivity.debug.text.length() > lastTextSize) {
                    Button b = findViewById(R.id.bottomButton);
                    b.setBackgroundColor(Color.GREEN);
                }

                lastTextSize = MainActivity.debug.text.length();

                handler.postDelayed(this, delay);
            }
        }, delay);

        TextView console = findViewById(R.id.debugConsoleTextView);
        console.setMovementMethod(new ScrollingMovementMethod());
    }

    public void onClearButton(View view) {
        MainActivity.debug.clearText();
        TextView console = findViewById(R.id.debugConsoleTextView);
        console.setText(MainActivity.debug.text);
    }

    public void onBackButton(View view) {
        this.finish();
    }

    public void onBottomButtom(View view) {
        ScrollView scrollView = findViewById(R.id.scrollView);
        scrollView.fullScroll(View.FOCUS_DOWN);

        Button b = findViewById(R.id.bottomButton);
        b.setBackground(defaultBottomButton);
    }


}
