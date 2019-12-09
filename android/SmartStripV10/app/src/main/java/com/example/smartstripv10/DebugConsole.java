package com.example.smartstripv10;

public class DebugConsole {
    public StringBuilder text;

    public DebugConsole() {
        text = new StringBuilder("NEW DEBUG CONSOLE\n");
    }

    public void putText(String input) {
        text.append(input);
        text.append("\n\n");
    }

    public void clearText() {
        text = new StringBuilder("NEW DEBUG CONSOLE\n");
    }
}
