## Signal Processing Test
### `main.c` 
Implements a modified `getSample()` function that fills four arrays:
+ `rawData[256]` - unaltered values read from ADC
+ `filteredData[256]` - smoothed with a sliding average filter
+ `extendedPeaks[256]` - holds the value of the last peak found
+ `final[256]` - a single value (average of all peaks) populates all elements
After sampling, the serial terminal will be filled with the array values in CSV format:  
```
rawData[0],filteredData[0],extendedPeaks[0],final[0]  
rawData[1],filteredData[1],extendedPeaks[1],final[1]  
...
```

### `waves.py` 
plots the four curves described above: raw (green), filtered (red), peaks (blue), final (yellow)  

![alt text][plot]

[plot]: https://github.com/fullprocess/SmartStrip/blob/master/atmega/SignalProcessingTests/plot.png

## Instructions
### 1. Flash the Arduino
Flash [simple\_serial\_monitor.ino](https://github.com/fullprocess/SmartStrip/blob/master/arduino/SerialMonitors/simple_serial_monitor.ino) to the Arduino to monitor the 
TX output of the ATmega644. 

### 2. Flash the ATmega
Compile and flash to the ATmega644. Then, connect power to the Arduino and Atmega at the same time, and open a serial terminal to the Arduino (e.g. from Arduino Studio)

### 3. Plot the waveforms
Copy the CSV-style output from the serial terminal, paste into a file named `data.txt` in the same directory as `waves.py`, and run `python waves.py`. 

