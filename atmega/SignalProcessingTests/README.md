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
plots the four curves described above: 
+ raw: GREEN
+ filtered: RED
+ peaks: BLUE
+ final: YELLOW

