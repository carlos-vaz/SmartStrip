/*
 * SmartStrip_Pseudocode_100919.c
 *
 * Created: 10/9/2019 11:16:58 AM
 * Author : ngh9
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
//#include <unistd.h>
//#include <stdlib.h>


int getSample(int);
void allOn();
void manualControl(int device, uint8_t en, uint8_t on);
void recordTeachIn(int device, int state);
int teachInToggle(int device);

typedef struct {
	// EEPROM state - training variables
	float threshold;
	uint16_t minOn;		// Lowest ON example value
	uint16_t maxOff;
	uint16_t onCount;	// Num ON examples
	uint16_t offCount;
	uint16_t suff_ex;	// sufficient examples

	// EEPROM state - control variables
	uint8_t outlet;
	uint8_t algo_enable;
	uint8_t manual_enable;
	uint8_t manual_value;
} device_t;

device_t Devices[6];			// Device EEPROM states 
	
char relayState;

char teachIn;	//Teach-in enable(1)/disable(0)

#define LOOP_COUNT_SHUTOFF 100

int all_on_iter = 0; // counts the loop iterations to determine when to shut off relays (after allOn())

int BLEConnection = 0;

#define RSSI_THRESH = -65;

int allOnPeriod;	// Set for duration of "All On" period

int main(void)
{

	/*Load state from EEPROM*/
	
	/*BLE Setup Stuff*/
	BLEConnection = 1;
	

	/* LOOP */
	while (1) 
	{
		/*Check Bluetooth Message Buffer*/
		

		/*Get RSSI strength and set trigger if necessary*/
		if (BLEConnection==1 && allOnPeriod==0) {
			// Send AT+RSSI to BT module and read response
			// On reading a close RSSI, set allOnPeriod = 1
		}


		/*If RSSI causes a trigger, activate all relays temporarily*/
		if (allOnPeriod) {
			if(all_on_iter == 0) {
			// Start of "All On" period
				allOn();
			}

			if(all_on_iter == LOOP_COUNT_SHUTOFF) {
			// End of "All On" period
				all_on_iter = 0;
				allOnPeriod = 0;
			}

			//TODO: When "All On" period is almost over, start measuring
			// currents and prepare relay values, but do not set relays yet...

			all_on_iter++;
		}

		/*Otherwise, for each on device in AUTO, get current sample, compare to threshold, and open relay if necessary*/
		else {
			
			char update_mask = 0;
			int i;
			for(i=0; i<6; i++) {
				if (relayState & (1<<i)) {
					uint16_t sample = getSample(i);
					if (sample < Devices[i].threshold && Devices[i].algo_enable==1) {
						update_mask |= (1<<i);
					}
				}
			}
			relayState &= ~update_mask;
		}

		/*Update relay and LED GPIO states based on relayState*/
			
		/*ledState = relayState;
		PORTX = relayState &= 0x3f;
		PORTY = ledState & 0x3f;*/
	}
}


/* FUNCTIONS */


/*Toggle teach-in mode of selected outlet*/
int teachInToggle(int device) {
	teachIn ^= (1<<device);
}

/*Take sample, classify, refresh threshold, and update counts*/
void recordTeachIn(int device, int state) {
	
	int sample = getSample(device);
	
	/*OFF sample*/
	if (state == 0) {
		/*If sample is LT all ON samples, update threshold and offCount*/
		if (sample < Devices[device].minOn) {
			Devices[device].offCount++;
			Devices[device].threshold += (float)(sample*Devices[device].offCount-1)/Devices[device].offCount;
		}
	}

	/*ON sample*/
	else {
		/*If sample is GT all OFF samples, update threshold and onCount*/
		if (sample > Devices[device].maxOff) {
			Devices[device].onCount++;
			Devices[device].threshold += (float)(sample*Devices[device].onCount-1)/Devices[device].onCount;
		}
	}


	/*Check device status - if device has at least 3 off and 3 on samples, status = 1*/
	if (Devices[device].onCount >= 3 && Devices[device].offCount >= 3) {
		Devices[device].suff_ex = 1;
	}
	else {
		Devices[device].suff_ex = 0;
	}
}

/*Manual control of devices*/
void manualControl(int device, uint8_t en, uint8_t on) {
	Devices[device].manual_enable = en;
	Devices[device].manual_value = on;
}

/*Function to close all relays in AUTO*/
void allOn() {
	int i;
	char relayMask = 0;
	for(i=0; i<6; i++) {
		relayMask |= (Devices[i].manual_enable << i);
	}
	relayState |= ~relayMask;
}


/*Get filtered current sample from selected device*/
int getSample(int device){
	
	int rawData[256];
	int peaks[10];
	int peakCount = 1;
	int sumStore = 0;
	
	//Counter variables
	int i;
	int j;

	/*Filtering sliding window width*/
	int filtFactor = 5;
	
	/*Sample device CS at 3kHz for 0.5s*/
	for (i=0; i<256; i++) {
		
	
		/*
		rawData[i] = ADCRead(device);
		*/

		/*Make all samples positive (512-1024)*/
		if (rawData[i]<512) {
			rawData[i] = rawData[i]+(2*(512-rawData[i]));
		}

		/*Normalize to 0-512*/
		rawData[i] = rawData[i] - 512;

		if (i>=(filtFactor-1)) {
			
			sumStore = 0;
			
			// Overwrite raw data array with filtered (mean) data
			// rawData(i-filtFactor) = mean(rawData((i-filtFactor):i))
			for (j=(i-filtFactor); j<=i; j++) {
				
				sumStore = sumStore + rawData[j];
				
			}
			
			rawData[i-filtFactor] = sumStore/filtFactor;	
	
		}
	
		_delay_ms(5);
		
	}



	/*Find peaks in sample array - move through array, find elements that end a zeros (previously negative section), then find max. values between each zeros section*/


	/*Average peaks to get sample*/
	
	return 0; // TODO: return sample

}

