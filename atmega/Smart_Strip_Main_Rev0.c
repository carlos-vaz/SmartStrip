/*
 * SmartStrip_Pseudocode_100919.c
 *
 * Created: 10/9/2019 11:16:58 AM
 * Author : ngh9
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>


/* SETUP - All defined variables are global */
	
	/*Define variables for outlet/device status*/
		/*%Relay status*/
		bool relayState[6];

		/*Teach-in enable(1)/disable(0)*/
		bool teachIn[6];

		/*Algorithm control enable(1)/disable(0)*/
		bool control[6];

	/*Define variables for device current data*/
		/*Number of "off" samples saved*/
		int offcount[6];

		/*Number of "on" samples saved*/
		int oncount[6];

		/*Device status indicates whether enough on/off samples have been recorded to allow device algorithm control*/
		int status[6];

		/*Define 3-dimensional array for current data: device(6) x state(2) x sample(100)*/
		/*6 devices, each device has on and off samples, each device can have up to 100 (could change this) on and off samples*/
		int currentdata[6][2][100];

		/*On/off current thresholds*/
		int threshold[6];
			
	/*Generic counter, tracking variables that will be overwritten*/
	bool sampStatus;
		
	int i;
	int device;
	int state;
	int sample;
	int all_on_iter = 0; // counts the loop iterations to determine when to shut off relays (after allOn())
	#define LOOP_COUNT_SHUTOFF 100
	

	/*Run startup diagnostics*/
	bool statusSensors = startupCheckSensors();
		
	/*Variables for BLE Status*/
		/*BLE Connected*/
		bool BLEConnection;

		/*BLE RSSI Strength (if connected) - 2 variables to detect rising edge*/
		int rssiLast;
		int rssiNow;		
		
		/*Threshold for RSSI strength trigger*/
		int rssiThresh = 50;
			
		/*Trigger to close relays on rising edged - latched by RSSI checker, unlatched by relay control*/
		bool rssiTrig;

int main(void)
{	
	
	/*BLE Setup Stuff*/
	

	/* LOOP */
	while (1) 
	{
		
		
		/*If no BLE connection, poll for BLE connection*/
		if (BLEConnection == 0) {
			// Try to connect
		}

		/*Check Bluetooth Message Buffer*/
		

		/*Get RSSI strength and set trigger if necessary*/
		if (BLEConnection==0 && rssiTrig==0) {
			// Send AT+RSSI to BT module and read response
		}


		/*If RSSI causes a trigger, activate all relays temporarily*/
		if (rssiTrig) {
			if(all_on_iter == 0) {
			// Start of "All On" period
				allOn();
			}
			
			if(all_on_iter == LOOP_COUNT_SHUTOFF) {
			// End of "All On" period
				all_on_iter = 0;
				rssiTrig = 0;
				allResume();
			}

			//TODO: When "All On" period is almost over, start measuring
			// currents and prepare relay values, but do not set relays yet...

			all_on_iter++;
		}

		/*Otherwise, for each on device in AUTO, get current sample, compare to threshold, and open relay if necessary*/
		else {
			for i=1:1:6 {
				if (relayState(i) == 1) {
					sample = getSample(i);
					if ((sample < threshold(i)) && (control(i)==1)) {
						relayState(i) = 0;
					}
				}
			}
			
		}

		/*Update relay and LED GPIO states based on relayState*/
		ledState = relayState;
		PORTX(1:6) = relayState;
		PORTY(1:6) = ledState;	
	}

}


/* FUNCTIONS */


/*Check sensors at microcontroller startup*/
int startupCheckSensors() {
	
	bool OK = 1;

	for (int i=0; i<6; i++) {
		if (ADCInput(i) <2.4 || ADCInput(i) > 2.6) {
			OK = 0;
			break;
		}
	}
	return OK;
}



/*Toggle teach-in mode of selected outlet*/
int teachInToggle(device) {

	teachIn(device) = ~teachIn(device);

}



/*Take sample, classify, refresh threshold, and update counts*/
int recordTeachIn(device, state) {
	
	sample = getSample(device);
	
	/*Off sample*/
	if (state == 0) {
		/*Check sample is LT all on samples*/
		sampStatus = 1;

		for i = 1:1:oncount(device) {
			if (sample >= currentdata(device, 0, i)) {
				sampStatus = 0;
				break
			}
		}

		/*If sample good, add to device's off sample library and update count*/
		if (sampStatus == 1){
			currentdata(device, 0, offcount(device)+1) = sample;
			offcount(device)++;
		}
	}

	/*On sample*/
	else {
		/*Check sample is GT all off samples*/
		sampStatus = 1;

		for i = 1:1:offcount(device) {
			if (sample <= currentdata(device, 1, i)) {
				sampStatus = 0;
				break
			}
		}

		/*If sample good, add to device's on sample library and update count*/
		if (sampStatus == 1){
			currentdata(device, 1, oncount(device)+1) = sample;
			oncount(device)++;
		}
	}


	/*Check device status - if device has at least 3 off and 3 on samples, status = 1*/
	if (status(device)==0 && sampStatus==1){
		if ((offcount(device)>=3) && (oncount(device)>=3)) {
			status(device) = 1;
		}
	}

	/*Update threshold*/
	if (status(device)==1 && sampStatus==1) {
		threshold(device) = avg([avg(currentdata(device, 1, (0:oncount(device))), mean(currentdata(device, 0, (0:offcount(device)))]);
	}

	return oncount(device), offcount(device), status(device), sampStatus, threshold(device)

}



/*Enable/disable algorithm outlet control*/
int controlEnable(device) {

	control(device) = ~control(device);

}



/*Manual control of devices*/
int manualControl(device) {

	relayState(device) = ~relayState(device);

}


/*Function to close all relays in AUTO, pause, check device currents, and open off devices in AUTO*/
int allOn() {
	
	/*Pause delay for all relays closed (s)*/
	int pauseDelay = 30;

	int i;
	for(i=0; i<6; i++) {
		if (control(i)==1) {
			relayState(i) = 1;
		}
	}
	
	/*Move relayState to I/O pins*/

	sleep(pauseDelay);

	for(i=0; i<6; i++) {
		if (control(i)==1) {
			sample = getSample(i);
			if (sample < threshold(i)) {
				relayState(i) = 0;
			}
		}
	}
	
	/*Move relayState to I/O pins*/
	

}



/*Get filtered current sample from selected device*/
int getSample(device){
	
	int rawData[256] = 0;
	int peaks[10] = 0;
	int peakCount = 1;

	/*Filtering sliding window width*/
	int filtFactor = 5;
	
	/*Sample device CS at 3kHz for 0.5s*/
	for i=1:1:256 {
		
		rawData(i) = ADCRead(device);
		
		/*Make all samples positive (512-1024)*/
		if (rawData(i)<512) {
			rawData(i) = rawData(i)+(2*(512-rawData(i)));
		}
		
		/*Normalize to 0-512*/
		rawData(i) = rawData(i) - 512;
		
		if (i>=(filtFactor-1)) {
			
			/*Overwrite raw data array with filtered data */
			rawData(i-filtFactor) = mean(rawData((i-filtFactor):i));
		}
		
		sleep(x);
		
	}



	/*Find peaks in sample array - move through array, find elements that end a zeros (previously negative section), then find max. values between each zeros section*/


	/*Average peaks to get sample*/
	

	return sample;

}

