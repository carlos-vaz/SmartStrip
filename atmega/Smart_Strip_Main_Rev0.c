/*
 * SmartStrip_Pseudocode_100919.c
 *
 * Created: 10/9/2019 11:16:58 AM
 * Author : ngh9
 */ 

#include <avr/io.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>

using namespace std;


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
			BLEConnect();
		}

		/*Otherwise, check BLE inbox*/
		else {
			/*If message received in BLE inbox, process message*/
			switch(BLEMessage) {

				/*Device teach-in toggle*/
				case 1 {
					
					/*Send device number to teachInToggle to toggle teach-in mode of selected device*/
					teachInToggle(device)
				}
				
				/*Device record teach-in sample*/
				case 2 {
					
					/*Send device and state (off/on sample) to recordTeachIn – returns updated on/off sample counts, 
					status of device (enough samples for auto control), sample status (good/bad), and updated device threshold*/
					[oncount(device), offcount(device), status(device), sampStatus, threshold(device)] = recordTeachIn(device, state)
				}

				/*Device control enable/disable*/
				case 3 {

					/*Send device and to controlEnable to toggle algorithm/manual control of outlet*/
					controlEnable(device);
				}

				/*Device manual off/on*/
				case 4 {

					/*Send device to manualControl to toggle outlet relays*/
					manualControl(device, state);
				}
			}
		}

		/*Get RSSI strength and set trigger if necessary*/
			/*Update last with now*/
			rssiLast = rssiNow;
		
			/*Get new RSSI Strength*/
			rssiNow = getRSSI();
		
			/*Set trigger on rising edge over threshold*/
			if (rssiNow >= rssiThresh) && (rssiLast < rssiThresh) {
				rssiTrig = 1;
			}

		/*If all devices off, check BLE RSSI value for trigger*/
		if (relayState == [0, 0, 0, 0, 0, 0] && rssiTrig) {
			
			/*Close all relays in AUTO, pause, check device currents, and open off devices in AUTO*/
			allOn();

			/*Reset RSSI Trigger*/
			rssiTrig = 0;
			
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

	for i = 1:1:6 {
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

	for i=1:1:6 {
		if (control(i)==1) {
			relayState(i) = 1;
		}
	}
	
	/*Move relayState to I/O pins*/

	sleep(pauseDelay);

	for i=1:1:6 {
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
	
	int rawData[1500] = 0;
	int filtData[1500] = 0;
	int peaks[25] = 0;
	int peakCount = 1;

	/*Filtering sliding window width*/
	int filtFactor = 5;
	
	/*Sample device CS at 3kHz for 0.5s*/
	for i=1:1:1500 {
		
		rawData(i) = ADCRead(device);
		
	
		
		sleep(5);
		
	}

	/*Remove negative currents (LT 512), normalize all currents to 0-512, and filter current data using sliding window of length filtFactor*/
	for i = filtFactor:1:1500 {

		if (rawData(i)<512) {
			rawData(i) = 512;
		}
		rawData(i) = rawData(i) - 512;

		filtData(i) = mean(rawData((i-filtFactor):i));

	}

	/*Find peaks in sample array - move through array, find elements that end a zeros (previously negative section), then find max. values between each zeros section*/

	


	/*Average peaks to get sample*/
	

	return sample;

}
