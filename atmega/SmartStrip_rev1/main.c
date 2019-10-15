/*
 * Copyright
 * SmartStrip Holdings Inc.
 *
 * Created: 10/9/2019 11:16:58:098922 AM, EASTERN STANDARD (NY)
 * Originally programmed in NickC++ (for Matlab)
 * Author : ngh9
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

/* Every (BAUDCOUNTER + 1 )clock cycles a baud is generated. 
 * (8 MHz / 16) / (51+1) = 9615 Baud, which is the closest
 * we can get to 9600*/
#define BAUDCOUNTER 51

typedef struct device {
	// EEPROM state - training variables
	float threshold;
	uint16_t minOn;		// Lowest ON example value
	uint16_t maxOff;
	uint16_t onCount;	// Num ON examples
	uint16_t offCount;
	uint8_t suff_ex;	// sufficient examples

	// EEPROM state - control variables
	uint8_t outlet;
	uint8_t algo_enable;
	uint8_t manual_enable;
	uint8_t manual_value;
} device_t;
			// 	 ________________________________________________________
typedef struct command { //	| STORE	|     TEACH 	|      MODE	|   OVERRIDE	| 
	char op;	//	|-------------------------------------------------------|
	uint8_t outlet;	//	|   X	|	X	|		| 		|	// X = normal field
	uint8_t threshold;//	|   X	|	X	|      (R)	|		|	// (R) = response field
	uint8_t mode;	//	|	|		| 	X	| 		|
	uint8_t mask;	//	|	|		| 		|	X	|
	uint8_t duration;//	|	|		| 		|	X	|
	uint8_t stop;	//	|	|		|      (R)	|		|
	uint8_t ok;	//	|  (R)	|      (R)	|		|      (R)	|
			// 	 -------------------------------------------------------
} command_t;

// Capture/control functions
int getSample(int);
void allOn();
void manualControl(int device, uint8_t en, uint8_t on);
void recordTeachIn(int device, int state);
void teachInToggle(int device);

// Bluetooth/Serial functions
void bluetoothConfig();
command_t parseCommand();
void int2str(uint16_t val, char* str);
void UART_Init(unsigned int ubrr);
void Serial_Send(char data[]);
void UART_TX(unsigned char data);
int Serial_Available();
unsigned char UART_RX(void);
int equals(char a[], const char b[]);
void clearBuf();


device_t Devices[6];	// Device EEPROM states 
char relayState;
char teachIn;	//Teach-in enable(1)/disable(0)

#define LOOP_COUNT_SHUTOFF 100

int all_on_iter = 0; // counts the loop iterations to determine when to shut off relays (after allOn())

#define RSSI_THRESH = -65;

int allOnPeriod;	// Set for duration of "All On" period

#define RX_BUFF_SIZE 20

volatile char btRxBuffer[RX_BUFF_SIZE];	 // Interrupt deposits BT data here for reading at our leisure
volatile int buff_i = 0;
volatile int new_msg_flag;

// Interrupt Service Routine for serial data ready
ISR(USART_RX_vect) {
	btRxBuffer[buff_i++] = UART_RX();
	if(buff_i == RX_BUFF_SIZE)
		buff_i = 0;

	new_msg_flag = 1;
}


int main(void)
{

	/*Load state from EEPROM*/
	cli();	// disable interrupts to avoid corruption
	sei();	// re-enable interrupts
	
	
	
	/*BLE Setup Stuff*/
	bluetoothConfig();

	/* LOOP */
	while (1) 
	{
		/*Check Bluetooth Message Buffer*/
		if(new_msg_flag) {
			_delay_ms(100); // allow some time in case RX buffer is still filling up

			command_t cmd = parseCommand(btRxBuffer);

			if(cmd.op == 0)
			{
				// STORE command
			}
			else if(cmd.op == 1)
			{
				// TEACH command
				
			}
			else if(cmd.op == 2)
			{
				// MODE command
				
			}
			else if(cmd.op == 3)
			{
				// OVERRIDE command
				
			}


			clearBuf();
			new_msg_flag = 0;
		}
		

		/*Get RSSI strength and set trigger if necessary*/
		if (allOnPeriod==0) {
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
void teachInToggle(int device) {
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
			if(sample > Devices[device].maxOff)
				Devices[device].maxOff = sample;
		}
	}

	/*ON sample*/
	else {
		/*If sample is GT all OFF samples, update threshold and onCount*/
		if (sample > Devices[device].maxOff) {
			Devices[device].onCount++;
			Devices[device].threshold += (float)(sample*Devices[device].onCount-1)/Devices[device].onCount;
			if(sample < Devices[device].minOn)
				Devices[device].minOn = sample;
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

int max(int array[]) {
	int length = sizeof(array);
	int max = array[0];
	int i;

	for (i=1; i<length; i++){
		if (array[i] > max) {
			max = array[i];
		}
	}
	return max;
}


/*Get filtered current sample from selected device*/
int getSample(int device){

	#define SAMPLES 256
	#define MAXPEAKS 10
	#define WINDOW 5
	#define FIND_PEAKS_HALF_WINDOW 2


	int rawData[SAMPLES];
	int peaks[MAXPEAKS];
	int peakCnt = 0;

	
	/*Sample device CS at 3kHz for 0.5s*/
	int i;
	for (i=0; i<SAMPLES; i++) {
		/*
		rawData[i] = ADCRead(device);
		*/

		/*Make all samples positive (512-1024)*/
		if (rawData[i]<512) {
			rawData[i] = 1024 - rawData[i];
		}

		/*Shift values to range 0-512*/
		rawData[i] = rawData[i] - 512;

		if (i >= WINDOW - 1) {
			// Overwrite raw data array with filtered (mean) data
			int k;
			float avg = 0;
			for(k=0; k<WINDOW; k++) {
				avg += (float)rawData[i-k] / WINDOW;
			}

			rawData[i-WINDOW+1] = (int) avg;
		}

		_delay_ms(5);

	}

	/*Find peaks in sample array - move through array, find elements that end a zeros (previously negative section), then find max. values between each zeros section*/
	for(i=FIND_PEAKS_HALF_WINDOW; i<SAMPLES-FIND_PEAKS_HALF_WINDOW; i++) {

		int k, max = 0;
		// Find max around window center (excluding center point)
		for(k=-1*FIND_PEAKS_HALF_WINDOW; k<=FIND_PEAKS_HALF_WINDOW; k++) {
			if(k != 0 && rawData[i+k] > max)
				max = rawData[i+k];
		}

		// check if center is strictly greater than surrounding window
		if(rawData[i] > max) {
			peaks[peakCnt++] = rawData[i];
		}

		if(peakCnt == MAXPEAKS)
			break;
	}

	float sample = 0;

	// Average all peaks
	for(i=0; i<peakCnt; i++) {
		sample += (float) peaks[i] / peakCnt;
	}

	return (int) sample;
}


/* SERIAL / BLUETOOTH FUNCTION DEFINITIONS*/

void bluetoothConfig()
{
	// Setup UART
	UART_Init(BAUDCOUNTER);

	_delay_ms(1000);

	Serial_Send("AT+ROLE0\r\n");		// Role = peripheral (responds)
	_delay_ms(100);	
	Serial_Send("AT+UUID0xFFE0\r\n");	// Set UUID for service (phone looks for this)
	_delay_ms(100);	
	Serial_Send("AT+CHAR0xFFE1\r\n");	// Set UUID for characteristic (phone looks for this)
	_delay_ms(100);	
	Serial_Send("AT+NAMESmartStrip\r\n");	// ame that shows up on phone scan
	_delay_ms(100);	

	new_msg_flag = 0;
}

command_t parseCommand()
{
	command_t ret;

	// DO SOME PARSING

	return ret;
}

void int2str(uint16_t val, char* str)
{
	int digit, i;
	for(i=0; i<5; i++) {
		digit = val / pow(10,(4-i));
		str[i] = digit + 48; // ascii offset
		val = val - (pow(10,(4-i)) * digit);
	}
}

void UART_Init(unsigned int ubrr)
{
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	// Enable receiver/transmitter
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	// Set frame format: 8 data + 1 stop bit
	UCSR0C = (3<<UCSZ00);
	//UCSR0C |= (1<<USBS0); // 2 stop bits

	// Enable RX Complete Interrupts
	UCSR0B |= (1<<RXCIE0);
	sei();
}

void Serial_Send(char data[])
{
	int i;
	for(i = 0; 1; i++) {
		if(data[i] == '\0')
			return;
		UART_TX(data[i]);
	}
}

void UART_TX(unsigned char data)
{
	// Wait until transmit buff is empty
	while( !(UCSR0A & (1<<UDRE0)));

	// Put data into buff
	UDR0 = data;
}

int Serial_Available()
{
	return ( (UCSR0A & (1<<RXC0)) == 1<<RXC0) ? 1 : 0;
}

unsigned char UART_RX(void)
{
	// Wait for data to be available
	while( !(UCSR0A & (1<<RXC0)));

	// return data in buff
	return UDR0;
}

int equals(char a[], const char b[]) {
	int i = 0;
	while(1) {
		if(a[i] != b[i] || i > 256) 
			return 0;
		if(a[i] == '\0')
			return 1;
		i++;
	}
}

void clearBuf()
{
	buff_i = 0;
	int i;
	for(i=0; i<RX_BUFF_SIZE; i++)
		btRxBuffer[i] = 0;
}

