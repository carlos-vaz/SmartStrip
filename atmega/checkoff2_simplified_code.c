/*
 * Copyright
 * SmartStrip Holdings Inc.
 *
 * Created: 10/9/2019 11:16:58:098922 AM, EASTERN STANDARD (NY)
 * Authors : ngh9, cdv16
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000

#include <util/delay.h>
#include <math.h>

/* Every (BAUDCOUNTER + 1 )clock cycles a baud is generated. 
 * (8 MHz / 16) / (51+1) = 9615 Baud, which is the closest
 * we can get to 9600*/
#define BAUDCOUNTER 51

typedef struct thresh {
	uint16_t val;	
	uint16_t offval;
	uint16_t onval;
	uint16_t onCount;
	uint16_t offCount;
} thresh_t;


typedef struct device {
	// EEPROM state - training variables
	thresh_t threshold;
	uint16_t minOn;// Lowest ON example value
	uint16_t maxOff;
	uint8_t suff_ex;	// sufficient examples

	// EEPROM state - control variables
	uint8_t outlet;
	uint8_t algo_enable;
	uint8_t manual_enable;
	uint8_t manual_value;
} device_t;
	
	
							// 	 ________________________________________________________
typedef struct command { 	//	| STORE	|     TEACH 	|      MODE	|   OVERRIDE	|
	char op;				//	|-------------------------------------------------------|
	unsigned char outlet;	//	|   X	|	X	|		| 		|	// X = normal field
	thresh_t threshold;		//	|   X	|	X	|      (R)	|		|	// (R) = response field
	unsigned char mode;		//	|	|		| 	X	| 		|
	unsigned char mask;		//	|	|		| 		|	X	|
	unsigned char duration;	//	|	|		| 		|	X	|
	unsigned char stop;		//	|	|		|      (R)	|		|
	unsigned char ok;		//	|  (R)	|      (R)	|		|      (R)	|
	// 	 -------------------------------------------------------
} command_t;

// Capture/control functions
int getSample(int);
void allOn();
void manualControl(int device, uint8_t en, uint8_t on);
int recordTeachIn(int device, int state);
void teachInToggle(int device);
void updateGPIO();

// ADC functions
void adcConfig();
uint16_t sampleADC();


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
char ledState;
char teachIn;	//Teach-in enable(1)/disable(0)

#define FLASH_COUNT 500

int flash_iter = 0; //Counts the loop iterations to determine when to toggle flashing LEDs

#define RSSI_THRESH = -65;

int allOnPeriod;	// Set for duration of "All On" period

#define RX_BUFF_SIZE 20

volatile char btRxBuffer[RX_BUFF_SIZE];	 // Interrupt deposits BT data here for reading at our leisure
volatile int buff_i = 0;
volatile int new_msg_flag;

// Interrupt Service Routine for serial data ready
ISR(USART0_RX_vect) {
	btRxBuffer[buff_i++] = UART_RX();
	if(buff_i == RX_BUFF_SIZE)
		buff_i = 0;

	new_msg_flag = 1;
}


int main(void)
{	
	
	/*GPIO Setup Stuff*/	
		// Define Port C I/O Pins as Outputs for LEDs and Set to 0
		DDRC |= 0b11111111;
		PORTC = 0b00000000;
	
	
	/*ADC Setup Stuff*/
	adcConfig();

	
	/*Serial Setup Stuff*/
	UART_Init(BAUDCOUNTER);
	new_msg_flag = 0;

	//Define device max. off = 0, min. on = 1024
	int i;
	for (i=0; i<6; i++){
		Devices[i].maxOff = 0;
		Devices[i].minOn = 1024;
	}

	////////////////////////////////////////////// LOOP //////////////////////////////////////////////
	while (1) 
	{
		
		/*Check Bluetooth Message Buffer*/
		if(new_msg_flag) {
			_delay_ms(100); // allow some time in case RX buffer is still filling up

			//Parse Serial buffer contents into cmd struct
			command_t cmd = parseCommand(btRxBuffer);

			clearBuf();
			new_msg_flag = 0;

			//Check for command operation
			if(cmd.op == 't')
			{
				// TEACH command - toggle device teach-in mode on/off
				teachInToggle((int) cmd.outlet);
				
				//Return status
				if(teachIn & (1<<cmd.outlet)) {
					Serial_Send("Teach-In On!\n\n");
				}
				else {
					Serial_Send("Teach-In Off!\n\n");
				}
				
				
			}
			
			else if(cmd.op == 'm')
			{
				// MODE command - get device teach-in sample, indicating on or off
				//Check if outlet is in teach-in mode
				if(teachIn) {
					int outlet = (int) log(teachIn)/log(2);
					int sample = 0;
					//Get teach in - send device/outlet and mode (0 = Off, 1 = On)
					sample = recordTeachIn(outlet, (int) cmd.mode);
					
					//OK
					Serial_Send("Teach-In Recorded: mode = ");
					UART_TX(cmd.mode + 48);
					Serial_Send("\n");
					
					//Send updated device info.
					char str[6];
					int2str(sample, str);
					Serial_Send("Sample: ");
					Serial_Send(str);
					int2str(Devices[outlet].threshold.val, str);
					Serial_Send(", Threshold Value: ");
					Serial_Send(str);
					int2str(Devices[outlet].threshold.onCount, str);
					Serial_Send(", onCount = ");
					Serial_Send(str);
					int2str(Devices[outlet].threshold.offCount, str);
					Serial_Send(", offCount = ");
					Serial_Send(str);
					Serial_Send("\n\n\n");

				}
				
			}		
		}
		

		//Once algorithm control enabled, toggle LED based on sample GT/LT threshold
		else {

			//Get a sample from first ADC channel
			uint16_t sample = getSample(0);
					
			//If device is on and control enabled, LED on
			if (sample >= Devices[0].threshold.val && Devices[0].suff_ex) {
				//LED On
				PORTC = 0b00000001;
			}
			
			//If device is off and control enabled, LED off
			if (sample < Devices[0].threshold.val && Devices[0].suff_ex) {
				//LED Off
				PORTC = 0b00000000;
			}
	
			//Show every 50 samples
			if(flash_iter>50 && !teachIn){
				char str[6];
				int2str(sample, str);
				Serial_Send("Sample: ");
				Serial_Send(str);
				Serial_Send("\n");
				
				flash_iter = 0;
			}
	
			
		}

		
		flash_iter++;
	}
	
}


////////////////////////////////////////////// FUNCTIONS //////////////////////////////////////////////


/*Toggle teach-in mode of selected outlet*/
void teachInToggle(int device) {
	teachIn ^= (1<<device);
}

/*Take sample, classify, refresh threshold, and update counts*/
int recordTeachIn(int device, int state) {
	
	int sample = getSample(device);
	
	/*OFF sample*/
	if (state == 0) {
		//If sample is LT all ON samples, update average off current
		if (sample < Devices[device].minOn) {
			Devices[device].threshold.offCount++;
			Devices[device].threshold.offval = (uint16_t)((Devices[device].threshold.offval*((float)(Devices[device].threshold.offCount-1)/Devices[device].threshold.offCount))+((float)sample/Devices[device].threshold.offCount));
			
			if(sample > Devices[device].maxOff) {
				Devices[device].maxOff = sample;
			}
			
		}
	}

	/*ON sample*/
	else {
		//If sample is GT all OFF samples, update average on current
		if (sample > Devices[device].maxOff) {
			Devices[device].threshold.onCount++;
			Devices[device].threshold.onval = (uint16_t)((Devices[device].threshold.onval*((float)(Devices[device].threshold.onCount-1)/Devices[device].threshold.onCount))+((float)sample/Devices[device].threshold.onCount));
			
			if(sample < Devices[device].minOn) {
				Devices[device].minOn = sample;
			}
			
		}
	}
	
	//Update current off/on threshold
	Devices[device].threshold.val = (Devices[device].threshold.offval + Devices[device].threshold.onval)/2;


	/*Check device status - if device has at least 3 off and 3 on samples, status = 1*/
	if (Devices[device].threshold.onCount >= 3 && Devices[device].threshold.offCount >= 3) {
		Devices[device].suff_ex = 1;
	}
	else {
		Devices[device].suff_ex = 0;
	}
	
	//Send back sample
	return sample;
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

void adcConfig() {
	DDRA= 0x00; //Set port A to inputs (from ADC)  
	
	//Initialize A/D Conversion
	ADCSRA = 1<<ADEN | 1<<ADPS2|0<<ADPS1|1<<ADPS0; // 0x87 // 0b10000111 // Set  ADC Enable bit (7) in ADCSRA register, and set ADC prescaler to 128
	ADMUX=0<<REFS1|1<<REFS0|0<<ADLAR; //0x60; // 0b01100000  // select Analog  Reference voltage to be AVcc and use right justify
}

// Must be called from within getSample(), which selects the channel before calling this!
uint16_t sampleADC() {
	uint8_t  Current1;
	uint8_t  Current2;
	uint16_t  Current;

	ADCSRA|=(1<<ADSC); // Start conversion 
	while ((ADCSRA & (1<<ADIF))==0); // wait for completion
	Current2=ADCL;
	Current1=ADCH;
	Current= (Current1<<8)|Current2;
	Current= Current & 0x3FF;

	return Current;
}


/*Get filtered current sample from selected device*/
int getSample(int device) {

	#define SAMPLES 256
	#define MAXPEAKS 10
	#define WINDOW 5
	#define FIND_PEAKS_HALF_WINDOW 2

	int peakCnt = 0;

	uint16_t rawData[SAMPLES];
	uint16_t peaks[MAXPEAKS];

	// Select ADC Channel based on device
	switch(device)
	{
		//Set MUX selectors to choose ADC to sample from
		case 0: 0<<MUX3|0<<MUX2|0<<MUX1|0<<MUX0;// 0b01100000
		case 1: 0<<MUX3|0<<MUX2|0<<MUX1|1<<MUX0;// 0b01100001
		case 2: 0<<MUX3|0<<MUX2|1<<MUX1|0<<MUX0;// 0b01100010
		case 3: 0<<MUX3|1<<MUX2|0<<MUX1|0<<MUX0;// 0b01100100
		case 4: 0<<MUX3|1<<MUX2|0<<MUX1|1<<MUX0;// 0b01100101
		case 5: 0<<MUX3|1<<MUX2|1<<MUX1|0<<MUX0;// 0b01100110
	}


	//Sample device CS at 3kHz for 256 samples
	int i;
	for (i=0; i<SAMPLES; i++) {

		//Get sample from ADC
		rawData[i] = sampleADC();


		//Make all samples positive (512-1024)
		if (rawData[i]<512) {
			rawData[i] = 1024 - rawData[i];
		}

		//Shift values to range 0-512
		rawData[i] = rawData[i] - 512;

		if (i >= WINDOW - 1) {

			// Overwrite raw data array with filtered (mean) data
			int k;
			float avg = 0;
			
			//Sum
			for(k=0; k<WINDOW; k++) {
				avg += (float)rawData[i-k];
			}
			
			//Divide
			avg = avg/WINDOW;
		
			//Overwrite raw data array with average
			rawData[i-WINDOW+1] = (int) avg;
		}


		//Delay - See window vs. delay table
		_delay_us(15);	
	}


	//Find peaks in sample array - move through array, find elements that end a zeros (previously negative section), then find max. values between each zeros section
	for(i=FIND_PEAKS_HALF_WINDOW-1; i<SAMPLES-FIND_PEAKS_HALF_WINDOW-WINDOW; i++) {

		int k, max = 0;
		
		// Find max around window center
		for(k=-1*FIND_PEAKS_HALF_WINDOW; k<=FIND_PEAKS_HALF_WINDOW; k++) {
			if(rawData[i+k] > max) {
				max = rawData[i+k];
			}
			
		}

		// Check if point = max
		if(rawData[i] == max) {
			peaks[peakCnt++] = rawData[i];
		}

		if(peakCnt == MAXPEAKS) {
			break;
		}
		
	}

	float sample = 0;

	// Average all peaks
	for(i=0; i<peakCnt; i++) {
		sample += (float) peaks[i];
	}
	
	sample = sample/peakCnt;

	return (int) sample;
}


/* SERIAL / BLUETOOTH FUNCTION DEFINITIONS*/
command_t parseCommand()
{
	command_t ret;
	thresh_t threshold;

	// DO SOME PARSING
	switch (btRxBuffer[0])
	{
		case 's':
			ret.op = 's';
			ret.outlet = (unsigned char)btRxBuffer[1];
			ret.threshold.val = (uint16_t)btRxBuffer[2];
			ret.threshold.onval = (uint16_t)btRxBuffer[4];
			ret.threshold.offval = (uint16_t)btRxBuffer[6];
			ret.threshold.onCount = (uint16_t)btRxBuffer[8];
			ret.threshold.offCount = (uint16_t)btRxBuffer[10];
			break;
		case 't':
			ret.op = 't';
			ret.outlet = (unsigned char)btRxBuffer[1];
			ret.threshold.val = (uint16_t)btRxBuffer[2];
			ret.threshold.onval = (uint16_t)btRxBuffer[4];
			ret.threshold.offval = (uint16_t)btRxBuffer[6];
			ret.threshold.onCount = (uint16_t)btRxBuffer[8];
			ret.threshold.offCount = (uint16_t)btRxBuffer[10];
			break;
		case 'm':
			ret.op = 'm';
			ret.mode = (unsigned char)btRxBuffer[1];
			break;

		case 'o':
			ret.op = 'o';
			ret.mask = (unsigned char)btRxBuffer[1];
			ret.duration = (unsigned char)btRxBuffer[2];
			break;
	}

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

