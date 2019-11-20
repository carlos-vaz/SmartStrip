/* MIT License
 * Copyright (c) 2019 SmartStrip Holdings, Inc.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. */

//LAST MODIFIED: 11/13/19 4:49PM

#include <avr/io.h>
#include <avr/interrupt.h>

//Define CPU f = 8MHz for util/delay.h for accurate timing
#define F_CPU 8000000
#include <util/delay.h>

#include <math.h>

/* Every (BAUDCOUNTER + 1 )clock cycles a baud is generated. 
 * (8 MHz / 16) / (51+1) = 9615 Baud, which is the closest
 * we can get to 9600*/
#define BAUDCOUNTER 51


//////////////////////////////////////////////////////////////// STRUCT DECLARATIONS ////////////////////////////////////////////////////////////////

// THRESHOLD struct - used once by each device to store threshold data
typedef volatile struct thresh {
	uint16_t val;		// On/off threshold value	
	uint16_t onval;		// Average on current		
	uint16_t offval;	// Average off current
	uint16_t onCount;	// Number of on samples taken
	uint16_t offCount;	// Number of off samples taken
} thresh_t;

// DEVICE struct - used once by each device to store threshold, minimum on sample, maximum off sample
typedef volatile struct device {
	// EEPROM state - training variables
	thresh_t threshold;	// thresh struct (see above)
	uint16_t minOn;		// Lowest ON sample value
	uint16_t maxOff;	// Greatest off sample value
	uint8_t suff_ex;	// sufficient examples

	// EEPROM state - control variables
	uint8_t outlet;			// Defines outlet number (0 - 5)
	uint8_t algo_enable;	// Algorithm control enabled/disabled by user
	uint8_t manual_enable;	// ???
	uint8_t manual_value;	// ???
} device_t;
	
// COMMAND struct - used for parsing Bluetooth messages (up to 20B). Messages are parsed based on type (Byte 0) according to the
//					table below - see parseCommand()
							// 	 _______________________________________
typedef struct command { 	//	| STORE	|   TEACH	| MODE	| OVERRIDE	| 
	char op;				//	|---------------------------------------|
	unsigned char outlet;	//	|   X	|	  X		|		| 			|	// X = normal field
	thresh_t threshold;		//	|   X	|	  X		|  (R)	|			|	// (R) = response field
	unsigned char mode;		//	|		|	  X		| 	X	| 			|
	unsigned char mask;		//	|		|			| 		|	  X		|
	unsigned char duration;	//	|		|			| 		|	  X		|
	unsigned char stop;		//	|		|			|  (R)	|			|
	unsigned char ok;		//	|  (R)	|    (R)	|		|    (R)	|
							// 	 ---------------------------------------
} command_t;


//////////////////////////////////////////////////////////////// FUNCTION DECLARATIONS ////////////////////////////////////////////////////////////////

// Bluetooth/Serial functions
void bluetoothConfig();						// Configure AT-09 Bluetooth module at start-up
command_t parseCommand();					// Parse Bluetooth messages from 20B block to struct
void int2str(uint16_t val, char* str);		// Convert integer to string (used for serial monitor)
void UART_Init(unsigned int ubrr);			// Initialize UART comm w/ AT-09 Bluetooth module
void Serial_Send(char data[]);				// Send string over UART
void UART_TX(unsigned char data);			// Send single char over UART (used by Serial_Send)
unsigned char UART_RX(void);				// Receive single char over UART (used to fill btRxBuffer)
void clearBuf();							// Clear btRxBuffer once BT message has been parsed to struct
void Data_Send(char data[], int length);	// Special Serial_Send for sending device data after teach-in

// Capture/control functions
int getSample(int);											// Set ADC MUX, sample, LPF, find peaks, and average for given outlet. Return one sample value.											
void manualControl(int device, uint8_t en, uint8_t on);		// ???
void recordTeachIn(int device, int state);					// Record one teach-in sample for the given device at given state. Updates device threshold struct.
void teachInToggle(int device, unsigned char mode);			// Set given device in or out of teach-in mode (given mode)
void updateGPIO();											// Move relayState and ledState chars to PORTC and PORTD GPIO

// ADC functions
void adcConfig();		// Configure ADC - enable, set pre-scaler
uint16_t sampleADC();	// Get sample from ADC - read registers and concatenate to 10-bit (ADC MUX set by getSample)

//////////////////////////////////////////////////////////////// VARIABLE DECLARATIONS ////////////////////////////////////////////////////////////////

#define LOOP_COUNT_SHUTOFF 150
#define RX_BUFF_SIZE 20

//Device data array - threshold (val, onval, offval, onCount, offCount), minOn, maxOff, suff_ex, outlet, algo_enable, manual_enable, manual_value
device_t Devices[6];

//Teach-in state of each device
char teachIn;	//Teach-in enable(1)/disable(0)

//States of relays and LEDs - sent to GPIO using updateGPIO()	
char relayState;
char ledState;


int all_on_iter = 0; // counts the loop iterations to determine when to shut off relays (after allOn())

int allOnPeriod = 0;	// Indicates whether or not devices are in "All On" after RSSI rising edge

//String for int2str serial comm debug
char str[6];


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

	/*Load state from EEPROM*/
	//cli();	// disable interrupts to avoid corruption
	//sei();	// re-enable interrupts
	
	//_delay_ms(5000);
	
	/*GPIO Setup Stuff*/
		// Define Port C I/O Pins as Outputs for Relays and Set to 0
		DDRC |= 0b11111111;
		PORTC = 0b00000000;
	
		// Define Port D I/O Pins as Outputs for LEDs (except lower 2 - serial) and Set to 0
		DDRD |= 0b11111100;
		PORTD = 0b00000000;
	
	
	//Set PORTA to inputs, enable ADC, set ADC prescaler to 64, set analog reference voltage to AVcc, and right justify A/D input
	adcConfig();

	//Configure Bluetooth module, initialize UART, and clear new message flag
	bluetoothConfig();

	//Define device max. off = 0, min. on = 1024
	int i;
	for (i=0; i<6; i++){
		Devices[i].maxOff = 0;
		Devices[i].minOn = 1024;
	}

//////////////////////////////////////////////////////////////// LOOP ////////////////////////////////////////////////////////////////
	
	//Blink LEDs to indicate ready
	PORTD = 0b10000100; _delay_ms(100); PORTD = 0b01001000; _delay_ms(100); PORTD = 0b00110000; _delay_ms(100); PORTD = 0b01001000; _delay_ms(100);
	PORTD = 0b10000100; _delay_ms(100); PORTD = 0b01001000; _delay_ms(100); PORTD = 0b00110000; _delay_ms(100); PORTD = 0b01001000; _delay_ms(100);
	PORTD = 0b10000100; _delay_ms(100); PORTD = 0b00000000;
	

	while (1) 
	{
		// Check Bluetooth Message Buffer
		if(new_msg_flag) {
			_delay_ms(100); // allow some time in case RX buffer is still filling up
			
			//Parse Serial buffer contents into cmd struct
			//command_t cmd = parseCommand(btRxBuffer);
			command_t cmd = parseCommand();
			
			clearBuf();
			new_msg_flag = 0;

			// STORE //////////////////////////////////////////////////////////////////////////////////////////////////////////////
			if(cmd.op == 's')
			{
				
				//Send OK Message
				Serial_Send("k\0");
				
			}
			
			// TEACH /////////////////////////////////////////////////////////////////////////////////////////////////////////////
			else if(cmd.op == 't')
			{	
				
				// TEACH command - toggle device teach-in mode on/off
				teachInToggle((int) cmd.outlet, cmd.mode);
				
				//Return updated device threshold data going out of teach-in mode
				if(teachIn & (1<<cmd.outlet)) {
					//Teach-in on
				}
				else {					
					//On teach-in off, send all device data to app
					
					//Send threshold struct data - 10 bytes
					char send[17];
					
					//Tell Carlos this is the device data
					send[0] = 'I';
					send[1] = 'D';
					
					//Threshold.val
					uint16_t * threshVal = (uint16_t*) &(send[2]);
					*threshVal = Devices[cmd.outlet].threshold.val;
					
					//Threshold.onval
					uint16_t * threshOnVal = (uint16_t*) &(send[4]);
					*threshOnVal = Devices[cmd.outlet].threshold.onval;
					
					//Threshold.offval
					uint16_t * threshOffVal = (uint16_t*) &(send[6]);
					*threshOffVal = Devices[cmd.outlet].threshold.offval;
					
					//Threshold.onCount
					uint16_t * threshOnCount = (uint16_t*) &(send[8]);
					*threshOnCount = Devices[cmd.outlet].threshold.onCount;
					
					//Threshold.offCount
					uint16_t * threshOffCount = (uint16_t*) &(send[10]);
					*threshOffCount = Devices[cmd.outlet].threshold.offCount;
					
					//Devices.minOn
					uint16_t * minOn = (uint16_t*) &(send[12]);
					*minOn = Devices[cmd.outlet].minOn;
					
					//Devices.maxOff
					uint16_t * maxOff = (uint16_t*) &(send[14]);
					*maxOff = Devices[cmd.outlet].maxOff;
					
					//Devices.suff_ex
					send[16] = Devices[cmd.outlet].suff_ex;
				
					//Send threshold data to app
					Data_Send(send, 17);
					
					
					//Serial_Send("Thresh = ");
					char str[6];
					//int2str(Devices[cmd.outlet].threshold.val, str);
					//Serial_Send(str);
					//Serial_Send("\n");
					Serial_Send("offCount = ");
					int2str(Devices[cmd.outlet].threshold.offCount, str);
					Serial_Send(str);
					int2str(cmd.outlet, str);
					Serial_Send(str);
					Serial_Send("\n");
					Serial_Send("onCount = ");
					int2str(Devices[cmd.outlet].threshold.onCount, str);
					Serial_Send(str);
					int2str(cmd.outlet, str);
					Serial_Send(str);
					Serial_Send("\n");
					
				}
				
				//Send OK Message
				Serial_Send("k\0");
			}
			
			// MODE //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			else if(cmd.op == 'm')
			{
				// MODE command - get device teach-in sample, indicating on or off
								
				//Check if outlet is in teach-in mode
				//if(teachIn) {
					int outlet = (int) (log(teachIn)/log(2));
					
					Serial_Send("Before: ");
					int2str(outlet, str);
					Serial_Send(str);
					int2str(Devices[outlet].threshold.onCount, str);
					Serial_Send(str);
					
					//Get teach in - send device/outlet and mode (0 = Off, 1 = On)
					recordTeachIn(outlet, (int)cmd.mode);
					
					Serial_Send("After: ");
					int2str(outlet, str);
					Serial_Send(str);
					int2str(Devices[outlet].threshold.onCount, str);
					Serial_Send(str);
					
					//OK
					/*Serial_Send("Teach-In Recorded: mode = ");
					UART_TX(cmd.mode + 48);
					Serial_Send("\n"); */
					
					//Send updated device info.
					/* int2str(Devices[outlet].threshold.val, str);
					Serial_Send(", Threshold Value: ");
					Serial_Send(str);
					int2str(Devices[outlet].threshold.onCount, str);
					Serial_Send(", onCount = ");
					Serial_Send(str);
					int2str(Devices[outlet].threshold.offCount, str);
					Serial_Send(", offCount = ");
					Serial_Send(str);
					Serial_Send("\n\n\n"); */
				//}
				
				//Send OK Message
				Serial_Send("k\0");

			}
			
			// OVERRIDE /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			else if(cmd.op == 'o') 
			{
				
				//Check for all-on based on RSSI trigger
				if(cmd.outlet == 0xFF) {
					allOnPeriod = 1;
					all_on_iter = 0;
				}
				
				//Check for AUTO control enable
				else if (cmd.mask == 1){
					Devices[cmd.outlet].algo_enable = 1;					
				}
				
				//Otherwise, set to MAN and set relay state
				else {
					
					//Reset AllOn Period to 0
					allOnPeriod = 0;
					
					//Turn on
					if (cmd.mode == 1){
						Serial_Send("State ");
						UART_TX(relayState);
						_delay_ms(100);
						
						relayState |= 1<<cmd.outlet;
						
						Serial_Send("Manual On ");
						UART_TX(cmd.outlet+48);
						
						Serial_Send("State ");
						UART_TX(relayState);
						_delay_ms(100);
					}
					
					//Turn off
					else {
						Serial_Send("State ");
						UART_TX(relayState);
						_delay_ms(100);
						
						relayState &= ~(1<<cmd.outlet);
						
						Serial_Send("Manual Off ");
						UART_TX(cmd.outlet+48);
						
						Serial_Send("State ");
						UART_TX(relayState);
						_delay_ms(100);
					}
					
					Devices[cmd.outlet].algo_enable = 0;
				}
				
				//Send OK Message
				Serial_Send("k\0");
			}
		}
		

		/*If RSSI trigger, activate all relays temporarily*/
		if (allOnPeriod == 1) {
			
			//Close relays
			if (all_on_iter == 0){
				//For all devices with algorithm enabled, set relay state to 1
				for(i=0; i<6; i++) {
					relayState |= (Devices[i].algo_enable << i) & (Devices[i].suff_ex << i);
				}
				
				updateGPIO();
				
				Serial_Send("AllOn Start");
			}
			

			//Increment iteration counter for allOn timing
			//Delay for 100 loops for 150 ms
			all_on_iter++;
			_delay_ms(100);

			if(all_on_iter == LOOP_COUNT_SHUTOFF) {
				// End of "All On" period
				all_on_iter = 0;
				allOnPeriod = 0;
				
				Serial_Send("AllOn End");
				
				//When "All On" period is over, get samples and update relay status based on samples
				char update_mask = 0;
				int i;
				for(i=0; i<6; i++) {
					
					//Check if device is out of teach-in
					if (~(teachIn & (1<<i))){
						
						//Get a sample
						uint16_t sample = getSample(i);
						
						//If device is off and control enabled, open relay
						if (sample < Devices[i].threshold.val && Devices[i].algo_enable==1) {
								update_mask |= (1<<i);
						}
					}
		
				}
				
				//Move not(mask) to relayState
				relayState &= ~update_mask;
				updateGPIO();
			}
		}


		/*Otherwise, for each on device in AUTO, get current sample, compare to threshold, and open relay if necessary*/
		else {
			
			char update_mask = 0;
			int i;
			for(i=0; i<6; i++) {
				//Check if device is out of teach-in
				if (~(teachIn & (1<<i))){

					//Check if relay is currently on
					if (relayState & (1<<i)) {
					
						//Get a sample
						uint16_t sample = getSample(i);
					
						//If device is off and control enabled, open relay
						if (sample < Devices[i].threshold.val && Devices[i].algo_enable==1) {
								update_mask |= (1<<i);
						}
					}
				}
			}
			
			//Move not(mask) to relayState
			relayState &= ~update_mask;
			updateGPIO();
		}


		//Update LEDs based on state of relays
			//If allOn (relays closed after RSSI trigger) - all LEDs off
			//Otherwise, if in teach-in, blink LED
			//Otherwise, set LED state to relay state
			
			
			
			//Check if allOn
			if (allOnPeriod == 1){
				ledState = 0;
			}
			
			//Otherwise, update ledState = relayState
			else {
				ledState = relayState;
				
			}

		updateGPIO();		
	}
}


//////////////////////////////////////////////////////////////// OPERATION FUNCTIONS ////////////////////////////////////////////////////////////////


/* Update GPIO Output Pins */
void updateGPIO () {
	
	//Set PORTC to relayState (mask upper 2 - not used)
	PORTC = relayState & 0x3f;
	
	//Shift LED states up 2 slots to PORTD - pins 0 and 1 are serial
	PORTD = ledState<<2;
	
}


//Toggle teach-in mode of selected outlet
void teachInToggle(int device, unsigned char mode) {
	
	//Set teach-in mode (on or off) for selected device
	if (mode == 1) {
		teachIn |= (1<<device);
		
		Serial_Send("Mode = 1");
		
		//Close relay for teach-in device
		//relayState[device] = 1;
		relayState |= 1<<device;
	}
	
	else {
		teachIn &= ~(1<<device);
		
		Serial_Send("Mode = 0");
	}
	
	//Update GPIO
	updateGPIO();
	
}

//Take sample, classify, refresh threshold, and update counts
void recordTeachIn(int device, int state) {
	
	int sample = getSample(device);
	
	//OFF sample
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

	//ON sample
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
	
	
	//Check device status - if device has at least 3 off and 3 on samples, sufficient samples for algorithm control
	if (Devices[device].threshold.onCount >= 3 && Devices[device].threshold.offCount >= 3) {
		Devices[device].suff_ex = 1;
		
		//Update current off/on threshold
		Devices[device].threshold.val = (Devices[device].threshold.offval + Devices[device].threshold.onval)/2;
		
		//Shift threshold by offset percentage once sufficient samples
		Devices[device].threshold.val -= 0.05*(Devices[device].threshold.onval - Devices[device].threshold.offval);
		
		
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


//////////////////////////////////////////////////////////////// ADC & SAMPLING FUNCTIONS ////////////////////////////////////////////////////////////////

void adcConfig() {
	DDRA= 0x00; //Set port A to inputs (from ADC)  
	
	//Initialize A/D Conversion
	ADCSRA = 1<<ADEN | 1<<ADPS2|1<<ADPS1|0<<ADPS0; // 0x87 // 0b10000111 // Set  ADC Enable bit (7) in ADCSRA register, and set ADC prescaler to 64
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

	#define SAMPLES 1536
	#define MAXPEAKS 100
	
	#define WINDOW 5
	#define SAMPDELAY 15
	
	#define FIND_PEAKS_HALF_WINDOW 2

	int peakCnt = 0;

	uint16_t rawData[SAMPLES];
	uint16_t peaks[MAXPEAKS];
	
	
	uint8_t devSel;
	devSel = (uint8_t) device;
	
	//Select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (devSel & 0x0F);

	// Select ADC Channel based on device
	//switch(device)
	//{
	//Set MUX selectors to choose ADC to sample from
	//case 0: 0<<MUX3|0<<MUX2|0<<MUX1|0<<MUX0;// 0b01100000
	//case 1: 0<<MUX3|0<<MUX2|0<<MUX1|1<<MUX0;// 0b01100001
	//case 2: 0<<MUX3|0<<MUX2|1<<MUX1|0<<MUX0;// 0b01100010
	//case 3: 0<<MUX3|1<<MUX2|0<<MUX1|0<<MUX0;// 0b01100100
	//case 4: 0<<MUX3|1<<MUX2|0<<MUX1|1<<MUX0;// 0b01100101
	//case 5: 0<<MUX3|1<<MUX2|1<<MUX1|0<<MUX0;// 0b01100110
	//}


	//Sample device CS at 3kHz for 1536
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
		_delay_us(SAMPDELAY);
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


//////////////////////////////////////////////////////////////// SERIAL & BLUETOOTH FUNCTIONS ////////////////////////////////////////////////////////////////

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
	Serial_Send("AT+NAMESmartStrip\r\n");	// Name that shows up on phone scan
	_delay_ms(100);	

	new_msg_flag = 0;
}

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
			//ret.threshold.val = (uint16_t)btRxBuffer[2];
			//ret.threshold.onval = (uint16_t)btRxBuffer[4];
			//ret.threshold.offval = (uint16_t)btRxBuffer[6];
			//ret.threshold.onCount = (uint16_t)btRxBuffer[8];
			//ret.threshold.offCount = (uint16_t)btRxBuffer[10];
			ret.mode = (unsigned char)btRxBuffer[2];
			break;
			
		case 'm':
			ret.op = 'm';
			ret.mode = (unsigned char)btRxBuffer[1];
			break;

		case 'o':
			ret.op = 'o';
			/*ret.mask = (unsigned char)btRxBuffer[1];
			ret.duration = (unsigned char)btRxBuffer[2];*/
			// New override message format
			ret.outlet = (unsigned char)btRxBuffer[1];  // (-1 here means start an ALLONPERIOD, otherwise = outlet to override)
			ret.mode = (unsigned char)btRxBuffer[2];	// (0 = off, 1 = on)
			ret.mask = (unsigned char)btRxBuffer[3];	// (0 = manual, 1 = auto)
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

//Send characters over serial
void Serial_Send(char data[])
{
	int i;
	for(i = 0; 1; i++) {
		if(data[i] == '\0') {
			_delay_ms(50);
			return;
		}
		UART_TX(data[i]);
	}
	//Delay added to reset AT-09 buffer
	_delay_ms(50);
}

//Send device data of serial - specified length
void Data_Send(char data[], int length)
{
	int i;
	for(i = 0; i<length; i++) {
		UART_TX(data[i]);
	}
	//Delay added to reset AT-09 buffer
	_delay_ms(50);
}

void UART_TX(unsigned char data)
{
	// Wait until transmit buff is empty
	while( !(UCSR0A & (1<<UDRE0)));

	// Put data into buff
	UDR0 = data;
}

unsigned char UART_RX(void)
{
	// Wait for data to be available
	while( !(UCSR0A & (1<<RXC0)));

	// return data in buff
	return UDR0;
}

void clearBuf() 
{
	buff_i = 0;
	int i;
	for(i=0; i<RX_BUFF_SIZE; i++) {
		btRxBuffer[i] = 0;
	}		
}

