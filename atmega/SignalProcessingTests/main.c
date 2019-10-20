#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>


int getSample();
void ADC_Init();
uint16_t sampleADC();
void UART_Init(unsigned int ubrr);
void Serial_Send(char string[]);
void int2str(uint16_t, char*);
void UART_TX(unsigned char data);

#define BAUDCOUNTER 51

#define SAMPLES 256
#define MAXPEAKS 10
#define WINDOW 5
#define FIND_PEAKS_HALF_WINDOW 2


int rawData[SAMPLES];
int filteredData[SAMPLES];
int extendedPeaks[SAMPLES];
int final[SAMPLES];
int peaks[MAXPEAKS];

int main() {
	// Define PB0 Input I/O Pin
	DDRB |= 0b00000001;

	_delay_ms(500);
	PORTB = 0b00000000; // LED off

	ADC_Init();

	UART_Init(BAUDCOUNTER);


	_delay_ms(3000);
	PORTB = 0b00000001; // LED on

	getSample();

	// OUTPUT CSV FORMAT: [raw],[filtered],[last peak],[final sample value]

	int i;
	char str[6];
	for(i=0; i<SAMPLES; i++) {
		int2str(rawData[i], str);
		Serial_Send(str);
		Serial_Send(",");
		int2str(filteredData[i], str);
		Serial_Send(str);
		Serial_Send(",");
		int2str(extendedPeaks[i], str);
		Serial_Send(str);
		Serial_Send(",");
		int2str(final[i], str);
		Serial_Send(str);
		Serial_Send("\n");
	}

	while(1);


	return 0;
}

void ADC_Init()
{
	// Set  ADC Enable bit (7) in ADCSRA register, and set ADC prescaler to 64
	// 0b10000111 
	ADCSRA = 1<<ADEN | 1<<ADPS2|1<<ADPS1|0<<ADPS0; 
	// AVCC with external capacitor at AREF pin + use ADC0 + use right justify
	// 0b01000000
	ADMUX = 0<<REFS1|1<<REFS0|0<<ADLAR;
}

uint16_t sampleADC()
{
	ADCSRA|=(1<<ADSC); // Start conversion 
	while ((ADCSRA & (1<<ADIF))==0); // wait for completion
	unsigned char Current2 = ADCL;
	unsigned char Current1 = ADCH;
	uint16_t Current = (Current1<<8)|Current2;
	Current = Current & 0x3FF;
	return Current;
}

/*Get filtered current sample from selected device*/
int getSample() {

	int peakCnt = 0;


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
			filteredData[i-WINDOW+1] = (int) avg;
		}


		//Delay - See window vs. delay table
		_delay_us(15);
	
		//Toggle GPIO on every 5 samples
		//For fs = 3000Hz, GPIO should be at 300Hz
		if ((i%5) == 0) {
			PORTB = ~PORTB; // Toggle GPIO
		}
	}


	int extendedPeak = 0;

	//Find peaks in sample array - move through array, find elements that end a zeros (previously negative section), then find max. values between each zeros section
	for(i=FIND_PEAKS_HALF_WINDOW-1; i<SAMPLES-FIND_PEAKS_HALF_WINDOW-WINDOW; i++) {

		int k, max = 0;

		// Find max around window center
		for(k=-1*FIND_PEAKS_HALF_WINDOW; k<=FIND_PEAKS_HALF_WINDOW; k++) {
			if(filteredData[i+k] > max) {
				max = filteredData[i+k];
			}	
		}

		// Check if point = max
		if(filteredData[i] == max) {
			peaks[peakCnt++] = filteredData[i];
			extendedPeak = rawData[i];
		}

		extendedPeaks[i] = extendedPeak;

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


	for(i=0; i<SAMPLES; i++) {
		final[i] = (int) sample;
	}

	return (int) sample;
}


void int2str(uint16_t val, char* str)
{
	int digit, i;
	for(i=0; i<5; i++) {
		digit = val / pow(10,(4-i));
		str[i] = digit + 48; // ascii offset
		val = val - (pow(10,(4-i)) * digit);
	}
	str[5] = '\0';
}

void UART_Init(unsigned int ubrr)
{
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	// Enable receiver/transmitter
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	// Set frame format: 8 data + 2 stop bits
	UCSR0C = (1<<USBS0) | (3<<UCSZ00);
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

