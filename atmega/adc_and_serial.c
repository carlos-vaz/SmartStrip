/* Collects samples out of ADC onto an array 
 * and sends the array over serial. 
 *
 * Make sure the clock speed is 8 MHz! Otherwise
 * serial communication will not work. 
 *
 * This code has only been tested with the ATmega328P
 * but should also work with the 644 
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

/* Every (BAUDCOUNTER + 1 )clock cycles a baud is generated. 
 * (8 MHz / 16) / (51+1) = 9615 Baud, which is the closest
 * we can get to 9600. 
 */

#define BAUDCOUNTER 51

uint16_t sampleADC();
void UART_Init(unsigned int ubrr);
void Serial_Send(char string[]);
void int2str(uint16_t, char*);
void UART_TX(unsigned char data);
unsigned char UART_RX(void);

int main(void)
{
	// Define PB0 Input I/O Pin
	DDRB |= 0b00000001;

	// Setup UART
	UART_Init(BAUDCOUNTER);

	// Setup ADC
	ADCSRA = 1<<ADEN | 1<<ADPS2|1<<ADPS1|1<<ADPS0; // 0b10000111 // Set  ADC Enable bit (7) in ADCSRA register, and set ADC prescaler to 128
	ADMUX = 0<<REFS1|1<<REFS0|0<<ADLAR; // 0b01000000  // AVCC with external capacitor at AREF pin + use ADC0 + use right justify

	_delay_ms(1000);
	Serial_Send("Ready to rumble...\n");

	while (1)
	{
		uint16_t val = sampleADC();
		//uint16_t val = 65535; 
		char valString[5];
		int2str(val, valString);
		Serial_Send("val = ");
		Serial_Send(valString);
		Serial_Send("\n");

		PORTB = PORTB ^ 0b00000001; // toggle LED
		//_delay_ms(50); 
	}

	return 0;
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

unsigned char UART_RX(void)
{
	// Wait for data to be available
	while( !(UCSR0A & (1<<RXC0)));

	// return data in buff
	return UDR0;
}


