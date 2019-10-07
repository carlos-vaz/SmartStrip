/* Interfaces with the AT-09 Serial Bluetooth Module
 * and responds to LED off/on/toggle commands from 
 * a Bluetooth master. 
 * 
 * Make sure the clock speed is 8 MHz! Otherwise
 * serial communication will not work. 
 *
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

/* Every (BAUDCOUNTER + 1 )clock cycles a baud is generated. 
 * (8 MHz / 16) / (51+1) = 9615 Baud, which is the closest
 * we can get to 9600. 
 */

#define BAUDCOUNTER 51

void UART_Init(unsigned int ubrr);
void Serial_Send(char string[]);
void int2str(uint16_t, char*);
void UART_TX(unsigned char data);
int Serial_Available();
unsigned char UART_RX(void);
int equals(char a[], const char b[]);

char btCommand[50];

int main(void)
{
	// Define PB0 Input I/O Pin
	DDRB |= 0b00000001;

	// Setup UART
	UART_Init(BAUDCOUNTER);

	_delay_ms(1000);
	PORTB = 0b00000001; // led on

	// Configure AT-09 BT Module
	Serial_Send("AT+ROLE0\r\n");		// Role = peripheral (responds)
	_delay_ms(100);	
	Serial_Send("AT+UUID0xFFE0\r\n");	// Set UUID for service (phone looks for this)
	_delay_ms(100);	
	Serial_Send("AT+CHAR0xFFE1\r\n");	// Set UUID for characteristic (phone looks for this)
	_delay_ms(100);	
	Serial_Send("AT+NAMESmartStrip\r\n");	// ame that shows up on phone scan
	_delay_ms(100);	


	while(1)
	{
		while(!Serial_Available()); // wait for message

		int i = 0;
		while(Serial_Available())
		{
			btCommand[i++] = UART_RX();
			_delay_ms(10);
		}
		btCommand[i] = 0;

		Serial_Send("**");
		Serial_Send(btCommand);
		Serial_Send("**");

		if(equals(btCommand, "off")) {
			PORTB = 0b00000000; // LED off
		}

		else if(equals(btCommand, "on")) {
			PORTB = 0b00000001; // LED on
		}

		else if(equals(btCommand, "tog")) {
			PORTB ^= 0b00000001; // LED toggle
		}
	}

	return 0;
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

