/*
 * SD_Concept_Testing_09142019.c
 *
 * Created: 9/14/2019 10:58:03 AM
 * Author : ngh9
 */ 

#include <avr/io.h>
#include <util/delay.h>

#define BAUDCOUNTER 6	// Every BAUDCOUNTER clock cycles a baud is generated
			// (1 MHz / 16) / 6 = 10416.67 Baud (closest to 9600)

void UART_Init(unsigned int ubrr);
void UART_Transmit(unsigned char data);
unsigned char UART_Receive(void);

int main(void)
{
	// Define PB0 Input I/O Pin
	DDRB |= 0b00000001;

	// Setup UART
	UART_Init(BAUDCOUNTER);

	while (1)
	{
		PORTB = 0b00000001;
		_delay_ms(500);
	
		UART_Transmit(112);

		PORTB = 0b00000000;
		_delay_ms(500);
	}

	return 0;
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

void UART_Transmit(unsigned char data)
{
	// Wait until transmit buff is empty
	while( !(UCSR0A & (1<<UDRE0)));

	// Put data into buff
	UDR0 = data;
}

unsigned char UART_Receive(void)
{
	// Wait for data to be available
	while( !(UCSR0A & (1<<RXC0)));

	// return data in buff
	return UDR0;
}


