/* Collects samples out of ADC onto an array 
 * and sends the array over serial. 
 *
 * Make sure the clock speed is 8 MHz! Otherwise
 * serial communication will not work. 
 *
 * This code 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

/* Every (BAUDCOUNTER + 1 )clock cycles a baud is generated. 
 * (8 MHz / 16) / (51+1) = 9615 Baud, which is the closest
 * we can get to 9600. 
 */

#define BAUDCOUNTER 51

void UART_Init(unsigned int ubrr);
void Serial_Send(volatile char string[]);
void int2str(uint16_t, char*);
void UART_TX(unsigned char data);
int Serial_Available();
unsigned char UART_RX(void);
int equals(volatile char a[], const char b[]);

#define SIZE 50
// Volatile declarations needed to prevent compiler
// from overlooking changes to values during ISR
volatile char btCommand[SIZE];	 
volatile int buff_i = 0;
volatile int new_msg_flag;

ISR(USART_RX_vect) {
	btCommand[buff_i++] = UART_RX();
	if(buff_i == SIZE)
		buff_i = 0;

	new_msg_flag = 1;
}

void clearBuf()
{
	buff_i = 0;
	int i;
	for(i=0; i<SIZE; i++)
		btCommand[i] = 0;
}

int main(void)
{
	// Define PB0 Output I/O Pin
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
	Serial_Send("AT+NAMESmartStrip\r\n");	// name that shows up on phone scan
	_delay_ms(100);	


	// clear new message flag (buffer contains multiple "OK")
	new_msg_flag = 0;	
	clearBuf();

	int i = 0;
	while(1)
	{
		while(new_msg_flag==0);

		_delay_ms(100); // wait a little for characters to fill buffer

		Serial_Send("**");
		char intstr[5];
		int2str(i++, intstr);
		Serial_Send(intstr);
		Serial_Send(btCommand);
		Serial_Send("//");

		if(equals(btCommand, "off")) {
			PORTB = 0b00000000; // LED off
		}

		else if(equals(btCommand, "on")) {
			PORTB = 0b00000001; // LED on
		}

		else if(equals(btCommand, "toggle")) {
			PORTB ^= 0b00000001; // LED toggle
		}

		clearBuf();
		new_msg_flag = 0;
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

	// Enable RX Complete Interrupts
	UCSR0B |= (1<<RXCIE0);
	sei();
}

void Serial_Send(volatile char data[])
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

int equals(volatile char a[], const char b[]) {
	int i = 0;
	while(1) {
		if(a[i] != b[i] || i > 256) 
			return 0;
		if(a[i] == '\0')
			return 1;
		i++;
	}
}


