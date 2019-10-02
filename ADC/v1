#include <asf.h>
#include <avr/io.h>
#include <avr/interrupt.h>


uint8_t  Current1;
uint8_t  Current2;
uint16_t  Current;
int main (void)
{
	//Initialize pins
	DDRD = 0xFF; //Set port D to outputs (to relays)
	DDRA= 0x00; //Set port A to inputs (from ADC)  
	
	//Initialize A/D Conversion
	ADCSRA = 1<<ADEN | 1<<ADPS2|0<<ADPS1|1<<ADPS0; // 0x87 // 0b10000111 // Set  ADC Enable bit (7) in ADCSRA register, and set ADC prescaler to 32
	ADMUX=0<<REFS1|1<<REFS0|0<<ADLAR; //0x60; // 0b01100000  // select Analog  Reference voltage to be AVcc and use ADC0 and use right justify


	while(1){
		ADCSRA|=(1<<ADSC); // Start conversion 
		while ((ADCSRA & (1<<ADIF))==0); // wait for completion
		Current2=ADCL;
		Current1=ADCH;
		Current= (Current1<<8)|Current2;
		Current= Current & 0x3FF;

	}
}
