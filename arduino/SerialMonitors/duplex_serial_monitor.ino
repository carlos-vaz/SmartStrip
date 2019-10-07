/*
 * A simple program that monitors the TX and RX serial pins
 * of the ATmega (which will be connected to the Serial Bluetooth
 * module). Two software serial instances are used, and only the RX
 * is relevant. 
 */

#include <SoftwareSerial.h>

SoftwareSerial atmega_tx (8,9); //(RX, TX)
SoftwareSerial atmega_rx (10,11); 

void setup() {
	Serial.begin(115200);
	atmega_tx.begin(9600);
	atmega_rx.begin(9600);
	delay(500);
}

void loop() {

	atmega_tx.listen();
	delay(50);
	if(atmega_tx.available() > 0) 
	{ 
		Serial.print("ATMEGA says: ");
		while(atmega_tx.available() > 0)
		{
			Serial.print((char)atmega_tx.read());
			delay(10);
		}
	}

	atmega_rx.listen();
	delay(50);
	if(atmega_rx.available() > 0) 
	{ 
		Serial.print("AT-09 says: ");
		while(atmega_rx.available() > 0)
		{
			Serial.print((char)atmega_rx.read());
			delay(10);
		}
	}
}
