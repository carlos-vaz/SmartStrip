/*
 * A simple program that reads from the ATmega serial
 * TX pin and relays it to the RX pin of the Serial-USB
 * adapter on the Arduino so that your computer can read
 * the values. 
 */

#include <SoftwareSerial.h>

SoftwareSerial atmega_serial (2,3); //(RX, TX)

void setup() {
	Serial.begin(115200);
	atmega_serial.begin(9600);
}

void loop() {
	while(atmega_serial.available() > 0) {
		Serial.print((char)atmega_serial.read());
	}
}

