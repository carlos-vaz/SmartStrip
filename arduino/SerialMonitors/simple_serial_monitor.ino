/*
 * A simple program that monitors serial transmissions
 * on pin 2 and outputs them onto your computer terminal.
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

