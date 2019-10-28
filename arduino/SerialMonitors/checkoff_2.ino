/*
 * Two-way communication from your computer terminal 
 * to ATmega via Arduino
 */

#include <SoftwareSerial.h>

void clearSendBuff();

SoftwareSerial atmega_serial (8,9); //(RX, TX)
#define BUFFLEN 50
char sendBuff[BUFFLEN];

void setup() {
  Serial.begin(115200);
  atmega_serial.begin(9600);
}

void loop() {
  // DISPLAY ATMEGA RESPONSES
  while(atmega_serial.available() > 0) {
    Serial.print((char)atmega_serial.read());
  }

  // SEND ATMEGA COMMANDS
  int i = 0;
  clearSendBuff();
  while(Serial.available() > 0) {
    sendBuff[i++] = Serial.read();
  }
  int ignore = 0;
  for(int j=0; j<i; j++) {
    if(ignore) {
      ignore = 0;
    } else if(sendBuff[j] == '*') {
      unsigned char binVal = sendBuff[j+1] - 48;
      atmega_serial.write(&binVal, 1);
      ignore = 1;
    } else {
      atmega_serial.print(sendBuff[j]);
    }
  }
  
}

void clearSendBuff() {
  for(int i=0; i<BUFFLEN; i++) {
    sendBuff[i] = 0;
  }
}

/*
 * t*0*0*0*0*0*0*0*0*0*0*0
 * 
 */
