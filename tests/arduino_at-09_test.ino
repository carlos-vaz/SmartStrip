#include <SoftwareSerial.h>

SoftwareSerial btSerial(2,3);
char cmd[100];		// Buffer to hold command from Smartphone
char reply[100];	// Buffer to hold reply from AT-09 after "AT" commands
int led = 0;		// State of LED

void setup() {
	Serial.begin(115200);
	btSerial.begin(9600);
	delay(1000);
	Serial.println("Starting...");

	// Configure AT-09
	sendCommand("AT");			// Does nothing
	sendCommand("AT+ROLE0");		// Role = peripheral (responds)
	sendCommand("AT+UUID0xFFE0");		// Set UUID for service (phone looks for this)
	sendCommand("AT+CHAR0xFFE1");		// Set UUID for characteristic (phone looks for this)
	sendCommand("AT+NAMESmartStrip");	// Name that shows up on phone scan
	delay(1000);

	pinMode(7, OUTPUT);
	digitalWrite(7, led);
}

void loop() {
	checkInbox();
	delay(100);
}

void sendCommand(const char * command) {
	// Send command to AT-09
	Serial.print("Command: ");
	Serial.println(command);
	btSerial.println(command);
	delay(100);
	// Receive reply
	int i = 0;
	while (btSerial.available()) {
		reply[i++] = btSerial.read();
	}
	reply[i] = '\0';
	Serial.print("Reply: ");
	Serial.println(reply);
}

void checkInbox() {
	int i = 0;
	if(btSerial.available()) {
		// Read phone's command from the BLE serial buffer
		Serial.print("value: ");
		while(btSerial.available()) {
			cmd[i++] = (char)btSerial.read();
		}
		cmd[i] = '\0';
		Serial.print("recevied: ");
		Serial.println(cmd);
		
		// Act accordingly 
		if( equals(cmd, "led") ) {
			Serial.println("Toggling LED");
			led = (led + 1) % 2;
			digitalWrite(7, led);
		}
		else if( equals(cmd, "msg") ) 
			Serial.println("Here's a message");
		else
			Serial.println("CMD NOT RECOGNIZED!");
		Serial.println("");
	}
}

boolean equals(char a[], const char b[]) {
	int i = 0;
	while(1) {
		if(a[i] != b[i] || i > 256) 
			return false;
		if(a[i] == '\0')
			return true;
		i++;
	}
}

