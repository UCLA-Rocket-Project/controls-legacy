#include <Arduino.h>
#include "pcf8574.h"

// hacky plan: combine the top 8-N bits of PORT1 with the bottom N bits of PORT2
#define PORT1 PORTD
#define DDR1 DDRD
#define PIN1 PIND

#define PORT2 PORTB
#define DDR2 DDRB
#define PIN2 PINB

// # of bits to read from PORT2 (the rest of them will come from PORT1)
const byte PORT2_BITS = 3;
const byte PORT1_MASK = 0xFF << PORT2_BITS;
const byte PORT2_MASK = ~PORT1_MASK;

PCF8574 leds(0x20); //leds are ~switches since they go 0 to flash the light
PCF8574 ethernet(0x21);
byte switchValue = 0xFF & ~1;
byte lastCachedRead = 0xFF;
byte ledVal = 0xFF;
long readValid = 0;
bool keyInserted = false;

void printBinary(byte inByte);
byte reverse(byte inByte);

void setup () {
	// setup ports to all input pllup
	Serial.begin(115200);
	DDR1 &= ~PORT1_MASK;
	PORT1 |= PORT1_MASK;
	DDR2 &= ~PORT2_MASK;
	PORT2 |= PORT2_MASK;

	leds.write8(0xFF);
	ethernet.write8(0xFF);
}

long lastFlash = 0;
long FLASH_MASK = 0b11000000;

void loop() {
	/*  
		initially, since we read across 2 bytes the bit order is:
		2 3 4 5 6 7 0 1
		the below renormalizes it to 
		7 6 5 4 3 2 1 0
	*/
	byte rawPort = (PIN1 & PORT1_MASK) | (PIN2 & PORT2_MASK);
	rawPort = reverse(rawPort);
	byte port = rawPort << PORT2_BITS | rawPort >> (8 - PORT2_BITS);

	port = ~port; // since inputs are on when they are switched to gnd
	if(port != lastCachedRead) {
		lastCachedRead = port;
		readValid = millis();
	}
	else if(switchValue != lastCachedRead && millis() > readValid + 25) { // 25ms debounce
		switchValue = lastCachedRead;
		printBinary(switchValue);
		// update everything in terms of switchValue
		keyInserted = switchValue & 1;
		if(keyInserted) {
			Serial.println(F("FLASH ENABLED"));
		}
		// if key is not inserted, ignore all bits from PORT2 (equivalently, take only bits from PORT1)
		if(!keyInserted) {
			switchValue &= PORT1_MASK;
		}
		ledVal = ~switchValue;
		leds.write8(ledVal);
		ethernet.write8(switchValue);
	}
	if(keyInserted && millis() > lastFlash + 50) { // flash the top 2 leds every 50 ms
		lastFlash = millis();
		if(switchValue & FLASH_MASK == 0) { //only flash if the switches themselves are off
			ledVal ^= FLASH_MASK;
			leds.write8(ledVal);
		}
	}
}
byte reverse(byte b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}
void printBinary(byte inByte) {
  for (int b = 7; b >= 0; b--) {
    Serial.print(bitRead(inByte, b));
  }
  Serial.println();
}