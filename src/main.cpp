/* LICENSE
 *
 */
/*
 * Includes
 */
#if ARDUINO >= 100
#include "Arduino.h"			 // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"			// for delayMicroseconds
#include "pins_arduino.h"	// for digitalPinToBitMask, etc
#endif
#include <avr/sleep.h>
#include <avr/pgmspace.h>
/*
 * Library classs includes
 */
#include <Wire.h>
#include <DS2482.h>
#include <OneWire.h>
#include <CmdCallback.hpp>
#include <CmdParser.hpp>
#include "WireWatchdog.h"

/*
 * Local constants
 */

#define LED_ON() digitalWrite(13, 1); \
					ledOn = 1;
#define LED_OFF() digitalWrite(13, 0); \
					ledOn = 0;
#define MAX_CMD_BUFSIZE 16

const byte ow_pin = 3;
extern byte mode;

/*
 * Objects
 */
/*DS2482 ds1(0);*/
OneWire ds(ow_pin);
WireWatchdog wdt(ow_pin, 0x2f);
CmdCallback<5> cmdCallback;
CmdParser cmdParser;

/*
 * Local variables
 */
byte alarmSignal, wdFired, ledOn = 0;
unsigned long ledOnTime = 0;
static unsigned long owLowStart = 0;
unsigned long wdTime = 0;
unsigned long timeStamp = 0;
static unsigned long alarmPolling = 0;
boolean stringComplete = false;  // whether the string is complete
String inputString = "";         // a String to hold incoming data

/*
* Function declarations
*/
void ow_monitor();
bool alarm_handler();
uint8_t ds2408DataRead(OneWireBase *ds, uint8_t* addr, uint8_t* data);

void wdtAlarm() 
{
	Serial.println("Alarm test");
}

void test_detect(OneWire *ds, uint8_t t_slot, uint8_t t_w1l, uint8_t t_w0l, uint8_t t_rc, uint8_t t_msr)
{
	byte addr[8], i, j;
	
	ds->t_slot = t_slot;
	ds->t_w0l = t_w0l;
	ds->t_w1l = t_w1l; 
	ds->t_rc = t_rc;
	ds->t_msr = t_msr;

	Serial.print(t_slot);
	Serial.write(' ');
	Serial.print(t_w0l);
	Serial.write(' ');
	Serial.print(t_w1l);
	Serial.write(' ');
	Serial.print(t_msr);
	Serial.write(' ');
	Serial.println(t_rc);
	/* slot	low	int	rc 	rd
	*/
	ds->reset_search();
	if (ds->reset() == 0) {
			Serial.println("	No devices!");
			return;
	}
	j = 0;
	while (j < 7 && ds->search(addr)) {
		Serial.print("	#");
		Serial.print(j++);
		for (i = 0; i < 8; i++) {
			Serial.print(' ');
			Serial.print(addr[i], HEX);
		}
		Serial.println();
	}
	if (j < 3 || j > 8)
		Serial.print("?");
}

void ow_line_check(OneWire *ds) {
	byte slot[] = { 80 };
	byte t_msr[] = { 13, 14, 15 };
	byte t_w0l[] = { 70 };
	byte t_rc[] = { 4, 5, 6, 7 };
	byte t_w1l[] = { 9, 10, 11, 12 };
	byte s, w0l, rc, msr, w1l;

	Serial.println("line and device check");
	for (s = 0; s < sizeof(slot); s++)
		for (w0l = 0; w0l < sizeof(t_w0l); w0l++)
			for (msr = 0; msr < sizeof(t_msr); msr++)
				for (rc = 0; rc < sizeof(t_rc); rc++)
					for (w1l = 0; w1l < sizeof(t_w1l); w1l++) {
						Serial.print(".");
						if (t_w0l[w0l] > slot[s]) {
							//Serial.print(F("w0l > slot "));
							continue;
						}
						// T_SLOT - T_W1L + T_RC
						if (t_w1l[w1l] + t_rc[rc] > slot[s]) {
							continue;
						}
						// T_MSR - T_W1L
						if (t_msr[msr] < t_w1l[w1l]) {
							//Serial.print(F("msr < w1l "));
							continue;
						}
						// T_W1L - T_RC)
						if (t_w1l[w1l] < t_rc[rc]) {
							//Serial.print(F("w1l < rc "));
							continue;
						}
						test_detect (ds, slot[s], t_w1l[w1l], t_w0l[w0l], t_rc[rc], t_msr[msr]);
						delay (10);
					}
}

void owSearch(OneWireBase *ds)
{
	byte adr[8];
	byte j = 0;

	Serial.print(F("reset..."));
	if (!ds->reset()) {
		Serial.println(F("no devs!"));
		return;
	} 
	Serial.println(F("success!"));
	ds->reset_search();
	//ds->configure(DS2482_CONFIG_APU); 
	delay (1);

	while (ds->search(adr)) {
		Serial.print("#");
		Serial.print(j++);
		Serial.print(":");
		for (int i = 0; i < 8; i++) {
			Serial.write(' ');
			Serial.print(adr[i], HEX);
		}
		Serial.println();
	}
	Serial.print(j);
	Serial.println(" sensors found");
}

void owDataRead(OneWireBase *ds)
{
	byte adr[8], data[8];

	Serial.println(F("Data read"));
	adr[0] = 0x29;
	adr[1] = 0;
	adr[2] = 0x48;
	adr[3] = 0x3;
	adr[4] = 0x5;
	adr[5] = 0x66;
	adr[6] = 0x77;
	adr[7] = ds->crc8 (adr, 7);
	Serial.print(adr[1], HEX);
	Serial.print("..");
	Serial.print(adr[6], HEX);
	Serial.print(' ');
	Serial.println(adr[7], HEX);
	ds2408DataRead(ds, adr, data);
	adr[1] = 0x42;
	adr[7] = ds->crc8 (adr, 7);
	Serial.print(adr[1], HEX);
	Serial.print("..");
	Serial.print(adr[6], HEX);
	Serial.print(' ');
	Serial.println(adr[7], HEX);
	ds2408DataRead(ds, adr, data);
	adr[1] = 0xA2;
	adr[2] = 0xd9;
	adr[3] = 0x84;
	adr[4] = 0x0;
	adr[5] = 0x16;
	adr[6] = 0x4;
	adr[7] = ds->crc8 (adr, 7);
	Serial.print(adr[1], HEX);
	Serial.print("..");
	Serial.print(adr[6], HEX);
	Serial.print(' ');
	Serial.println(adr[7], HEX);
	ds2408DataRead(ds, adr, data);
}

void wdtCommand(uint8_t cmd)
{
	switch (cmd)
	{
	case 0x5A:
		owSearch(&ds);
		//alarm_handler();
		break;
	case 0x4B:
		owDataRead(&ds);
		break;
	default:
		Serial.print("unknown ");
		Serial.println(cmd, HEX);
		break;
	}
}

void funcSearch(CmdParser *myParser)
{
	owSearch(&ds);
}

void funcData(CmdParser *myParser)
{
	owDataRead(&ds);
}

void funcTest(CmdParser *myParser)
{
	ow_line_check(&ds);
}

void funcMode(CmdParser *myParser)
{
	mode = atoi(myParser->getCmdParam(1));
	Serial.print(F(" mode="));
	Serial.print(mode);
}

void funcPinSet(CmdParser *myParser)
{
	uint8_t pin, val;

	Serial.print(F("pset: "));
	pin = atoi(myParser->getCmdParam(1));
	Serial.print(F(" pin: "));
	Serial.print(pin);
	Serial.print(F(" val: "));
	val = atoi(myParser->getCmdParam(2));
	Serial.println(val);
	pinMode(pin, OUTPUT);
	digitalWrite (pin, val);
}

void resetInput()
{
	// clear the string:
	inputString = "";
	stringComplete = false;
	Serial.print(F("\now:# "));
	Serial.flush();
}

/*
 * SerialEvent occurs whenever a new data comes in the hardware serial RX. This
 * routine is run between each time loop() runs, so using delay inside loop can
 * delay response. Multiple bytes of data may be available.
 */
void serialEvent() {
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		// ESC
		if (inChar == 27) {
			Serial.println();
			resetInput();
			return;
		}
		// if the incoming character is a newline, set a flag so the
		// main loop can do something about it:
		if (inChar == 0x0d) {
			stringComplete = true;
			Serial.println();
			return;
		}
		if (inChar == 0x08 ||
				(inChar > 0x1f && inChar < 0x7f)) {
			// add it to the inputString:
			inputString += inChar;
			Serial.print(inChar);
			Serial.flush();
			return;
		}
	}
}


void setup() {
	ADCSRA = 0;	// disable ADC
	Serial.begin(115200);

	digitalWrite(A1, HIGH);
	pinMode(A1, OUTPUT);
	digitalWrite(A3, LOW);
	pinMode(A3, OUTPUT);
	Wire.begin(); 
	wdt.onAlarm(wdtAlarm);
	wdt.onCommand(wdtCommand);
	wdt.begin();

	Serial.println(F("One Wire Watchdog active"));
	cmdCallback.addCmd("srch", &funcSearch);
	cmdCallback.addCmd("pset", &funcPinSet);
	cmdCallback.addCmd("data", &funcData);
	cmdCallback.addCmd("test", &funcTest);
	cmdCallback.addCmd("mode", &funcMode);
	// reserve bytes for the inputString
	inputString.reserve(MAX_CMD_BUFSIZE);
	resetInput();
	/*noInterrupts ();
	interrupts ();
	*/
	delay (200);
	::attachInterrupt(digitalPinToInterrupt(ow_pin), ow_monitor, CHANGE);
	wdFired = 0;
	delay (200);
	digitalWrite(A3, HIGH);
	wdTime = millis();
	alarmPolling = millis();
}

/* interrupt handling for change on 1-wire */
void ow_monitor()
{
	bool p = wdt.lineRead();
	unsigned long now = micros();

	if (p) {
		unsigned long duration = now - owLowStart;
		digitalWrite(A3, 1);
		wdTime = millis();
		if (owLowStart == 0)
			return;
		owLowStart = 0;
		if (duration > 800) {
			timeStamp = millis();
			LED_ON();

			ledOnTime = millis();
			alarmSignal = 1;
			return;
		}
		if (duration > 300) {
			// reset
		}
	} else {
		digitalWrite(A3, 0);
		// went down, log (reduce by some overhead)
		owLowStart = now - 50;
	}
}

void ledBlink()
{
	if (millis() - ledOnTime > 300) {
		if (ledOn) {
			ledOnTime = millis();
			LED_OFF();
		}
		else if (wdFired) {
			LED_ON();
			ledOnTime = millis();
		}
	}
}

void cmdLoop()
{
	unsigned char cmdBuf[MAX_CMD_BUFSIZE];

	if (stringComplete) {
		// issue with trailing 0, maybe using c_str
		inputString.getBytes(cmdBuf, inputString.length() + 1);

		if (cmdParser.parseCmd(cmdBuf,
			inputString.length()) != CMDPARSER_ERROR) {
#if 0				
			Serial.print("Call ");
			Serial.print((char)cmdBuf[0], HEX);
			Serial.print("...");
			Serial.print((char)cmdBuf[3], HEX);
			Serial.print(" ");
			Serial.print((char)cmdBuf[4], HEX);
			Serial.print(" ");
			Serial.println(inputString.length());
#endif			
			cmdCallback.processCmd(&cmdParser);
		} else
		{
			Serial.println("?");
		}
		
		resetInput();
	}
}

void loop()
{
	wdt.loop();
	if (ledOnTime != 0 && !alarmSignal) {
		ledBlink();
	}
	if (alarmSignal) {
		bool p;
		int cnt = 1000;
/*
		::detachInterrupt(digitalPinToInterrupt(ow_pin));
		::attachInterrupt(digitalPinToInterrupt(ow_pin), ow_monitor, CHANGE);
*/
		wdTime = millis();
		Serial.println(F("Alarms signal "));
 		do {
			delayMicroseconds (150);
			p = wdt.lineRead();
			if (p)
				break;
			delayMicroseconds (500);
			if (cnt-- == 0) {
				Serial.println(F("OW line fail"));
				break;
			}
		} while (!p);
		// interrupt to host
		digitalWrite (A1, LOW);
		wdTime = millis();
		alarmPolling = millis();
		if (mode & MODE_ALRAM_HANDLING) {
			alarm_handler();
		}
		alarmSignal = 0;
	}
	if (millis() - alarmPolling > 500) {
		alarmPolling = millis();
		if (mode & MODE_ALRAM_POLLING) {
			alarm_handler();
		}
	}
	if (mode & MODE_WATCHDOG && (wdFired == 0 && millis() - wdTime > 5000)) {
		wdFired = 1;
		Serial.println(F("Watchdog!"));
		//LED_ON();
		ledOnTime = millis();
	}
	cmdLoop();
	 /*else
		sleep_cpu();*/
	sleep_enable();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_cpu();
}

void toggleDs2413(OneWireBase *ds, uint8_t* addr)
{
	unsigned char pio, pion;
	do {
		ds->reset();
		ds->select(addr);
		ds->write (0xF5);
		pio = ds->read();
	} while (((pio & 0xF) != (~(pio >> 4) & 0xF)));
	if ((pio & 2) == 0)
		pion = 3;
	else
		pion = 2;
	pion |= 0x2;
	do {
		ds->reset();
		ds->select(addr);
		ds->write (0x5A);
		ds->write (0xFC | pion);
		ds->write (0xFF & (~(0xFC | pion)));
		pio = ds->read();
	} while (pio != 0xAA);
}

uint8_t ds2408DataRead(OneWireBase *ds, uint8_t* addr, uint8_t* data)
{
	uint8_t tmp, i;

	/* read latch */
	ds->reset();
	ds->select(addr);
	// read data registers
	ds->write (0xF0);
	/* target address activity latch state */
	ds->write (0x89);
	ds->write (0);
	for (i = 0; i < 7; i++) {
		delayMicroseconds (10);
		data[i] = ds->read();
	}
	
	ds->reset();
	ds->select(addr);
	ds->write (0xC3);
	delayMicroseconds (100);
	tmp = ds->read();
	if (tmp != 0xAA) {
		Serial.println(F("latch reset error"));
		return tmp;
	}
	Serial.print(F("Data "));
	for (i = 0; i < 6; i++) {
		Serial.print(data[i], HEX);
		Serial.print(F(" "));
	}
	Serial.println(data[i], HEX);

	return tmp;
}

uint8_t ds2408TogglePio(OneWireBase *ds, uint8_t* addr, uint8_t pio, uint8_t* data)
{
	uint8_t d;

	if (data == NULL) {
		ds->reset();
		ds->select(addr);
		// read data registers
		ds->write (0xF0);
		/* target address latch state */
		ds->write (0x89);
		ds->write (0);
		delayMicroseconds (100);
		d = ds->read();
	} else
		d = *data;
	
	if (d & pio)
		d &= ~(pio);
	else
		d |= pio;

	ds->reset();
	ds->select(addr);
	ds->write (0x5A);
	ds->write (d);
	ds->write (0xFF & ~(d));
	delayMicroseconds (100);
	d = ds->read();
	if (d != 0xAA) {
		Serial.println(F("data write error"));
	}

	return d;
}

/*
switch table: input dev/pin to ouput bus/dev/pin
output may have funcs like timer to switch off
*/
bool alarm_handler()
{
	byte addr[8];
	byte j = 0;
	uint8_t i, latch, data[8], retry = 5;

	ds.reset_search();
	while (ds.search(addr, false)) {
		Serial.print("#");
		Serial.print(j++);
		Serial.print(":");
		for (i = 0; i < 8; i++) {
			Serial.write(' ');
			Serial.print(addr[i], HEX);
		}
		Serial.println();
		if ((mode & MODE_ALRAM_HANDLING) == 0) {
			// interrupt to host
			digitalWrite (A1, LOW);
			return true;
		}
		latch = 0;
		switch (addr[0]) {
			case 0x29:
				do {
					latch = ds2408DataRead(&ds, addr, data);
					if (latch == 0xAA) {
						latch = data[1];
						break;
					}
				} while (retry-- > 0);
				break;
			case 0x28:
				break;
			case 0x35:
				break;
		}
		if (latch != 0 && latch != 0xff) {
			/*
			Get target switch
			Source: <fam> <2 bit store | 6 bit id> <pins> <version> <serial> <random data ... >

			29 A2 D9 84 0 16 04 4C:8 => 29.A2D984001604
			29 A2 D9 84 0 16 10 B0:1 => 
			*/
			if ((addr[1] & 0x3F) == 0x2 && latch & 0xf) {
				static uint8_t target[8] = { 0x3A, 0x01, 0xDA, 0x84, 0x00, 0x00, 0x05, 0xA3 };
				toggleDs2413 (&ds, target);
			}
			if (addr[1] & 0xA2 && latch & 0xf) {
				retry = 5;
				do {
					latch = ds2408TogglePio(&ds, addr, 1, NULL);
					if (latch == 0xAA) {
						latch = data[1];
						break;
					}
				} while (retry-- > 0);
			}
			if ((addr[1] & 0x3F) == 0x0 && latch & 0xf) {
				// light switch kitchen
				//static uint8_t target[8] = { 0x29, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x04, 0x4C };
				retry = 5;
				do {
					latch = ds2408TogglePio(&ds, addr, 1, NULL);
					if (latch == 0xAA) {
						latch = data[1];
						break;
					}
				} while (retry-- > 0);
			}
		}
	}
	if (j) {
		Serial.print(j);
		Serial.println(" alarms");
	}
	return j > 0 ? true : false;
}
