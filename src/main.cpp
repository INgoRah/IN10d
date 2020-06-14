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
#include <OneWireBase.h>
#include <DS2482.h>
#include <OneWire.h>
#include "WireWatchdog.h"
#include "CmdCli.h"
#include "OwDevices.h"

#define DEBUG
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
DS2482 ds1(0);
OneWire ds2(2);
OneWire ds3(ow_pin);
WireWatchdog wdt(ow_pin, 0x2f);
OneWireBase *ds = &ds1;
OneWireBase* bus[3] = { &ds1, &ds2, &ds3 };
CmdCli cli;
OwDevices ow;

/*
 * Local variables
 */
byte alarmSignal, wdFired, ledOn = 0;
unsigned long ledOnTime = 0;
static unsigned long owLowStart = 0;
unsigned long wdTime = 0;
unsigned long timeStamp = 0;
static unsigned long alarmPolling = 0;
static int id;

/*
* Function declarations
*/
void ow_monitor();
bool alarm_handler(OneWireBase *ds);

void wdtAlarm() 
{
	Serial.println(F("Alarm test"));
}

uint8_t *hostData = NULL;

void wdtCommand(uint8_t cmd, uint8_t data)
{
	uint8_t cnt, adr[8], j;
	//static uint8_t buf[3 * 8];
#define MAX_DATA (15 * 7)

	switch (cmd)
	{
	case 0x5A:
		hostData = (uint8_t*)malloc(MAX_DATA);
		//hostData = buf;
		wdt.setStatus(0x01);
		Serial.println("start ... ");
		id = 0;
		ds->reset_search();
		if (ds->reset()) {
			cnt = 0;
			j = 1;
			while (ds->search(adr)) {
				memcpy (&hostData[cnt], adr, 7);
				cnt += 7;
				// wrap around in case of overflow
				if (cnt > MAX_DATA)
					cnt = 0;
				Serial.print("#");
				Serial.print(j++);
				Serial.print(":");
				for (int i = 0; i < 8; i++) {
					Serial.write(' ');
					Serial.print(adr[i], HEX);
				}
				Serial.println();
			}
			wdt.setData(hostData, cnt);
			wdt.setStatus(0x0);
			Serial.print(j-1);
			Serial.println(" sensors found");
		}
		else
			wdt.setStatus(0x80);
		break;
	case 0x5B:
		free(hostData);
		/*
		wdt.setStatus(0x01);
		Serial.print("searching ... ");
		if (ds->search(&adr[1])) {
			wdt.setStatus(0);
			id++;
			adr[0] = id;
			wdt.setData(adr, 9);
			Serial.println(id);
		} else {
			wdt.setStatus(0x80);
			Serial.println("fail");
		}
		*/
		break;
	case 0x4B:
		ow.statusRead(ds);
		break;
	default:
		Serial.print("unknown ");
		Serial.println(cmd, HEX);
		break;
	}
}

void setup() {
	ADCSRA = 0;	// disable ADC
	Serial.begin(115200);

	Serial.print(F("One Wire Control..."));
	digitalWrite(A1, HIGH);
	pinMode(A1, OUTPUT);
	digitalWrite(A3, LOW);
	pinMode(A3, OUTPUT);
	delay (100);
	ds->resetDev();
	ds->configureDev(DS2482_CONFIG_APU); 
	delay (10);
	wdt.onAlarm(wdtAlarm);
	wdt.onCommand(wdtCommand);
	wdt.begin();
	wdFired = 0;

	ow.begin();
	delay (100);
	::attachInterrupt(digitalPinToInterrupt(ow_pin), ow_monitor, CHANGE);
	delay (100);
	digitalWrite(A3, HIGH);
	wdTime = millis();
	alarmPolling = millis();
	Serial.println(F("active"));
	cli.begin(&ow);
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
			int retry = 10;
			while (!alarm_handler(bus[alarmSignal-1])) {
				if (retry-- == 0)
					break;
			};
		}
		alarmSignal = 0;
		// ds->reset();
	}
	if (millis() - alarmPolling > 2000) {
		alarmPolling = millis();
		if (mode & MODE_ALRAM_POLLING) {
			for (int i = 0; i < 2;i++)
				alarm_handler(bus[i]);
		}
	}
	if (mode & MODE_WATCHDOG && (wdFired == 0 && millis() - wdTime > 5000)) {
		wdFired = 1;
		Serial.println(F("Watchdog!"));
		//LED_ON();
		ledOnTime = millis();
	}
	cli.loop();
	 /*else
		sleep_cpu();*/
	sleep_enable();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_cpu();
}

/*
switch table: input dev/pin to ouput bus/dev/pin
output may have funcs like timer to switch off
*/
bool alarm_handler(OneWireBase *ds)
{
	byte addr[8];
	byte j = 0;
	uint8_t i, latch, data[10], retry = 5;
	uint8_t cnt = 10;

	ds->reset();
	ds->reset_search();
	while (ds->search(addr, false)) {
		Serial.print("#");
		Serial.print(j++);
		Serial.print(":");
		for (i = 0; i < 8; i++) {
			Serial.write(' ');
			Serial.print(addr[i], HEX);
		}
		if ((mode & MODE_ALRAM_HANDLING) == 0) {
			// interrupt to host
			digitalWrite (A1, LOW);
			return true;
		}
		latch = 0;
		switch (addr[0]) {
			case 0x29:
				retry = 5;
				do {
					latch = ow.ds2408RegRead(ds, addr, data);
					if (latch == 0xAA) {
						latch = data[2];
						break;
					}
				} while (retry-- > 0);
				Serial.print(" (");
				Serial.print(data[1], HEX);
				Serial.print(" ");
				Serial.print(data[2], HEX);
				Serial.println(")");
				break;
			case 0x28:
				ds->reset();
				ds->select(addr);
				ds->write(0xBE);  // Read Scratchpad
				// we need 9 bytes
				for ( i = 0; i < 9; i++)
			    	latch = ds->read();
				Serial.println();
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
			if ((addr[1] == 0x42) && (latch & 4)) {
				// temp
				static uint8_t target[8] = { 0x3A, 0x01, 0xDA, 0x84, 0x00, 0x00, 0x05, 0xA3 };
				ow.toggleDs2413 (bus[0], target);
			}
			if ((addr[1] & 0x3F) == 1 && latch & 0xf) {
				retry = 5;
				do {
					latch = ow.ds2408TogglePio(ds, addr, 1, data);
					if (latch == 0xAA) {
						break;
					}
				} while (retry-- > 0);
				if (latch != 0xAA)
					return false;
			}
			if ((addr[1] & 0x3F) == 2 && latch == 0x4) {

			}
			if (addr[1] == 0x01 && latch & 0xf) {
				// light switch kitchen
				//static uint8_t target[8] = { 0x29, 0xA2, 0xD9, 0x84, 0x00, 0x16, 0x04, 0x4C };
				retry = 5;
				do {
					latch = ow.ds2408TogglePio(ds, addr, 1, NULL);
					if (latch == 0xAA)
						break;
				} while (retry-- > 0);
			}
		} else {
			return false;
		}
		if (cnt-- == 0) {
			Serial.println("Error searching");
			break;
		}
	}
#ifdef DEBUG	
	if (j) {
		Serial.print(j);
		Serial.println(" alarms");
	} else {/*
		Serial.print("Status=");
		Serial.println(ds->status);
		*/
	}
#endif	
	return j > 0 ? true : false;
}
