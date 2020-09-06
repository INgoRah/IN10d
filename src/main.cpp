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
#include "TwiHost.h"
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
#define MAX_CMD_BUFSIZE 64

#define MAX_BUS 3
#define MAX_SWITCHES 4 * 12

byte sw_tbl[MAX_BUS][MAX_SWITCHES][2] = { {
{ 0, 0x0 },
{ 0, 0x0 },
{ 0, 0x0 }
} };

byte busPtr[MAX_BUS];

extern byte mode;

/*
 * Objects
 */
DS2482 ds1(0);

WireWatchdog wdt0(A0);
WireWatchdog wdt1(A1);
WireWatchdog wdt2(A2);
WireWatchdog* wdt[MAX_BUS] = { &wdt0, &wdt1, &wdt2 };

TwiHost host(0x2f);
OneWireBase *ds = &ds1;
//OneWireBase* bus[3] = { &ds1, &ds1, &ds1 };
CmdCli cli;
OwDevices ow;

/*
 * Local variables
 */
byte alarmSignal, wdFired, ledOn = 0;
unsigned long ledOnTime = 0;
unsigned long wdTime = 0;
static unsigned long alarmPolling = 0;
static int id;
uint8_t *hostData = NULL;

/*
* Function declarations
*/
bool alarm_handler(byte busNr);

void wdtAlarm() 
{
	Serial.println(F("Alarm test"));
}

void hostCommand(uint8_t cmd, uint8_t data)
{
	uint8_t cnt, adr[8], j;
	//static uint8_t buf[3 * 8];
#define MAX_DATA (15 * 7)

	switch (cmd)
	{
	case 0x5A:
		hostData = (uint8_t*)malloc(MAX_DATA);
		//hostData = buf;
		host.setStatus(0x01);
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
			host.setData(hostData, cnt);
			host.setStatus(0x0);
			Serial.print(j-1);
			Serial.println(" sensors found");
		}
		else
			host.setStatus(0x80);
		break;
	case 0x5B:
		free(hostData);
		/*
		host.setStatus(0x01);
		Serial.print("searching ... ");
		if (ds->search(&adr[1])) {
			host.setStatus(0);
			id++;
			adr[0] = id;
			host.setData(adr, 9);
			Serial.println(id);
		} else {
			host.setStatus(0x80);
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

void initSwTable()
{
	uint16_t len;
	uint8_t vers;

	vers = eeprom_read_byte((const uint8_t*)0);
	len = eeprom_read_word((const uint16_t*)2);
	if (len != 0xFFFF && vers != 0xff) {
		Serial.print("vers=");
		Serial.print(vers);
		Serial.print(" len=");
		Serial.print(len);
		Serial.print(" tbl=");
		Serial.println(sizeof(sw_tbl));
	}
	//if (len <= sizeof(sw_tbl))
		eeprom_read_block((void*)sw_tbl, (const void*)4, len);
}

void setup() {
	ADCSRA = 0;	// disable ADC
	Serial.begin(115200);

	Serial.print(F("One Wire Control..."));
/*	digitalWrite(A1, HIGH);
	pinMode(A1, OUTPUT);
	digitalWrite(A3, LOW);
	pinMode(A3, OUTPUT); */
	delay (100);
	initSwTable();
	ds->resetDev();
	ds->configureDev(DS2482_CONFIG_APU); 
	delay (10);
	// wdt.onAlarm(wdtAlarm);
	host.onCommand(hostCommand);
	host.begin();
	wdFired = 0;

	ow.begin();
	delay (100);
	PCMSK1 |= (_BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11));
    PCIFR |= _BV(PCIF1); // clear any outstanding interrupt
    PCICR |= _BV(PCIE1); // enable interrupt for the group	
	delay (100);
	wdTime = millis();
	alarmPolling = millis();
	Serial.println(F("active"));
	cli.begin(&ow);
}

/* interrupt handling for change on 1-wire */
ISR (PCINT1_vect) // handle pin change interrupt for A0 to A4 here
{
	uint8_t i;

	for (i = 0; i < MAX_BUS; i++) {
		if (wdt[i]->alarmCheck()) {
			alarmSignal++;
			ledOnTime = millis();
			LED_ON();
		}
	}
	wdTime = millis();
#if 0
	unsigned long now = micros();
	bool p = wdt.lineRead();
	if (p) {
		unsigned long duration = now - owLowStart;
		digitalWrite(3, 1);
		wdTime = millis();
		if (owLowStart == 0)
			return;
		owLowStart = 0;
		if (duration > 800) {
			if (alarmSignal > 0)
				return; 
			LED_ON();
			alarmSignal++;
			Serial.write("!");
			ledOnTime = millis();
			return;
		}
		if (duration > 300) {
			// reset
		}
	} else {
		digitalWrite(3, 0);
		// went down, log (reduce by some overhead)
		owLowStart = now - 50;
	}
#endif
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
	host.loop();
	if (ledOnTime != 0 && !alarmSignal) {
		ledBlink();
	}
	if (alarmSignal) {
		Serial.println(F("Alarms signal "));
#if 0
		// is this needed? Line was high when
		// alarm is set
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
#endif
		// interrupt to host
		digitalWrite (HOST_ALRM_PIN, LOW);
		wdTime = millis();
		alarmPolling = millis();
		if (mode & MODE_ALRAM_HANDLING) {
			int retry;
			for (int i = 0; i < MAX_BUS; i++) {
				if (wdt[i]->alarm) {
					retry = 5;
					while (!alarm_handler(i)) {
						if (retry-- == 0)
							break;
					}
					wdt[i]->alarm = false;
				}
			}
		}
		alarmSignal--;
		// ds->reset();
	}
	if (millis() - alarmPolling > 2000) {
		alarmPolling = millis();
		if (mode & MODE_ALRAM_POLLING) {
			for (int i = 0; i < MAX_BUS;i++)
				alarm_handler(i);
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

bool switchHandle(uint8_t busNr, uint8_t adr1, uint8_t latch)
{
	union s_adr src;
	union d_adr dst;
	int retry;

	// busNr should be == addr[2]
	Serial.print(busNr, HEX);
	Serial.print(".");
	src.sa.adr =  adr1;
	// first two latches are output
	// needs special handling table
	src.sa.latch = latch >> 2;
	// todo: check for only one bit !
	Serial.print(src.data, HEX);
	dst.da.bus = busNr;
	dst.da.type = 0;
	dst.da.adr = adr1;
	dst.da.pio = 0;
	Serial.print(" [");
	Serial.print(dst.data, HEX);
	Serial.print("/");
	dst.da.pio = 1;
	Serial.print(dst.data, HEX);
	if (latch < 4) {
		Serial.println("] Notify");
		// hmmm...let switch something else also?? shift >> 2 not working!!
		return true;
	}
	Serial.println("]");
	/* Timed handler: reset or start timer 
	 * based on PIR or standard light (bath)
	 * PIR needs 1 min of initialization, ignore after start
	 * Single trigger mode for 3 secs
	bool handled;??
	for (uint8_t i = 0; i < MAX_SIGNALS; i++) {
		if (latch > 0 && src.data == sig_tbl[busNr][i][0]) {
			check polarity and ignore falling edge?
			High when motion is detected. Check before switching off
		}
	}
	 */
	/*
	 * Standard handler: from 5 bit adr (1..1f) and 3 bit latch (1..7)
	 * Get target switch
	 * Source: <fam> <2 bit store | 6 bit id> <pins> <version> <serial> <random data ... >

	29 A2 D9 84 0 16 04 4C:8 => 29.A2D984001604
	29 A2 D9 84 0 16 10 B0:1 => 
	*/
	for (uint8_t i = 0; i < MAX_SWITCHES; i++) {
		if (latch > 0x3 && src.data == sw_tbl[busNr][i][0]) {
			byte adr[8];

			dst.data = sw_tbl[busNr][i][1];
			Serial.print(F("switch#"));
			Serial.print(i);
			Serial.print(" ");
			Serial.print(src.data, HEX);
			Serial.print(" -> ");
			Serial.print(dst.data, HEX);
			Serial.print(" adr=");
			Serial.print(dst.da.adr, HEX);
			Serial.print(" pio=");
			Serial.println(dst.da.pio + 1, HEX);
			// type 0:
			ow.adrGen(ds, dst.da.bus, adr, dst.da.adr);
			Serial.print(" target=");
			for (i = 0; i < 7; i++) {
				Serial.write(' ');
				Serial.print(adr[i], HEX);
			}
			Serial.write(' ');
			Serial.println(adr[i], HEX);
#if 0
	// type 1:
		static uint8_t target[8] = { 0x3A, 0x01, 0xDA, 0x84, 0x00, 0x00, 0x05, 0xA3 };
		ow.toggleDs2413 (bus[0], target);
#endif			
			retry = 5;
			do {
				ds->selectChannel(dst.da.bus);
				latch = ow.ds2408TogglePio(ds, dst.da.bus, adr, dst.da.pio + 1, NULL);
				if (latch == 0xAA) {
					break;
				}
			} while (retry-- > 0);
			if (latch != 0xAA)
				return false;
		}
	}

	return true;
}

/*
switch table: input dev/pin to ouput bus/dev/pin
output may have funcs like timer to switch off
*/
bool alarm_handler(byte busNr)
{
	//OneWireBase *ds;
	byte addr[8];
	byte j = 0;
	uint8_t i, latch, data[10], retry = 5;
	uint8_t cnt = 10;

	//ds = bus[busNr];
	ds->selectChannel(busNr);
	ds->reset();
	ds->reset_search();
	while (ds->search(addr, false)) {
		j++;
		Serial.print(busNr);
		Serial.print(F("@"));
		for (i = 0; i < 8; i++) {
			if (addr[i] < 0x10)
				Serial.print(F("0"));
			Serial.print(addr[i], HEX);
		}
		if ((mode & MODE_ALRAM_HANDLING) == 0) {
			// interrupt to host
			digitalWrite (A1, LOW);
			return true;
		}
		latch = 0;
		busNr = addr[2];
		switch (addr[0]) {
			case 0x29:
				retry = 5;
				do {
					latch = ow.ds2408RegRead(ds, busNr, addr, data);
					if (latch == 0xAA) {
						latch = data[2];
						break;
					}
				} while (retry-- > 0);
				Serial.print(" (");
				Serial.print(data[1], HEX);
				Serial.print(" ");
				Serial.print(data[2], HEX);
				Serial.print(") | ");
				break;
			case 0x28:
				ds->reset();
				ds->select(addr);
				ds->write(0xBE);  // Read Scratchpad
				// we need 9 bytes
				for (i = 0; i < 9; i++)
			    	latch = ds->read();
				Serial.println();
				break;
			case 0x35:
				break;
		}
		if (latch != 0 && latch != 0xff) {
			switchHandle(busNr, addr[1], latch);
		} else {
			return false;
		}
		if (cnt-- == 0) {
			Serial.println("Error searching");
			break;
		}
	}

	return j > 0 ? true : false;
}
