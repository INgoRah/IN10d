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
#include "CircularBuffer.h"
#include <DallasTemperature.h>

#define DEBUG
/*
 * Local constants
 */

#define LED_ON() digitalWrite(13, 1); \
					ledOn = 1;
#define LED_OFF() digitalWrite(13, 0); \
					ledOn = 0;
#define MAX_CMD_BUFSIZE 64

struct _sw_tbl1 sw_tbl1[MAX_BUS][MAX_SWITCHES];

struct _sw_tbl sw_tbl[MAX_BUS * MAX_SWITCHES];

struct _sw_tbl time_tbl[MAX_TIMER];

extern byte mode;

/*
 * Objects
 */
DS2482 ds1(0);

WireWatchdog wdt0(A0);
WireWatchdog wdt1(A1);
WireWatchdog wdt2(A2);
WireWatchdog wdt3(A3);
WireWatchdog* wdt[MAX_BUS] = { &wdt0, &wdt1, &wdt2, &wdt3 };

TwiHost host(0x2f);
OneWireBase *ds = &ds1;
//OneWireBase* bus[3] = { &ds1, &ds1, &ds1 };
CmdCli cli;
OwDevices ow;
CircularBuffer<uint16_t, 10> events;

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
void temp_read(byte busNr, byte addr[8]);

void wdtAlarm() 
{
	Serial.println(F("Alarm test"));
}

void hostCommand(uint8_t cmd, uint8_t data)
{
	uint8_t cnt, adr[8], j;
	static uint8_t evt_data[2];
	uint16_t d;
	//static uint8_t buf[3 * 8];
#define MAX_DATA (15 * 7)

	switch (cmd)
	{
	case 0x01:
		host.setStatus(STAT_BUSY);
		if (events.size() > 0) {
			d = events.pop();
			evt_data[0] = (d & 0xff00) >> 8;
			evt_data[1] = d & 0xff;
			host.setData((uint8_t*)&evt_data, 2);
			/*Serial.print(evt_data[1], HEX);
			Serial.print(" ");
			Serial.println(evt_data[2], HEX);*/
			host.setStatus(STAT_OK);
		} else
			host.setStatus(STAT_NO_DATA);
		break;
	case 0x5A:
		hostData = (uint8_t*)malloc(MAX_DATA);
		//hostData = buf;
		host.setStatus(STAT_BUSY);
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
			host.setStatus(STAT_OK);
			Serial.print(j-1);
			Serial.println(" sensors found");
		}
		else
			host.setStatus(STAT_NO_DATA);
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
	case 0x02:
	{
		int i;
		uint8_t bus;
		union s_adr src;
		union d_adr dst;
		
		host.setStatus(STAT_BUSY);
		while (!Wire.available());
		dst.da.type = Wire.read();
		while (!Wire.available());
		src.sa.bus = Wire.read(); 
		while (!Wire.available());
		src.sa.adr = Wire.read(); 
		while (!Wire.available());
		src.sa.latch = Wire.read();
		while (!Wire.available());
		dst.da.bus = Wire.read();
		while (!Wire.available());
		dst.da.adr = Wire.read();
		while (!Wire.available());
		dst.da.pio = Wire.read();
		for (i = 0; i < (MAX_BUS * MAX_SWITCHES); i++) {
			if (sw_tbl[i].src.data == 0 || sw_tbl[i].src.data == src.data) {
				sw_tbl[i].src.data = src.data;
				sw_tbl[i].dst.data = dst.data;
				break;
			}
		}
		Serial.print(bus);
		Serial.print(".");
		Serial.print(src.sa.adr);
		Serial.print(".");
		Serial.print(src.sa.latch);
		Serial.print(" -> ");
		Serial.print(dst.da.bus);
		Serial.print(".");
		Serial.print(dst.da.adr);
		Serial.print(".");
		Serial.println(dst.da.pio);
		host.setStatus(STAT_OK);

		break;
	}
	default:
		Serial.print("unknown ");
		Serial.println(cmd, HEX);
		break;
	}
}

int convertSwTbl1to2()
{
	byte j, i;
	union s_adr src;
	union s_adr1 src1;
	union d_adr dst;
	int size = 0, cnt = 0;

	for (j = 0; j < MAX_BUS; j++) {
		for (i = 0; i < MAX_SWITCHES; i++) {
			src1.data = sw_tbl1[j][i].src.data;
			dst.data = sw_tbl1[j][i].dst.data;
			if (src1.data == 0)
				continue;
			if (src1.data == 0xff && dst.data == 0xff)
				continue;
			size += sizeof(union s_adr) + sizeof(union d_adr);
			src.data = 0;
			src.sa.bus = j;
			/* old format didn't support long press */
			src.sa.press = 0;
			src.sa.adr = src1.sa.adr;
			src.sa.latch = src1.sa.latch;
			sw_tbl[cnt].src.data = src.data;
			sw_tbl[cnt].dst.data = dst.data;
			cnt++;
		}
	}

	return size;
}

void initSwTable()
{
	uint16_t len, pos;
	uint8_t vers;

	vers = eeprom_read_byte((const uint8_t*)0);
	len = eeprom_read_word((const uint16_t*)2);
	pos = 4;
	if (len != 0xFFFF && vers != 0xff) {
		Serial.print("vers=");
		Serial.print(vers);
		Serial.print(" len=");
		Serial.print(len);
		switch (vers) {
			case 1:
				Serial.print(" tbl=");
				Serial.println(sizeof(sw_tbl));
				len = sizeof(sw_tbl1);
				eeprom_read_block((void*)sw_tbl1, (const void*)pos, len);
				len = convertSwTbl1to2();
				Serial.print(" sw_tbl2 len=");
				Serial.println(len);
				break;
			case 2:
				eeprom_read_block((void*)sw_tbl, (const void*)pos, len);
				break;
		}
	}
	pos += len;
	vers = eeprom_read_byte((const uint8_t*)pos);
	len = eeprom_read_word((const uint16_t*)pos + 2);
	pos += 4;
	if (len != 0xFFFF && vers != 0xff) {
		Serial.print("special vers=");
		Serial.print(vers);
		Serial.print(" len=");
		Serial.print(len);
	}
}

void setup() {
	unsigned int Ctemp;

	ADCSRA = 0;	// disable ADC
	Serial.begin(115200);

	Serial.print(F("One Wire Control..."));
	digitalWrite(HOST_ALRM_PIN, HIGH);
/*	digitalWrite(A1, HIGH);
	pinMode(A1, OUTPUT);
	digitalWrite(A3, LOW);
	pinMode(A3, OUTPUT); */

	delay (100);
	initSwTable();
	delay (10);
	// wdt.onAlarm(wdtAlarm);
	host.onCommand(hostCommand);
	host.begin();
	wdFired = 0;

	ow.begin(ds);
	delay (100);
	PCMSK1 |= (_BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11));
    PCIFR |= _BV(PCIF1); // clear any outstanding interrupt
    PCICR |= _BV(PCIE1); // enable interrupt for the group	
	delay (100);
	wdTime = millis();
	alarmPolling = millis();
	Serial.println(F("active"));
	cli.begin(&ow);
    /* Setup ADC to use int 1.1V reference 
    and select temp sensor channel */
    ADMUX = (1<<REFS1) | (1<<REFS0) | (1<<MUX3);
    /* Set conversion time to 
    112usec = [(1/(8Mhz / 64)) * (14 ADC clocks  per conversion)]
     and enable the ADC*/
    ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADEN);
	delay (10);
    /* Perform Dummy Conversion to complete ADC init */
    ADCSRA |= (1<<ADSC);
	/* wait for conversion to complete */
	while ((ADCSRA & (1<<ADSC)) != 0)	;

	ADCSRA |= (1<<ADSC);
	/* wait for conversion to complete */
	while ((ADCSRA & (1<<ADSC)) != 0)	;

	Ctemp = ADC;
	Serial.print (Ctemp, HEX);
	Serial.print(" ");
	/* 166 = 25 ?
	   164 = 22?
	   162 = ??
	   162 = 21.5
	   161 = 21.5
	*/
	Serial.print (Ctemp * 0.89286 - 297.42);
	Serial.println(" C");
	ADCSRA = 0;	// disable ADC
#if 1
	uint8_t adr[8] = { 0x28, 0x65, 0x0E, 0xFD, 0x05, 0x00, 0x00, 0x4D };
	ow.tempRead(ds, 0, adr);
	delay(800);
	float temp = ow.tempRead(ds, 0, adr);
	Serial.print(F("Temperature: "));
	Serial.println(temp);
#endif
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
	/* */
	wdTime = millis();
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
		//Serial.println(F("Alarms signal "));
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
	// timer fires every micro ... :-( - maybe hold it?
	sleep_cpu();
	sleep_disable();
}

uint8_t bitnumber(uint8_t bitmap)
{
	int i, res = 1;

	for (i = 0x1; i <= 0x80; i = i << 1) {
		if (bitmap & i)
			return res;
		res++;
	}
	/* invalid */
	return 0xff;
}

bool switchHandle(uint8_t busNr, uint8_t adr1, uint8_t latch, uint8_t press)
{
	union s_adr src;
	union d_adr dst;
	int retry;

	// busNr should be == addr[2]

	src.data = 0;
	src.sa.bus = busNr;
	src.sa.adr =  adr1 & 0x3f;
	Serial.print(src.sa.bus, HEX);
	Serial.print(".");
	Serial.print(src.sa.adr, HEX);
	Serial.print(".");
	/* first two latches are usually output
	 * signaled from auto switch. The corresponding latch
	 * is not signaled!
	 * */
	src.sa.latch = bitnumber(latch);
	Serial.print(src.sa.latch, HEX);
	if (press == 0) {
		src.sa.press = 2;
		Serial.println(F(" pressing"));
		// find free timer and start it
		// tmr.src = src...
		tmr[0] = millis();
		lvl_tbl[0].level = 0;
		lvl_tbl[0].dst.da.bus = 2;
		lvl_tbl[0].dst.da.adr = 3;
	} else if (press > (500 / 32)) {
		src.sa.press = 1;
		Serial.print(F(" time="));
		Serial.println(press * 8);
		tmr[0] = 0;
	} else {
		src.sa.press = 0;
		Serial.println();
	}
	Serial.println(src.data, HEX);

	// put into fifo
	events.push(src.data);
	// todo: signal alarm only here once the data is available
	if ((mode & MODE_AUTO_SWITCH) == 0)
		return false;
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
	 * Standard handler: from 5 bit adr (1..1f) and 3 bit latch (0..7)
	 * Get target switch
	 */
	for (uint8_t i = 0; i < (MAX_BUS * MAX_SWITCHES); i++) {
		if (src.data == sw_tbl[i].src.data) {
			byte adr[8];

			dst.data = sw_tbl[i].dst.data;
#ifdef DEBUG
			Serial.print(F("switch #"));
			Serial.print(i);
			Serial.print(" ");
			Serial.print(src.data, HEX);
			Serial.print(" -> ");
			Serial.print(dst.da.bus);
			Serial.print(".");
			Serial.print(dst.da.adr);
			Serial.print(".");
			Serial.println(dst.da.pio + 1);
#endif
			// type 0:
			ow.adrGen(ds, dst.da.bus, adr, dst.da.adr);
#if 0
			Serial.print(" target=");
			for (i = 0; i < 7; i++) {
				Serial.write(' ');
				Serial.print(adr[i], HEX);
			}
			Serial.write(' ');
			Serial.println(adr[i], HEX);
#endif
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
		Serial.print(F(" "));
		if ((mode & MODE_ALRAM_HANDLING) == 0) {
			// interrupt to host
			digitalWrite (HOST_ALRM_PIN, LOW);
			return true;
		}
		latch = 0;
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
#ifdef DEBUG				
				Serial.print(" (");
				Serial.print(data[1], HEX);
				Serial.print(" ");
				Serial.print(data[2], HEX);
				Serial.print(") | ");
#endif				
				break;
			case 0x28:
				ow.tempRead(ds, busNr, addr);
				return true;
			case 0x35:
				break;
		}
		if (latch != 0 && latch != 0xff) {
			switchHandle(busNr, addr[1], latch, data[6]);
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
