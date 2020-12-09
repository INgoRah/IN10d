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
#ifdef CLI_SUPPORT
#include "CmdCli.h"
#endif
#include "OwDevices.h"
#include "SwitchHandler.h"  

/*
 * Local constants
 */

#define LED_ON() digitalWrite(13, 1); \
					ledOn = 1;
#define LED_OFF() digitalWrite(13, 0); \
					ledOn = 0;

byte mode;
byte debug;

/*
 * Objects
 */
TwiHost host(0x2f);
DS2482 ds1(0);
OwDevices ow;
SwitchHandler swHdl (&ow);
OneWireBase *ds = &ds1;

uint8_t sec;
uint8_t min;
uint8_t hour;

#if !defined(AVRSIM)

WireWatchdog wdt0(A0);
WireWatchdog wdt1(A1);
WireWatchdog wdt2(A2);
WireWatchdog wdt3(A3);
WireWatchdog* wdt[MAX_BUS] = { &wdt0, &wdt1, &wdt2, &wdt3 };

//OneWireBase* bus[3] = { &ds1, &ds1, &ds1 };
#ifdef CLI_SUPPORT
CmdCli cli;
#endif

/*
 * Local variables
 */
byte alarmSignal, pinSignal, wdFired, ledOn = 0;
unsigned long ledOnTime = 0;
unsigned long wdTime = 0;
static unsigned long alarmPolling = 0;
static int id;
uint8_t *hostData = NULL;

/*
* Function declarations
*/
void temp_read(byte busNr, byte addr[8]);

void wdtAlarm() 
{
	Serial.println(F("Alarm test"));
}

extern struct _sw_tbl sw_tbl[MAX_SWITCHES];

void hostCommand(uint8_t cmd, uint8_t data)
{
	uint8_t cnt, adr[8], j;
	static uint8_t evt_data[2];
	//static uint8_t buf[3 * 8];
#define MAX_DATA (15 * 7)

	switch (cmd)
	{
	case 0x01:
		host.setStatus(STAT_BUSY);
		if (host.events.size() > 0) {
			struct logData d;

			d = host.events.pop();
			evt_data[0] = (d.data & 0xff00) >> 8;
			evt_data[1] = d.data & 0xff;
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
		break;
	case 0x4B:
		break;
	case 0x02:
	{
		int i;
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
		src.sa.press = Wire.read();
		while (!Wire.available());
		dst.da.bus = Wire.read();
		while (!Wire.available());
		dst.da.adr = Wire.read();
		while (!Wire.available());
		dst.da.pio = Wire.read();
		for (i = 0; i < MAX_SWITCHES; i++) {
			if (sw_tbl[i].src.data == 0 || sw_tbl[i].src.data == src.data) {
				sw_tbl[i].src.data = src.data;
				sw_tbl[i].dst.data = dst.data;
				break;
			}
		}
		Serial.print(src.sa.bus);
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
	case 0x03:
		byte bus, pio, level, d;
		byte adr[8];
		
		host.setStatus(STAT_BUSY);
		while (!Wire.available());
		bus = Wire.read();
		while (!Wire.available());
		adr[1] = Wire.read();
		while (!Wire.available());
		// which pio to change
		pio = Wire.read();
		while (!Wire.available());
		// on/off or dim level
		level = Wire.read();
		ow.adrGen(ds, bus, adr, adr[1]);
		//ow.ds2408RegRead(ds, bus, adr, data, false);
		d = ow.ds2408PioRead(ds, bus, adr);
		if (d & pio && level == 0) {
			// off
			d &= ~(pio);
		} else {
			// on
			d |= pio;
			// set level if level 1 .. 16
			// level = 0 is first valid level on my custom ds2408
			if (level > 0 && level <= 16)
				pio = ((level - 1) << 4) | pio;
		}
		ow.ds2408PioSet(ds, bus, adr, pio);
		host.setStatus(STAT_OK);
		break;
	default:
		Serial.print("unknown ");
		Serial.println(cmd, HEX);
		break;
	}
}

void setup() {
	unsigned int Ctemp;

	ADCSRA = 0;	// disable ADC
	Serial.begin(115200);

	debug = 1;
	Serial.print(F("One Wire Control..."));
	digitalWrite(HOST_ALRM_PIN, HIGH);
/*	digitalWrite(A1, HIGH);
	pinMode(A1, OUTPUT);
	digitalWrite(A3, LOW);
	pinMode(A3, OUTPUT); */
	mode = MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH;

	delay (10);
	// wdt.onAlarm(wdtAlarm);
	host.onCommand(hostCommand);
	host.begin();
	wdFired = 0;

	ow.begin(ds);
	swHdl.begin(ds);
	PCMSK1 |= (_BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11));
	PCMSK2 |= (_BV(PCINT18) /*| _BV(PCINT19) | _BV(PCINT20) | _BV(PCINT21)*/);
	PCIFR |= _BV(PCIF1) | _BV(PCIF2); // clear any outstanding interrupt
	PCICR |= _BV(PCIE1) | _BV(PCIE2); // enable interrupt for the group	
	
	delay (50);
	wdTime = millis();
	alarmPolling = millis();
	Serial.println(F("active"));
#ifdef CLI_SUPPORT
	cli.begin(&ow);
#endif
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
	Serial.print (Ctemp * 0.89286 - 284.42);
	Serial.println(" C");
	ADCSRA = 0;	// disable ADC
#if 0
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

/* Connector on D2 (PD2), D3 (PD4) - custom wire D4 (PD5), D5 (PD6) */
ISR (PCINT2_vect) // handle pin change interrupt for A0 to A4 here
{
	if (PIND & _BV(PD2)) {
		Serial.println("PIR!");
		pinSignal |= 1;
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
	static unsigned long sec_time = 0;

	if (millis() > (sec_time + 995)) {
		sec_time = millis();
		if (sec++ == 60) {
			int light = analogRead(A6);
			Serial.print(F("Darknes="));
			Serial.println(light);
			sec = 0;
			if (min++ == 60) {
				
				min = 0;
				if (hour++ == 24)
					hour = 0;
			}
		}
	}
	host.loop();
	if (ledOnTime != 0 && !alarmSignal) {
		ledBlink();
	}
	swHdl.loop();
	if (pinSignal) {
		// interrupt to host
		digitalWrite (HOST_ALRM_PIN, LOW);
		swHdl.switchHandle(3, 9, 1, 0xff, mode);
		pinSignal = 0;
	}
	if (alarmSignal) {
		alarmPolling = millis();
		if (mode & MODE_ALRAM_HANDLING) {
			int retry;
			for (int i = 0; i < MAX_BUS; i++) {
				if (wdt[i]->alarm) {
					retry = 5;
					while (!swHdl.alarmHandler(i, mode)) {
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
				swHdl.alarmHandler(i, mode);
		}
	}
	if (mode & MODE_WATCHDOG && (wdFired == 0 && millis() - wdTime > 5000)) {
		wdFired = 1;
		Serial.println(F("Watchdog!"));
		//LED_ON();
		ledOnTime = millis();
	}
#ifdef CLI_SUPPORT
	cli.loop();
#endif
	 /*else
		sleep_cpu();*/
	sleep_enable();
	set_sleep_mode(SLEEP_MODE_IDLE);
	// timer fires every micro ... :-( - maybe hold it?
	sleep_cpu();
	sleep_disable();
}
#endif /* AVRSIM */