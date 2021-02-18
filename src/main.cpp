/* LICENSE
 *
 */
/*
 * Includes
 */
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#define wdr wdt_reset

/*
 * Library classs includes
 */
#include <Wire.h>
#include <OneWireBase.h>
#include <DS2482.h>
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
uint8_t hostData[12];

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
uint8_t sun;
uint16_t light;
byte light_sensor = 0;

#if !defined(AVRSIM)

WireWatchdog wdt0(A0);
WireWatchdog wdt1(A1);
WireWatchdog wdt2(A2);
WireWatchdog wdt3(A3);
WireWatchdog* wdt[MAX_BUS] = { &wdt0, &wdt1, &wdt2/*, &wdt3*/ };

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
uint8_t *hostBuf = NULL;

/*
* Function declarations
*/

void wdtAlarm()
{
}

extern struct _sw_tbl sw_tbl[MAX_SWITCHES];

int i2cRead()
{
	int cnt = 1000;

	while (cnt-- && !Wire.available())
		delayMicroseconds(1);
	if (cnt == 0)
		return -1;

	return Wire.read();
}

#define I2C_READ(arg) \
		do { \
			int d; \
			d = i2cRead(); \
			if (arg == -1) { \
				host.setStatus(STAT_FAIL); \
				return; \
			} \
			arg = d; \
		} while(0)

void hostCommand(uint8_t cmd, uint8_t data)
{
	uint8_t cnt, adr[8], bus;
#define MAX_DATA (15 * 7)

	switch (cmd)
	{
	case 0x5A:
		hostBuf = (uint8_t*)malloc(MAX_DATA);
		host.setStatus(STAT_BUSY);
		id = 0;
		ds->reset_search();
		if (ds->reset()) {
			cnt = 0;
			while (ds->search(adr)) {
				memcpy (&hostBuf[cnt], adr, 7);
				cnt += 7;
				// wrap around in case of overflow
				if (cnt > MAX_DATA)
					cnt = 0;
			}
			host.setData(hostBuf, cnt);
			host.setStatus(STAT_OK);
		}
		else
			host.setStatus(STAT_NO_DATA);
		break;
	case 0x5B:
		free(hostBuf);
		break;
	case 0x4B:
		break;
	case 0x02:
	{
		/* programming switch table */
		int i;
		union s_adr src;
		union d_adr dst;

		host.setStatus(STAT_BUSY);
		I2C_READ(dst.da.type);
		I2C_READ(src.sa.bus);
		I2C_READ(src.sa.adr);
		I2C_READ(src.sa.latch);
		I2C_READ(src.sa.press);
		I2C_READ(dst.da.bus);
		I2C_READ(dst.da.adr);
		I2C_READ(dst.da.pio);
		for (i = 0; i < MAX_SWITCHES; i++) {
			if (sw_tbl[i].src.data == 0 || sw_tbl[i].src.data == src.data) {
				sw_tbl[i].src.data = src.data;
				sw_tbl[i].dst.data = dst.data;
				break;
			}
		}
		host.setStatus(STAT_OK);
		break;
	}
	case 0x04:
	{
		int i;

		host.setStatus(STAT_BUSY);
		I2C_READ(bus);
		I2C_READ(adr[1]);
		Serial.print(bus);
		Serial.print(F("."));
		Serial.print(adr[1]);
		ow.adrGen (ds, bus, adr, adr[1]);
		/* here we change from slave to master
		* Would be nice to have a signal (GPIO) to signal
		* usage of the I2C */
		ow.ds2408RegRead(ds, bus, adr, hostData, false);
		for (i = 0; i < 9; i++) {
			Serial.print(hostData[i], HEX);
			Serial.print(F(" "));
		}
		Serial.println(hostData[i], HEX);
#if 0
		hostData[0] = d[0]; // PIO Logic State
		hostData[1] = d[1]; // Output latch
		hostData[2] = d[2]; // Activity latch state
		hostData[3] = d[5]; // Status
		hostData[4] = d[6]; // Status ext 1
		hostData[5] = d[7]; // Status ext 2
#endif
		host.setData((uint8_t*)hostData, 6);
		host.setStatus(STAT_OK);
		break;
	}
	default:
		break;
	}
}

void setup() {
	uint8_t mcusr_old;

	/* the watchdog timer remains active even after a system reset (except a
	 * power-on condition), using the fastest prescaler value.
	 * It is therefore required to turn off the watchdog early
     * during program startup */
	mcusr_old = MCUSR;
	MCUSR = 0;
	wdt_disable();
	if (mcusr_old & _BV(WDRF)) {
		/* Watchdog occured */
		Serial.println(F("Watchdog fired!"));
	}
	//ADCSRA = 0;	// disable ADC
	Serial.begin(115200);

	debug = 1;
	light = 700;
	Serial.print(F("One Wire Control"));
	digitalWrite(HOST_ALRM_PIN, HIGH);
	pinMode(HOST_ALRM_PIN, OUTPUT);
	mode = MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH;

	delay (10);
	// wdt.onAlarm(wdtAlarm);
	host.onCommand(hostCommand);
	host.begin();
	wdFired = 0;

	ow.begin(ds);
	//Wire.setWireTimeout(250, true);
	swHdl.begin(ds);
	PCMSK1 |= (_BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11));
	PCMSK2 |= (_BV(PCINT18) /*| _BV(PCINT19) | _BV(PCINT20) | _BV(PCINT21)*/);
	PCIFR |= _BV(PCIF1) | _BV(PCIF2); // clear any outstanding interrupt
	PCICR |= _BV(PCIE1) | _BV(PCIE2); // enable interrupt for the group

	delay (50);
	wdTime = millis();
	alarmPolling = millis();
#ifdef CLI_SUPPORT
	cli.begin(&ow);
#endif
#if 0
	unsigned int Ctemp;
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
#endif
	wdt_enable(WDTO_2S);
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
	if (PIND & _BV(PD2))
		pinSignal |= 1;
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
#if 0
	// was not working
	if (Wire.getWireTimeoutFlag()) {
		Wire.clearWireTimeoutFlag();
		Serial.println(F("I2C timeout detected!"));
	}
#endif
	host.loop();
	if (millis() > (sec_time + 995)) {
		sec_time = millis();
		if (sec++ == 60) {
			sec = 0;
			if (light_sensor) {
				ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADEN);
				_delay_us (100);
				light = analogRead(A6);
				ADCSRA = 0;
			}
			if (min++ == 60) {

				min = 0;
				if (hour++ == 24)
					hour = 0;
			}
		}
	}
	if (ledOnTime != 0 && !alarmSignal) {
		ledBlink();
	}
	if (pinSignal) {
		// interrupt to host
		digitalWrite (HOST_ALRM_PIN, LOW);
		swHdl.switchHandle(0, 9, 1, mode);
		pinSignal = 0;
	}
	/* if there is any host data transfer, avoid conflicts on the I2C bus
	 * and skip handling - transfer should be finished very quickly
	if (host.getStatus() == STAT_BUSY)
		return;
	 */
	swHdl.loop();
	if (alarmSignal) {
		alarmPolling = millis();
		if (debug > 2) {
			Serial.println(F("Alarm"));
		}
		if (mode & MODE_ALRAM_HANDLING) {
			byte retry;
			for (byte i = 0; i < MAX_BUS; i++) {
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
			for (byte i = 0; i < MAX_BUS;i++)
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
	wdr();
	 /*else
		sleep_cpu();*/
	sleep_enable();
	// timer fires every micro ... :-( - maybe hold it?
	set_sleep_mode(SLEEP_MODE_IDLE);
	// other sleep modes were not working, not woken up...
	sleep_cpu();
	sleep_disable();
	wdr();
}
#endif /* AVRSIM */