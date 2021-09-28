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
Used pins:
D3 - Relais ouput, negative polarity - active switching GND
D5 - PWM output (dimed LED) via open coollector transistor
D2 - PIR
D6 - used for alarm signal to host (class TwiHost)
D13 - built in LED
A0..A4 - 1-wire monitor
A6 - Light sensor (analog)
*/

/*
 * Local constants
 */
#define LED_ON() digitalWrite(13, 1); \
					ledOn = 1;
#define LED_OFF() digitalWrite(13, 0); \
					ledOn = 0;
#define HOST_SLAVE_ADR 0x2f

byte debug;

/*
 * Objects
 */
TwiHost host;
DS2482 ds1(0);
OwDevices ow;
SwitchHandler swHdl (&ow);
OneWireBase *ds = &ds1;

uint8_t sec;
uint8_t min;
uint8_t hour;
uint8_t sun;
uint8_t light;
byte light_sensor = 1;

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
uint8_t *hostBuf = NULL;

/*
* Function declarations
*/
static void ledBlink();
int i2cRead();


/*
* Function definitions
*/
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
	light = 125;
	Serial.print(F("One Wire Control"));
	digitalWrite(3, 1);
	delay (10);
	// host.onCommand(hostCommand);
	host.begin (HOST_SLAVE_ADR);
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

static void ledBlink()
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
				uint16_t t;

				ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADEN);
				_delay_us (100);
				t = analogRead(A6);
				ADCSRA = 0;
				t = (t >> 2) & 0xff;
				if (light != t) {
					light = t;
					host.addEvent (TYPE_BRIGHTNESS, 0, 9, light);
				}
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
		swHdl.switchHandle(0, 9, 1);
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
		host.setAlarm();
		if (swHdl.mode & MODE_ALRAM_HANDLING) {
			byte retry;
			for (byte i = 0; i < MAX_BUS; i++) {
				if (wdt[i]->alarm) {
					retry = 5;
					while (!swHdl.alarmHandler(i)) {
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
		if (swHdl.mode & MODE_ALRAM_POLLING) {
			for (byte i = 0; i < MAX_BUS;i++)
				swHdl.alarmHandler(i);
		}
	}
#if 0
	if (swHdl.mode & MODE_WATCHDOG && (wdFired == 0 && millis() - wdTime > 5000)) {
		wdFired = 1;
		Serial.println(F("Watchdog!"));
		//LED_ON();
		ledOnTime = millis();
	}
#endif
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