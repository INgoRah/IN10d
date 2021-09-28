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

#if defined(AVRSIM)
/*
 * Library classs includes
 */
#include <Wire.h>
#include <OneWireBase.h>
#include <DS2482.h>
#include "WireWatchdog.h"
#include "TwiHost.h"
#include "CmdCli.h"
#include "OwDevices.h"
#include "SwitchHandler.h"

/*
 * Local constants
 */

/*
 * Objects
 */
extern SwitchHandler swHdl;
extern OneWireBase *ds;

/*
 * Local variables
 */

/*
* Function declarations
*/

void testSetup() {
	Serial.begin(115200);

	Serial.print(F("One Wire Control..."));
	swHdl.begin(ds);
	light = swHdl.light_thr + 1;
	PCMSK1 |= (_BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11));
    PCIFR |= _BV(PCIF1); // clear any outstanding interrupt
    PCICR |= _BV(PCIE1); // enable interrupt for the group
}

void testLoop()
{

	swHdl.loop();
}

void tableSetup()
{
	sw_tbl[0].src.data = 0;
	sw_tbl[0].dst.data = 0;

	// global switch #0: 0, 1, 2 for on/off type
	// dst 1 2 1
	sw_tbl[0].src.sa.bus = 0;
	sw_tbl[0].src.sa.adr = 1;
	sw_tbl[0].src.sa.latch = 2; // 2nd bit = 2
	sw_tbl[0].dst.da.bus = 1;
	sw_tbl[0].dst.da.adr = 2;
	sw_tbl[0].dst.da.pio = 1;
	// global switch #1: 0, 1, 4 for level type
	// dst 3 3 0
	sw_tbl[1].src.data = 0;
	sw_tbl[1].dst.data = 0;
	sw_tbl[1].src.sa.bus = 0;
	sw_tbl[1].src.sa.adr = 1;
	sw_tbl[1].src.sa.latch = 3; // 3rd bit, 1, 2, 4
	sw_tbl[1].dst.da.bus = 3;
	sw_tbl[1].dst.da.adr = 3;
	sw_tbl[1].dst.da.pio = 0;
	// global switch: 0, 2, 2 for on/off type
	// dst 2 1 0
	sw_tbl[2].src.data = 0;
	sw_tbl[2].dst.data = 0;
	sw_tbl[2].src.sa.bus = 0;
	sw_tbl[2].src.sa.adr = 2;
	sw_tbl[2].src.sa.latch = 2; // 3rd bit, 1, 2, 4
	sw_tbl[2].dst.da.bus = 2;
	sw_tbl[2].dst.da.adr = 1;
	sw_tbl[2].dst.da.pio = 0;

	// global timer switch (PIR): 0, 2, 4 for level type
	// dst 3 3 0
	timed_tbl[0].type = TYPE_DARK_SOFT_30S;
	timed_tbl[0].src.data = 0;
	timed_tbl[0].dst.data = 0;
	timed_tbl[0].src.sa.bus = 0;
	timed_tbl[0].src.sa.adr = 2;
	timed_tbl[0].src.sa.latch = 3; // 3rd bit = 4 (1, 2, 4)
	timed_tbl[0].dst.da.bus = 3;
	timed_tbl[0].dst.da.adr = 3;
	timed_tbl[0].dst.da.pio = 0;
	// global timer switch (PIR): 2, 2, 1 for on/off type
	// dst 2 1 0
	timed_tbl[0].type = TYPE_DARK_30S;
	timed_tbl[1].src.data = 0;
	timed_tbl[1].dst.data = 0;
	timed_tbl[1].src.sa.bus = 2;
	timed_tbl[1].src.sa.adr = 2;
	timed_tbl[1].src.sa.latch = 1;
	timed_tbl[1].dst.da.bus = 2;
	timed_tbl[1].dst.da.adr = 1;
	timed_tbl[1].dst.da.pio = 0;
/*
	dim_tbl[2].dst.da.bus = 3;
	dim_tbl[2].dst.da.adr = 3;
	dim_tbl[2].dst.da.pio = 0;
	// optional
	dim_tbl[0].dst.da.bus = 2;
	dim_tbl[0].dst.da.adr = 7;
	dim_tbl[0].dst.da.pio = 0;
*/
}

int MainTest(int test)
{
	static const byte latch = 6;
	uint8_t dadr, dpio;

	testSetup();

	/*
	Case 4:
	switch on + timer on (not switching)
	*/
	if (test == 4) {
		// global switch #2: 0, 2, 2 for on/off type

		// global timer switch #2 (PIR): 2, 2, 1 for on/off type
		// dst 2 1 0
		dadr = sw_tbl[2].dst.da.adr;
		dpio = sw_tbl[2].dst.da.pio;
		pio_data[dadr] = 0xff;
		// switch on
		swHdl.switchHandle(0, 2, 2, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		if (pio_data[dadr] != 0xfe)
			return __LINE__;
		// timer switch (PIR): 0, 2, 4 for level type
		// must be ignored because already switched on
		// no timer started
		swHdl.switchHandle(2, 2, 1, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		// no change!
		if (pio_data[dadr] != 0xfe)
			return __LINE__;
		timer0_millis += DEF_SECS * 1000 + 100;
		swHdl.loop();
		// no change! Timer shouldn't be running
		if (pio_data[dadr] != 0xfe)
			return __LINE__;
		// switch off
		swHdl.switchHandle(0, 2, 2, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		if (pio_data[dadr] != 0xff)
			return __LINE__;
	}
	/*
	Case 5:
	level on + timer on (not switching)
	*/
	if (test == 5) {
		dadr = sw_tbl[1].dst.da.adr;
		dpio = sw_tbl[1].dst.da.pio;
		pio_data[dadr] = 0x0;
		// switch on
		swHdl.switchHandle(0, 1, 4, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		if (pio_data[dadr] != 0x71)
			return __LINE__;
		// no try to trigger the timer
		swHdl.switchHandle(0, 2, 4, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		if (pio_data[dadr] != 0x71)
			return __LINE__;
		timer0_millis += DEF_SECS * 1000 + 1000;
		swHdl.loop();
		if (pio_data[dadr] != 0x71)
			return __LINE__;
		// switch off
		swHdl.switchHandle(0, 1, 4, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		if (pio_data[dadr] != 0xf1)
			return __LINE__;
		swHdl.switchHandle(0, 1, 4, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		if (pio_data[dadr] != 0x00)
			return __LINE__;
	}
	/*
	Case 6:
	timer on and retrigger on / timer off
	*/
	if (test == 6) {
		// timer switch (PIR): 0, 2, 4 for level type
		// dst 3 3 0
		dadr = timed_tbl[0].dst.da.adr;
		dpio = timed_tbl[0].dst.da.pio;
		timed_tbl[0].type = TYPE_DARK_SOFT_30S;
		pio_data[dadr] = 0x0;

		swHdl.switchHandle(0, 2, 4, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		if (pio_data[dadr] != 0xF1)
			return __LINE__;
		timer0_millis += DEF_SECS * 500 + 100;
		swHdl.loop();
		swHdl.switchHandle(0, 2, 4, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		if (pio_data[dadr] != 0xF1)
			return __LINE__;
		timer0_millis += DEF_SECS * 500 + 100;
		swHdl.loop();
		// still on
		if (pio_data[dadr] != 0xF1)
			return __LINE__;

		timer0_millis += DEF_SECS * 500 + 100;
		swHdl.loop();
		if (pio_data[dadr] != 0xF1)
			return __LINE__;
		timer0_millis += DEF_SECS * 500 + 100;
		swHdl.loop();
		// now off
		if (pio_data[dadr] != 0x0)
			return __LINE__;
	}

	if (test == 7) {
		union pio p;

		p.da.bus = 0;
		p.da.adr = 9;
		p.da.pio = 3;
		p.da.type = 2;
		// convert?
		swHdl.switchLevel(p, 100);
		swHdl.switchLevel(p, 0);

	dst.da.bus = 1;
	dst.da.adr = 7;
	dst.da.pio = 1;
	swHdl.switchLevel(dst, 100);
	timed_tbl[0].src.data = 0;
	timed_tbl[0].src.sa.bus = 1;
	timed_tbl[0].src.sa.adr = 3;
	timed_tbl[0].src.sa.latch = latch;
	timed_tbl[0].dst.data = 0;
	timed_tbl[0].dst.da.adr = 7;
	timed_tbl[0].dst.da.bus = 2;
	timed_tbl[0].dst.da.pio = 1;
	dim_tbl[0].dst.data = timed_tbl[0].dst.data;
	/*
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	*/
	}
	if (test == 8) {
		union d_adr_8 dst;
		union pio p;

		p.da.bus = 0;
		p.da.adr = 9;
		p.da.pio = 0;
		p.da.type = 2;
		swHdl.switchLevel(p, 15);

		dst.da.bus = 0;
		dst.da.adr = 9;
		dst.da.pio = 0;
		swHdl.actorHandle(dst, ON);

		swHdl.actorHandle(dst, TOGGLE);
	}
	/*
	Case 9:
	timer on and soft off, level type
	*/
	if (test == 9) {
		// timer switch (PIR): 0, 2, 4 for level type
		// dst 3 3 0
		dadr = timed_tbl[0].dst.da.adr;
		dpio = timed_tbl[0].dst.da.pio;
		timed_tbl[0].type = TYPE_DARK_SOFT_30S;
		pio_data[dadr] = 0x0;

		swHdl.switchHandle(0, 2, 4, MODE_ALRAM_HANDLING | MODE_AUTO_SWITCH);
		if (pio_data[dadr] != 0xF1)
			return __LINE__;
		timer0_millis += 25 * 1000 + 100;
		swHdl.loop(1);
		// 26
		timer0_millis += 1000;
		swHdl.loop(1);
		// 29
		timer0_millis += 3000;
		swHdl.loop(1);
		timer0_millis += 1000;
		// off
		swHdl.loop(1);

		/* this is off!
		timer0_millis += DEF_SECS * 1000 + 100;
		swHdl.loop();
		*/
		if (pio_data[dadr] != 0x0)
			return __LINE__;
	}
	return 0;
}

int main()
{
	int ret = 0, i;
#if 0
	uint8_t bit = digitalPinToBitMask(5);
	Serial.print(bit);
	analogWrite(5, 100);
#endif
	debug = 4;
	light = 220;
	ret = MainTest(9);
	for (i = 1; i < 8; i++) {
		ret = MainTest(i);
		if (ret) {
			do {
				Serial.print(ret);
			} while (ret);
		}
	}
	return ret;
}
#endif /* AVRSIM */