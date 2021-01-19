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

extern struct _sw_tbl sw_tbl[MAX_SWITCHES];
extern struct _sw_tbl timed_tbl[MAX_TIMED_SWITCH];
extern struct _dim_tbl dim_tbl[MAX_DIMMER];

void testSetup() {
	Serial.begin(115200);

	Serial.print(F("One Wire Control..."));
	swHdl.begin(ds);

	PCMSK1 |= (_BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11));
    PCIFR |= _BV(PCIF1); // clear any outstanding interrupt
    PCICR |= _BV(PCIE1); // enable interrupt for the group
}

void testLoop()
{

	swHdl.loop();
}

int MainTest()
{
	static const byte latch = 6;
	union d_adr dst;

	testSetup();

	dim_tbl[1].dst.da.bus = 2;
	dim_tbl[1].dst.da.adr = 7;
	dim_tbl[1].dst.da.pio = 0;
	dim_tbl[0].dst.da.bus = 3;
	dim_tbl[0].dst.da.adr = 3;
	dim_tbl[0].dst.da.pio = 0;
	dst.da.bus = 2;
	dst.da.adr = 7;
	dst.da.pio = 0;
	swHdl.switchLevel(dst, 50);

	timed_tbl[0].src.data = 0;
	timed_tbl[0].src.sa.bus = 1;
	timed_tbl[0].src.sa.adr = 3;
	timed_tbl[0].src.sa.latch = latch;
	timed_tbl[0].dst.data = 0;
	timed_tbl[0].dst.da.adr = 7;
	timed_tbl[0].dst.da.bus = 2;
	timed_tbl[0].dst.da.pio = 1;
	dim_tbl[0].dst.data = timed_tbl[0].dst.data;
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	swHdl.switchHandle(1, 3, (1 << (latch - 1)), (320 / 32), MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH);
	/*
	union d_adr dst;
	dst.da.bus = 1;
	dst.da.adr = 2;
	swHdl.timerUpdate(dst, 1);
	*/
	do  {
		testLoop();
	} while(1);

	return 0;
}

int main()
{
	int ret = 0;

	ret = MainTest();

	return ret;
}
#endif /* AVRSIM */