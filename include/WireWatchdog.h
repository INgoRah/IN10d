#ifndef _WIREWATCHDOG_H
#define _WIREWATCHDOG_H

#if ARDUINO >= 100
#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"      // for delayMicroseconds
#include "pins_arduino.h"  // for digitalPinToBitMask, etc
#endif

#if defined(__AVR__)
#define IO_REG_TYPE uint8_t
#endif

#define MODE_WATCHDOG 0x1
#define MODE_ALRAM_POLLING 0x2
#define MODE_ALRAM_HANDLING 0x4

class WireWatchdog
{
	private:
		byte owPin;
		volatile IO_REG_TYPE *reg_;
		IO_REG_TYPE mask_;
		unsigned long owLowStart;

		static void (*user_onFired)(void);
		static void (*user_onAlarm)(void);
		static void (*user_onAlarmClear)(void);

	public:
		WireWatchdog(byte owPin);
		bool alarm;
		void onFired( void (*)(void) );
		void onAlarm( void (*)(void) );
		bool alarmCheck();
};

void wdtBegin();
void wdtLoop();
bool wdtLineRead();

#endif