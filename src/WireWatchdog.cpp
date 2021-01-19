/*
 * Library classs includes
 */
#include <Wire.h>
#include <WireWatchdog.h>

#if defined(__AVR__)
#define PIN_TO_BASEREG(pin) (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin) (digitalPinToBitMask(pin))
#define DIRECT_READ(base, mask) (((*(base)) & (mask)) ? 1 : 0)
#endif

void (*WireWatchdog::user_onFired)(void);
void (*WireWatchdog::user_onAlarm)(void);

WireWatchdog::WireWatchdog(byte owPin)
{
	mask_ = PIN_TO_BITMASK(owPin);
	reg_ = PIN_TO_BASEREG(owPin);
	owLowStart = 0;
	alarm = false;
	pinMode(owPin, INPUT);
	this->owPin = owPin;
}

void WireWatchdog::onFired( void (*function)(void) )
{
  user_onFired = function;
}

void WireWatchdog::onAlarm( void (*function)(void) )
{
  user_onAlarm = function;
}

bool WireWatchdog::alarmCheck()
{
	bool p = DIRECT_READ(reg_, mask_);

	if (p) {
		unsigned long duration;

		if (owLowStart == 0)
			return false;

		duration = micros() - owLowStart;
		owLowStart = 0;
		if (duration > 800) {
			//Serial.print(owPin - A0);
			alarm = true;
			return true;
		}
		if (duration > 300) {
			// reset
		}
	} else {
		// went down, log (reduce by some overhead)
		owLowStart = micros() - 15;
	}

	return false;
}
