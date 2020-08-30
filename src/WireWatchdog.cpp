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

extern void ow_monitor();
extern byte alarmSignal, wdFired, ledOn;
extern unsigned long ledOnTime;
extern unsigned long wdTime;

void (*WireWatchdog::user_onFired)(void);
void (*WireWatchdog::user_onAlarm)(void);

WireWatchdog::WireWatchdog(byte owPin)
{
	mask_ = PIN_TO_BITMASK(owPin);
	reg_ = PIN_TO_BASEREG(owPin);
}

void WireWatchdog::begin()
{
}

void WireWatchdog::end()
{
}

void WireWatchdog::onFired( void (*function)(void) )
{
  user_onFired = function;
}

void WireWatchdog::onAlarm( void (*function)(void) )
{
  user_onAlarm = function;
}


void WireWatchdog::loop()
{
}

bool WireWatchdog::lineRead()
{
	return DIRECT_READ(reg_, mask_);
}
