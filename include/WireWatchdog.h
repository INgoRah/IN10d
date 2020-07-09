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

#define DS2482_CHANNEL_SELECTION_REGISTER    0xD2	/* DS2482-800 only */
#define DS2482_STATUS_REGISTER         0xE1
#define DS2482_ALARM_STATUS_REGISTER         0xA8
#define DS2482_MODE_REGISTER         0x69
#define DS2482_DATA_REGISTER 0xA9

class WireWatchdog
{
	private:
		static byte owPin;
		byte slaveAdr;
		volatile IO_REG_TYPE *reg_;
		IO_REG_TYPE mask_;

		static void (*user_onFired)(void);
		static void (*user_onAlarm)(void);
		static void (*user_onAlarmClear)(void);
		static void (*user_onCommand)(uint8_t cmd, uint8_t data);
		static void receiveEvent(int howMany);
		static void requestEvent();

public:
	WireWatchdog(byte owPin, byte slaveAdr);
	static void setReg(uint8_t _reg);
	static void setStatus(uint8_t stat);
	static void setData(uint8_t *data, uint8_t len);
	void begin();
	void end();
	void loop();
	void onFired( void (*)(void) );
	void onAlarm( void (*)(void) );
	void onCommand( void (*)(uint8_t, uint8_t) );
	void onAlarmClear( void (*)(void) );
	bool lineRead();
};

void wdtBegin();
void wdtLoop();
bool wdtLineRead();

#endif