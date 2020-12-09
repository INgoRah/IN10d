#ifndef _TWIHOST_H
#define _TWIHOST_H

#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc
#include "CircularBuffer.h"

#define MODE_WATCHDOG 0x1

#define DS2482_CHANNEL_SELECTION_REGISTER    0xD2	/* DS2482-800 only */

#define DS2482_STATUS_REGISTER         0xE1

#define STAT_OK 0x0
#define STAT_BUSY 0x01
#define STAT_NO_DATA 0x80

#define DS2482_ALARM_STATUS_REGISTER         0xA8
#define DS2482_MODE_REGISTER         0x69
#define DS2482_DATA_REGISTER 0xA9

struct logData {
	byte h;
	byte min;
	byte sec;
	uint16_t data;
};

class TwiHost
{
	private:
		byte slaveAdr;

		static void (*user_onCommand)(uint8_t cmd, uint8_t data);
		static void receiveEvent(int howMany);
		static void requestEvent();

public:
	CircularBuffer<struct logData, 20> events;

	TwiHost(byte slaveAdr);
	static void setReg(uint8_t _reg);
	static void setStatus(uint8_t stat);
	static void setData(uint8_t *data, uint8_t len);
	void begin();
	void end();
	void loop();
	void onCommand( void (*)(uint8_t, uint8_t) );
};
#endif