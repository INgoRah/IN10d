#ifndef _TWIHOST_H
#define _TWIHOST_H

#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc
#include "CircularBuffer.hpp"

#define MODE_WATCHDOG 0x1
#define HOST_ALRM_PIN 6

#define DS2482_CHANNEL_SELECTION_REGISTER    0xD2	/* DS2482-800 only */

#define DS2482_STATUS_REGISTER         0xE1

#define STAT_OK 	0x0
/* ready to service data */
#define STAT_READY 	0x04
#define STAT_FAIL 	0xCC

#define STAT_POWER_IMP 0x10
#define STAT_START 0x20
#define STAT_EVT  	0x40
#define STAT_NO_DATA 0x80
#define STAT_BUSY 	0xC0

/** Alarm status register read will clear the alarm line, the
 * watchdog and returns the current alarm status
 */
#define DS2482_ALARM_STATUS_REGISTER         0xA8
#define DS2482_MODE_REGISTER         0x69
#define DS2482_DATA_REGISTER 0xA9
#define DS2482_TEST_REGISTER         0x58

#define TYPE_BRIGHTNESS 3

enum {
	SRC_CHANGE = 0,
	DST_CHANGE,
	TEMP_CHANGE,
	BRIGHTNESS_CHANGE,
	DIMMING_DOWN,
	HUMIDITY_CHANGE,
	POWER_IMP,
	SYS_START
};

struct logData {
	byte h;
	byte min;
	byte sec;
	/* 0: source data change
	   1: destination change (lights switched)
	   2: temperature alarm
	   3: brightness
	   4: dimming down */
	uint8_t type;
	uint16_t source;
	uint16_t data;
};

class TwiHost
{
	private:
		uint8_t hostData[9];
		byte rxBytes;
		byte _seq, _ack;
		uint8_t alarmStat;
		static void (*user_onCommand)(uint8_t cmd, uint8_t data);
		void command();
		void commandData();

		static void receiveEvent(int howMany);
		static void requestEvent();

	public:
		CircularBuffer<struct logData, 2> events;
		uint8_t status;

		TwiHost();
		void setStatus(uint8_t stat);
		uint8_t getStatus();
		void setAlarm(uint8_t channel = 0xF);
		void addEvent(uint8_t type, uint16_t source, uint16_t data = 0);
		void addEvent(uint8_t type, uint8_t bus, uint8_t adr, uint16_t data);
		void addEvent(union pio dst, uint16_t data = 0, uint8_t type = DST_CHANGE);
		void addEvent(union d_adr_8 dst, uint16_t data = 0, uint8_t type = DST_CHANGE);
		void begin(uint8_t slaveAdr);
		void loop();
		void onCommand( void (*)(uint8_t, uint8_t) );
		void handleAck(uint8_t ack);
};

#endif