#ifndef _TWIHOST_H
#define _TWIHOST_H

#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc
#include "CircularBuffer.h"

#define MODE_WATCHDOG 0x1
#define HOST_ALRM_PIN 6

#define DS2482_CHANNEL_SELECTION_REGISTER    0xD2	/* DS2482-800 only */

#define DS2482_STATUS_REGISTER         0xE1

#define STAT_OK 	0x0
#define STAT_BUSY 	0x01
#define STAT_PROCESSING 	0x2
#define STAT_NOPE 	0x03
#define STAT_WRONG 	0x05
/* ready to service data */
#define STAT_READY 	0x04
#define STAT_FAIL 	0xCC

#define STAT_EVT  	0x40
#define STAT_NO_DATA 0x80

/** Alarm status register read will clear the alarm line, the
 * watchdog and returns the current alarm status
 */
#define DS2482_ALARM_STATUS_REGISTER         0xA8
#define DS2482_MODE_REGISTER         0x69
#define DS2482_DATA_REGISTER 0xA9

#define TYPE_BRIGHTNESS 3

enum {
	SRC_CHANGE = 0,
	DST_CHANGE,
	TEMP_CHANGE,
	BRIGHTNESS_CHANGE,
	DIMMING_DOWN
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
		uint8_t hostData[12];
		byte rxBytes;
		byte _seq, _ack;
		uint8_t alarmSignal;
		static void (*user_onCommand)(uint8_t cmd, uint8_t data);
		void command();

		static void receiveEvent(int howMany);
		static void requestEvent();

	public:
		CircularBuffer<struct logData, 5> events;
		uint8_t status;

		TwiHost();
		void setStatus(uint8_t stat);
		uint8_t getStatus();
		static void setData(uint8_t *data, uint8_t len);
		void addEvent(uint8_t type, uint16_t source, uint16_t data = 0);
		void addEvent(uint8_t type, uint8_t bus, uint8_t adr, uint16_t data);
		void addEvent(union pio dst, uint16_t data = 0, uint8_t type = 1);
		void addEvent(union d_adr_8 dst, uint16_t data = 0, uint8_t type = 1);
		void begin(uint8_t slaveAdr);
		void loop();
		void onCommand( void (*)(uint8_t, uint8_t) );
		void handleAck(uint8_t ack);
};

#endif