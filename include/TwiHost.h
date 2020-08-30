#ifndef _TWIHOST_H
#define _TWIHOST_H

#include <Arduino.h>       // for delayMicroseconds, digitalPinToBitMask, etc

#define MODE_WATCHDOG 0x1
#define MODE_ALRAM_POLLING 0x2
#define MODE_ALRAM_HANDLING 0x4

#define HOST_ALRM_PIN 6

#define DS2482_CHANNEL_SELECTION_REGISTER    0xD2	/* DS2482-800 only */
#define DS2482_STATUS_REGISTER         0xE1
#define DS2482_ALARM_STATUS_REGISTER         0xA8
#define DS2482_MODE_REGISTER         0x69
#define DS2482_DATA_REGISTER 0xA9

class TwiHost
{
	private:
		byte slaveAdr;

		static void (*user_onCommand)(uint8_t cmd, uint8_t data);
		static void receiveEvent(int howMany);
		static void requestEvent();

public:
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