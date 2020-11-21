/*
 * Library classs includes
 */
#include <Wire.h>
#include <TwiHost.h>

#define DS2482_CMD_RESET               0xF0	/* No param */
#define DS2482_CMD_CHANNEL_SELECT      0xC3	/* Param: Channel byte - DS2482-800 only */
#define DS2482_CMD_MODE                0x69 /* Param: mode byte */
#define DS2482_CMD_SET_READ_PTR        0xE1	/* Param: DS2482_PTR_CODE_xxx */
#define DS2482_CMD_DATA 0x96

extern void ow_monitor();
extern byte alarmSignal, wdFired, ledOn;
extern unsigned long ledOnTime;
extern unsigned long wdTime;

void (*TwiHost::user_onCommand)(uint8_t, uint8_t);

// cache the last read byte on 1W bus
//static uint8_t rdData[16];
static uint8_t* rdData = NULL;
static uint8_t rdLen, rdPos;
uint8_t cmd = 0xFF;
// registers
byte cfg, ch, mode;
static uint8_t reg;
static byte status;

TwiHost::TwiHost(byte slaveAdr)
{
	this->slaveAdr = slaveAdr;
	mode = 0;
}

void TwiHost::begin()
{
	cmd = 0xff;
	rdLen = 0;
	mode = MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH;
	Wire.begin(slaveAdr);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
}

void TwiHost::end()
{
}

void TwiHost::onCommand( void (*function)(uint8_t, uint8_t) )
{
  user_onCommand = function;
}

void TwiHost::setReg(uint8_t _reg)
{
	reg = _reg;
}

void TwiHost::setStatus(uint8_t stat)
{
	reg = DS2482_STATUS_REGISTER;
	status = stat;
};

void TwiHost::loop()
{
	if (cmd != 0xff && user_onCommand) {
		/* TODO: select ds by channel */
		user_onCommand(cmd, 0);
		// mark as handled
		cmd = 0xff;
	}
}

void TwiHost::setData(uint8_t *data, uint8_t len) {
/*	if (len > sizeof(rdData))
		return;
	memcpy(rdData, data, len);
	*/
	rdData = data;
	rdLen = len;
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void TwiHost::receiveEvent(int howMany) {
	byte d;

	d = Wire.read();
	switch (d)
	{
	case DS2482_CMD_RESET:
		break;
	case DS2482_CMD_CHANNEL_SELECT:
		reg = DS2482_CHANNEL_SELECTION_REGISTER;
		d = Wire.read() & 0x0f;
		user_onCommand(d, d);
		break;
	case DS2482_CMD_SET_READ_PTR:
		reg = Wire.read();
		break;
	case DS2482_CMD_DATA:
		reg = DS2482_DATA_REGISTER;
		rdPos = 0;
		//Wire.write(rdData, rdLen);
		break;
	case DS2482_CMD_MODE:
		mode = Wire.read();
		Serial.print("mode=");
		Serial.println(mode, HEX);
		break;
	default:
		cmd = d;
		// handle in loop to not block status reads
	}
	wdTime = millis();
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void TwiHost::requestEvent() {
	switch (reg)
	{
	case DS2482_MODE_REGISTER:
		Wire.write(mode);
		break;
	case DS2482_STATUS_REGISTER:
		Wire.write(status);
		break;
	case DS2482_DATA_REGISTER:
		if (rdPos < rdLen && rdData != NULL)
			Wire.write(rdData[rdPos++]);
		else
			Wire.write(0xff);
		break;
	case DS2482_ALARM_STATUS_REGISTER:
		digitalWrite(HOST_ALRM_PIN, HIGH);
		Wire.write(alarmSignal | (wdFired << 1));
		if (alarmSignal) {
			alarmSignal = 0;
			digitalWrite(13, 0);
		}
		if (wdFired) {
			digitalWrite(13, 0);
			ledOnTime = 0;
			ledOn = 0;
			Serial.println(F("reset WDT"));
			wdFired = 0;
		}
		break;
	}
	wdTime = millis();
}
