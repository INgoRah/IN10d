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

#define DS2482_CMD_RESET               0xF0	/* No param */
#define DS2482_CMD_CHANNEL_SELECT      0xC3	/* Param: Channel byte - DS2482-800 only */
#define DS2482_CMD_MODE                0x69 /* Param: mode byte */
#define DS2482_CMD_SET_READ_PTR        0xE1	/* Param: DS2482_PTR_CODE_xxx */

#define DS2482_CHANNEL_SELECTION_REGISTER    0xD2	/* DS2482-800 only */
#define DS2482_ALARM_STATUS_REGISTER         0xA8

extern void ow_monitor();
extern byte alarmSignal, wdFired, ledOn;
extern unsigned long ledOnTime;
extern unsigned long wdTime;

void (*WireWatchdog::user_onFired)(void);
void (*WireWatchdog::user_onAlarm)(void);
void (*WireWatchdog::user_onCommand)(uint8_t);

// cache the last read byte on 1W bus
byte rd_data, wr_data;
uint8_t cmd = 0xFF, reg;
// registers
byte cfg, ch, status, mode;

WireWatchdog::WireWatchdog(byte owPin, byte slaveAdr)
{
	mask_ = PIN_TO_BITMASK(owPin);
	reg_ = PIN_TO_BASEREG(owPin);
	this->slaveAdr = slaveAdr;
	mode = 0;
}

void WireWatchdog::begin()
{
	cmd = 0xff;
	rd_data = 0xff;
	mode = MODE_ALRAM_HANDLING /* | MODE_ALRAM_POLLING */;
	Wire.begin(slaveAdr);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
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

void WireWatchdog::onCommand( void (*function)(uint8_t) )
{
  user_onCommand = function;
}

void WireWatchdog::loop()
{
	if (cmd != 0xff && user_onCommand) {
		/* TODO: select ds by channel */
		user_onCommand(cmd);
		// mark as handled
		cmd = 0xff;
	}
}

bool WireWatchdog::lineRead()
{
	return DIRECT_READ(reg_, mask_);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void WireWatchdog::receiveEvent(int howMany) {
	byte d;

	d = Wire.read();
	switch (d)
	{
	case DS2482_CMD_RESET:
		//::attachInterrupt(digitalPinToInterrupt(owPin), ow_monitor, CHANGE);
		break;
	case DS2482_CMD_CHANNEL_SELECT:
		reg = DS2482_CHANNEL_SELECTION_REGISTER;
		d = Wire.read() & 0x0f;
		break;
	case DS2482_CMD_SET_READ_PTR:
		reg = Wire.read();
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
void WireWatchdog::requestEvent() {
	switch (reg)
	{
	case DS2482_ALARM_STATUS_REGISTER:
		digitalWrite(A1, HIGH);
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