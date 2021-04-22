/*
 * Library classs includes
 */
#include <Wire.h>
#include <main.h>
#include <TwiHost.h>
#include "SwitchHandler.h"

#define CMD_RESET               0xF0	/* No param */
#define CMD_CHANNEL_SELECT      0xC3	/* Param: Channel byte - DS2482-800 only */
#define CMD_MODE                0x69 /* Param: mode byte */
#define CMD_SET_READ_PTR        0xE1	/* Param: DS2482_PTR_CODE_xxx */
#define CMD_DATA 0x96
#define CMD_TIME 0x40
#define CMD_SWITCH 3

extern TwiHost host;
extern SwitchHandler swHdl;

extern byte alarmSignal, wdFired, ledOn;
extern unsigned long ledOnTime;
extern unsigned long wdTime;

void (*TwiHost::user_onCommand)(uint8_t, uint8_t);

// cache the last read byte on 1W bus
static uint8_t* rdData = NULL;
static uint8_t rdLen, rdPos;
uint8_t cmd = 0xFF;
// registers
static uint8_t reg;
static uint8_t rxBuf[4];

TwiHost::TwiHost()
{
	rdLen = 0;
	cmd = 0xff;
}

void TwiHost::begin(uint8_t slaveAdr)
{
	digitalWrite(HOST_ALRM_PIN, HIGH);
	pinMode(HOST_ALRM_PIN, OUTPUT);

	Wire.begin(slaveAdr);
	Wire.onReceive(receiveEvent);
	Wire.onRequest(requestEvent);
}

void TwiHost::onCommand( void (*function)(uint8_t, uint8_t) )
{
  user_onCommand = function;
}

/*
void TwiHost::setReg(uint8_t _reg)
{
	reg = _reg;
}
*/

void TwiHost::setStatus(uint8_t stat)
{
	reg = DS2482_STATUS_REGISTER;
	status = stat;
};

void TwiHost::command()
{
	switch (cmd)
	{
	case 0x01:
		if (events.size() > 0) {
			struct logData d;
			union s_adr src;

			d = events.pop();
			src.data = d.source;
			hostData[0] = d.type;
			hostData[1] = src.sa.bus;
			hostData[2] = src.sa.adr;
			hostData[3] = src.sa.latch;
			hostData[4] = src.sa.press;
			hostData[5] = (d.data & 0xff00) >> 8;
			hostData[6] = d.data & 0xff;
			setData((uint8_t*)hostData, 7);
 			setStatus(STAT_OK);
		} else
			host.setStatus(STAT_NO_DATA);
		break;
	case 0x02:
	{
		/* programming switch table */
#if 0
		int i;
		union s_adr src;
		union d_adr dst;

		host.setStatus(STAT_BUSY);
		I2C_READ(dst.da.type);
		I2C_READ(src.sa.bus);
		I2C_READ(src.sa.adr);
		I2C_READ(src.sa.latch);
		I2C_READ(src.sa.press);
		I2C_READ(dst.da.bus);
		I2C_READ(dst.da.adr);
		I2C_READ(dst.da.pio);
		for (i = 0; i < MAX_SWITCHES; i++) {
			if (sw_tbl[i].src.data == 0 || sw_tbl[i].src.data == src.data) {
				sw_tbl[i].src.data = src.data;
				sw_tbl[i].dst.data = dst.data;
				break;
			}
		}
#endif
		host.setStatus(STAT_OK);
		break;
	}
	case CMD_SWITCH:
		// switching
		{
			byte level;
			union d_adr dst;

			dst.data = 0;
			if (rxBytes < 4) {
#ifdef EXT_DEBUG
				Serial.print("rx cnt=");
				Serial.println(rxBytes);
				Serial.print("avail=");
				Serial.println(Wire.available());
				Serial.println(F("invalid"));
#endif
				host.setStatus(STAT_FAIL);
				return;
			}
			dst.da.bus = rxBuf[0];
			dst.da.adr = rxBuf[1];
			dst.da.pio = rxBuf[2];
			level = rxBuf[3];
			if (dst.data == 0 || dst.data == 0xff) {
				Serial.println(F("invalid"));
				host.setStatus(STAT_FAIL);
				return;
			}
			setStatus(STAT_PROCESSING);
#ifdef EXT_DEBUG
			if (debug > 1) {
				Serial.print(dst.da.bus);
				Serial.print(F("."));
				Serial.print(dst.da.adr);
				Serial.print(F("."));
				Serial.print(dst.da.pio);
				Serial.print(F(" level="));
				Serial.println(level);

			}
#endif
			// switch off I2C slave till done
			if (swHdl.switchLevel(dst, level))
				setStatus(STAT_OK);
			else
				setStatus(STAT_NOPE);
			break;
		}
		case 0x04:
		{
#if 0
			int i;
			/* status read ... */

			I2C_READ(bus);
			I2C_READ(adr[1]);
			Serial.print(bus);
			Serial.print(F("."));
			Serial.print(adr[1]);
			ow.adrGen (bus, adr, adr[1]);
			/* here we change from slave to master
			* Would be nice to have a signal (GPIO) to signal
			* usage of the I2C */
			ow.ds2408RegRead(bus, adr, hostData, false);
			for (i = 0; i < 9; i++) {
				Serial.print(hostData[i], HEX);
				Serial.print(F(" "));
			}
			Serial.println(hostData[i], HEX);
			hostData[0] = d[0]; // PIO Logic State
			hostData[1] = d[1]; // Output latch
			hostData[2] = d[2]; // Activity latch state
			hostData[3] = d[5]; // Status
			hostData[4] = d[6]; // Status ext 1
			hostData[5] = d[7]; // Status ext 2
			host.setData((uint8_t*)hostData, 6);
#endif
			host.setStatus(STAT_OK);
			break;
		}
#if 0
		/* external search not supported. Will be done by owfs with a bus lock */
		case 0x5A:
			uint8_t cnt, adr[8], bus;
#define MAX_DATA (15 * 7)

			hostBuf = (uint8_t*)malloc(MAX_DATA);
			host.setStatus(STAT_BUSY);
			id = 0;
			ds->reset_search();
			if (ds->reset()) {
				cnt = 0;
				while (ds->search(adr)) {
					memcpy (&hostBuf[cnt], adr, 7);
					cnt += 7;
					// wrap around in case of overflow
					if (cnt > MAX_DATA)
						cnt = 0;
				}
				host.setData(hostBuf, cnt);
				host.setStatus(STAT_OK);
			}
			else
				host.setStatus(STAT_NO_DATA);
			break;
		case 0x5B:
			free(hostBuf);
			break;
#endif
		default:
			if (user_onCommand) {
				user_onCommand(cmd, 0);
				if (status == STAT_BUSY)
					setStatus(STAT_OK);
			}
			break;
	}
}

void TwiHost::loop()
{
	if (cmd != 0xff) {
		command();
		// mark as handled
		cmd = 0xff;
		rxBytes = 0;
	}
	if (rxBytes > 0) {
		/* should never happen */
		Serial.print (F("Data not handled: "));
		Serial.print (Wire.available());
		if (Wire.available()) {
			uint8_t d = Wire.read();

			Serial.print (F(" Bytes, Data="));
			Serial.println (d, HEX);
		}
		rxBytes = 0;
	}
}

extern uint8_t sec;
extern uint8_t min;
extern uint8_t hour;

void TwiHost::addEvent(uint8_t type, uint16_t source, uint16_t data)
{
	struct logData d;

	// put into fifo
	d.type = type;
	d.source = source;
	d.data = data;
	d.h = hour;
	d.min = min;
	d.sec = sec;
	events.push(d);
	digitalWrite (HOST_ALRM_PIN, LOW);
};

void TwiHost::addEvent(uint8_t type, uint8_t bus, uint8_t adr, uint16_t data)
{
	union s_adr src;

	src.data = 0;
	src.sa.bus = bus;
	src.sa.adr = adr;
	addEvent(type, src.data, data);
}

void TwiHost::addEvent(union d_adr dst, uint16_t data)
{
	union s_adr src;
	src.data = 0;
	src.sa.bus = dst.da.bus;
	src.sa.adr = dst.da.adr;
	src.sa.latch  = dst.da.pio;
	addEvent(1, src.data, data);
}

void TwiHost::setData(uint8_t *data, uint8_t len)
{
	rdData = data;
	rdLen = len;
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void TwiHost::receiveEvent(int howMany) {
	byte d;

	if (cmd != 0xff) {
		Serial.print (cmd);
		Serial.print (F(" cmd not yet handled"));

	}
	// got how many bytes - the one we read here
	host.rxBytes = howMany - 1;
	/* assert if not at least 1? */
	d = Wire.read();
	switch (d)
	{
	case CMD_RESET:
		host.setStatus(STAT_OK);
		break;
	case CMD_CHANNEL_SELECT:
		reg = DS2482_CHANNEL_SELECTION_REGISTER;
		d = Wire.read() & 0x0f;
		user_onCommand(d, d);
		break;
	case CMD_SET_READ_PTR:
		reg = Wire.read();
		host.rxBytes--;
		break;
	case CMD_DATA:
		// host will request data, so just set the register
		reg = DS2482_DATA_REGISTER;
		rdPos = 0;
		/* Preparing data is not working, cause counter reset before
		 * calling requestEvent.
		 * Flag host busy?
		*/
		host.status = STAT_READY;
		break;
	case CMD_MODE:
		mode = Wire.read();
		host.rxBytes--;
		break;
	case 1: // get event data
		if (host.events.size() > 0) {
			host.setStatus(STAT_BUSY);
			cmd = 1;
		}
		else
			host.setStatus(STAT_NO_DATA);
		break;
	case 0x40:
		hour = Wire.read();
		min = Wire.read();
		sun = Wire.read();
		host.setStatus(STAT_OK);
		host.rxBytes -= 3;
		break;
	case CMD_SWITCH:
		cmd = d;
		host.setStatus(STAT_BUSY);
		for (uint8_t i = 0; i < howMany - 1; ++i)
			rxBuf[i] = Wire.read();
		break;
	case 2:
		/* switch table */
#if 0
		I2C_READ(dst.da.type);
		I2C_READ(src.sa.bus);
		I2C_READ(src.sa.adr);
		I2C_READ(src.sa.latch);
		I2C_READ(src.sa.press);
		I2C_READ(dst.da.bus);
		I2C_READ(dst.da.adr);
		I2C_READ(dst.da.pio);
#endif
		/*fall-through */
	case 4:
		/* status read ... */
		// more bytes received, read in loop
		host.setStatus(STAT_BUSY);
		/*fall-through */
	default:
		cmd = d;
		Serial.print (cmd);
		Serial.print (F(" cmd "));
		Serial.print (host.rxBytes);
		Serial.print (F(" pending "));
		Serial.print("avail=");
		Serial.print(Wire.available());
		if (host.rxBytes > 0) {
			Serial.print (" [");
			Serial.print (Wire.peek());
			Serial.print (" ]");
		}
		Serial.println();
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
		Wire.write(host.getStatus());
		break;
	case DS2482_DATA_REGISTER:
		/* write bulk of data */
		if (rdPos < rdLen && rdData != NULL) {
			Wire.write(rdData, rdLen);
			rdPos = rdLen;
			//Wire.write(rdData[rdPos++]);
			//if (rdPos == rdLen)
			host.status = STAT_READY;
		}
		else {
			host.setStatus(STAT_NO_DATA);
			Wire.write(0xff);
		}
		break;
	case DS2482_ALARM_STATUS_REGISTER:
		{
			uint8_t stat = alarmSignal;

			digitalWrite(HOST_ALRM_PIN, HIGH);
			if (host.events.size() > 0) {
				stat |= STAT_EVT;
			}
			Wire.write((uint8_t)(stat | (wdFired << 1)));
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
	}
	wdTime = millis();
}
