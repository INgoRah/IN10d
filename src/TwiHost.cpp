/*
 * Library classs includes
 */
#include <Wire.h>
#include <avr/wdt.h>
#include <main.h>
#include <TwiHost.h>
#include "SwitchHandler.h"

#define CMD_RESET               0xF0	/* No param */
#define CMD_CHANNEL_SELECT      0xC3	/* Param: Channel byte - DS2482-800 only */
#define CMD_MODE                0x69 /* Param: mode byte */
#define CMD_SW_CFG             0x6A /* Param: light threshold byte | default dim light on level */
#define CMD_SET_READ_PTR        0xE1	/* Param: DS2482_PTR_CODE_xxx */
#define CMD_SET_READ_PTR_LOCK	0xE2
#define CMD_UNLOCK 0xEF
#define CMD_DATA 0x96
#define CMD_TIME 0x40
#define CMD_EVT_DATA 0x01
#define CMD_REBOOT 0xDE
/** Acknowledge event data reception with the id. This will only remove
 * it from the reporting queue
 */
#define CMD_ACK 0x78
#define CMD_SWITCH 3

extern TwiHost host;
extern SwitchHandler swHdl;

extern byte alarmSignal, wdFired, ledOn;
extern unsigned long ledOnTime;

unsigned long host_lock = 0;

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
	rdLen = sizeof(hostData);
	rdData = (uint8_t*)hostData;
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

void TwiHost::setAlarm(uint8_t alarm)
{
	if (alarm) {
		if (alarmSignal == 0) {
			alarmSignal = 1;
			digitalWrite(13, HIGH);
		}
	}
	else {
		if (alarmSignal) {
			alarmSignal = 0;
			digitalWrite(13, LOW);
		}
	}
}


uint8_t TwiHost::getStatus()
{
	uint8_t stat = status;

	if (host_lock)
		stat = STAT_LOCK;
	if (events.size() > 0)
		stat |= STAT_EVT;
	return stat;
}

void TwiHost::commandData()
{
	struct logData d;
	union s_adr src;

	// check for last ack...if not done, do not pop
	if (_ack != _seq) {
		if (debug > 2) {
			log_time();
			Serial.print (F("repeating "));
			Serial.print (_seq, HEX);
			Serial.print (F(" last="));
			Serial.println (_ack, HEX);
		}
		setStatus(STAT_OK);
		return;
	}
	_seq++;
	if (_seq == 0x80)
		_seq = 1;
	d = events.pop();
	src.data = d.source;
	hostData[0] = d.type;
	hostData[1] = src.sa.bus;
	hostData[2] = src.sa.adr;
	hostData[3] = src.sa.latch;
	hostData[4] = src.sa.press;
	hostData[5] = (d.data & 0xff00) >> 8;
	hostData[6] = d.data & 0xff;
	hostData[7] = _seq;
	hostData[8] = 0xAA; // should be crc
	setStatus(STAT_OK);
}

void TwiHost::command()
{
	switch (cmd)
	{
	case CMD_EVT_DATA:
		if (events.size() > 0) {
			commandData();
		} else
			setStatus(STAT_NO_DATA);
		break;
#if 0
	case 0x02:
	{
		/* programming switch table */
		int i;
		union s_adr src;
		union d_adr_8 dst;

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
		host.setStatus(STAT_OK);
		break;
	}
#endif
	case CMD_SWITCH:
		// switching
		{
			byte level;
			union pio dst;

			host_lock = 0;

			setStatus(STAT_PROCESSING);
			dst.data = 0;
			if (rxBytes < 4 && debug > 0) {
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
			if (dst.da.bus == 0 && dst.da.adr == 9)
				dst.da.type = 2;
			else
				dst.da.type = 0;
#ifdef DEBUG
			if (debug > 1) {
				log_time();
				printDst(dst);
				Serial.print(F(" level="));
				Serial.println(level);
			}
#endif
			// switch off I2C slave till done
			if (swHdl.switchLevel(dst, level))
				setStatus(STAT_OK);
			else {
				setStatus(STAT_NOPE);
				log_time();
				printDst(dst);
				Serial.println(F(" Host cmd failed!"));
			}
			break;
		}
#if 0
		case 0x04:
		{
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
			host.setStatus(STAT_OK);
			break;
		}
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
		wdt_reset();
		command();
		if (/*status != STAT_NOPE*/ 1) {
			// mark as handled
			cmd = 0xff;
			rxBytes = 0;
			// retry...
			// return
		}
		// more bytes in the queue?
	}
	if (rxBytes > 0) {
		/* should never happen */
		if (debug > 0) {
			Serial.print (F("Data not handled: "));
			Serial.print (Wire.available());
		}
		if (Wire.available()) {
			uint8_t d = Wire.read();
			if (debug > 0) {
				Serial.print (F(" Bytes, Data="));
				Serial.println (d, HEX);
			}
		}
		rxBytes = 0;
	}
	if (host.events.size() > 0 || _ack != _seq) {
		digitalWrite(HOST_ALRM_PIN, LOW);
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
	/* todo: if nothing in fifo, put it to the host data
	   and prepare for data get
	*/
	events.push(d);
	// let this be handled in the main loop
	// alarm pin will be raised if events contains
	// data
	//digitalWrite (HOST_ALRM_PIN, LOW);
};

void TwiHost::addEvent(uint8_t type, uint8_t bus, uint8_t adr, uint16_t data)
{
	union s_adr src;

	src.data = 0;
	src.sa.bus = bus;
	src.sa.adr = adr;
	addEvent(type, src.data, data);
}

void TwiHost::addEvent(union pio dst, uint16_t data, uint8_t type)
{
	union s_adr src;
	src.data = 0;
	src.sa.bus = dst.da.bus;
	src.sa.adr = dst.da.adr;
	src.sa.latch  = dst.da.pio;
	addEvent(type, src.data, data);
}

void TwiHost::addEvent(union d_adr_8 dst, uint16_t data, uint8_t type)
{
	union s_adr src;
	src.data = 0;
	src.sa.bus = dst.da.bus;
	src.sa.adr = dst.da.adr;
	src.sa.latch  = dst.da.pio;
	addEvent(type, src.data, data);
}

/* This function is called from an event
*/
void TwiHost::handleAck(uint8_t ack)
{
	unsigned long tm = millis() - host_lock;

	if (debug > 4 && host_lock) {
		Serial.print (F("I2C locked time = "));
		Serial.println(tm);
	}
	if (ack == _seq) {
		// serviced
		_ack = ack;
		if (debug > 3) {
			log_time();
			Serial.print (F("ACKed "));
			Serial.println(ack, HEX);
		}
		setStatus(STAT_OK);
	} else {
		setStatus(STAT_WRONG);
		if (debug > 0) {
			log_time();
			Serial.print (F("ACK mismatch "));
			Serial.print (ack, HEX);
			Serial.print (F(" != "));
			Serial.println (_seq, HEX);
		}
	}
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void TwiHost::receiveEvent(int howMany) {
	byte d;

	if (howMany < 1) {
		return;
	}
	d = Wire.read();
	if (cmd != 0xff && cmd != d &&
		(d == CMD_SWITCH || d == CMD_EVT_DATA)) {
		/* TODO: Handle queue
		5:27:37 1.1.2 level=100
 		cmd 3 not yet handled, Stat  2 new 3
		last ACK 4A != 4A
		*/
		if (debug > 0) {
			Serial.print (F(" cmd "));
			Serial.print (cmd);
			Serial.print (F(" not yet handled, Stat  "));
			Serial.print (host.getStatus(), HEX);
			Serial.print (F(" new "));
			Serial.println (d, HEX);
			Serial.print (F("last ACK "));
			Serial.print (host._ack, HEX);
			Serial.print (F(" != "));
			Serial.println (host._seq, HEX);
		}
	}
	/* assert if not at least 1? */
	switch (d)
	{
	case CMD_REBOOT:
		Serial.print (F("Forced Reset..."));
		while (1);
	case CMD_RESET:
		host.setStatus(STAT_OK);
		Serial.print (F("reset, cmd "));
		Serial.print (cmd);
		cmd = 0xff;
		break;
#if 0
	case CMD_CHANNEL_SELECT:
		reg = DS2482_CHANNEL_SELECTION_REGISTER;
		d = Wire.read() & 0x0f;
		user_onCommand(d, d);
		break;
#endif
	case CMD_UNLOCK:
		host_lock = 0;
		break;
	case CMD_SET_READ_PTR_LOCK:
		host_lock = millis();
		// fall through
	case CMD_SET_READ_PTR:
		if (howMany < 2) {
			Serial.println (F("READ PTX underflow"));
			reg = DS2482_ALARM_STATUS_REGISTER;
		}
		else
			reg = Wire.read();
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
		swHdl.mode = Wire.read();
		break;
	case CMD_SW_CFG:
		swHdl.light_thr = Wire.read();
		swHdl.dim_on_lvl = Wire.read();
		break;
	case CMD_ACK:
		if (howMany == 2)
			host.handleAck(Wire.read());
		break;
	case CMD_EVT_DATA: // get event data
		if (host.events.size() > 0) {
			host.setStatus(STAT_BUSY);
			cmd = CMD_EVT_DATA;
		}
		else
			host.setStatus(STAT_NO_DATA);
		break;
	case CMD_TIME:
		if (howMany > 1)
			hour = Wire.read();
		if (howMany > 2)
			min = Wire.read();
		if (howMany > 3)
			sun = Wire.read();
		else
			Serial.println (F("Time underflow"));

		host.setStatus(STAT_OK);
		break;
	case CMD_SWITCH:
		cmd = CMD_SWITCH;
		host.setStatus(STAT_BUSY);
		host.rxBytes = howMany - 1;
		for (uint8_t i = 0; i < howMany - 1; ++i)
			rxBuf[i] = Wire.read();
		break;
	default:
		Serial.println(F("unkown CMD"));
		break;
#if 0
	case 2:
		/* switch table */
		I2C_READ(dst.da.type);
		I2C_READ(src.sa.bus);
		I2C_READ(src.sa.adr);
		I2C_READ(src.sa.latch);
		I2C_READ(src.sa.press);
		I2C_READ(dst.da.bus);
		I2C_READ(dst.da.adr);
		I2C_READ(dst.da.pio);
		/*fall-through */
	case 4:
		/* status read ... */
		// more bytes received, read in loop
		host.setStatus(STAT_BUSY);
		/*fall-through */
	default:
		Serial.println(F("unkown CMD"));
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
#endif
	}
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void TwiHost::requestEvent()
{
	uint8_t val;

	switch (reg)
	{
	case DS2482_MODE_REGISTER:
		Wire.write(swHdl.mode);
		break;
	case DS2482_STATUS_REGISTER:
		val = host.getStatus();
		Wire.write(val);
		break;
	case DS2482_DATA_REGISTER:
		/* write bulk of data */
		if (rdPos < rdLen && rdData != NULL) {
			Wire.write(rdData, rdLen);
			val = host.getStatus();
			Wire.write(val);
			rdPos = rdLen;
			host.status = STAT_READY;
		}
		else {
			host.setStatus(STAT_NO_DATA);
			Wire.write(0xff);
		}
		reg = DS2482_STATUS_REGISTER;
		break;
	case DS2482_ALARM_STATUS_REGISTER:
		{
			uint8_t stat = host.alarmSignal;
			digitalWrite(HOST_ALRM_PIN, HIGH);
			if (host.events.size() > 0) {
				stat |= STAT_EVT;
			}
			Wire.write((uint8_t)(stat /*| (wdFired << 1)*/ ));
			host.setAlarm(0);
#if 0
			if (wdFired) {
				digitalWrite(13, 0);
				ledOnTime = 0;
				ledOn = 0;
				Serial.println(F("reset WDT"));
				wdFired = 0;
			}
#endif
			break;
		}
	}
}
