/* LICENSE
 *
 */
/*
 * Includes
 */
#include "CmdCli.h"
#include <avr/pgmspace.h>
/*
 * Library classs includes
 */
#include <Wire.h>
#include <OneWireBase.h>
#include <DS2482.h>
#include <OneWire.h>
#include <OwDevices.h>
#include "WireWatchdog.h"

#define DEBUG
/*
 * Local constants
 */
#define MAX_CMD_BUFSIZE 16

extern byte mode;
extern OneWireBase *ds;
extern OneWireBase* bus[];
extern WireWatchdog wdt;

extern bool alarm_handler(byte busNr);
extern void hostCommand(uint8_t cmd, uint8_t data);

CmdCli* me;

/*
 * Objects
 */

/*
 * Local variables
 */
static boolean stringComplete = false;  // whether the string is complete
static String inputString = "";         // a String to hold incoming data
static OwDevices* ow;
static byte curBus;

/* Based on code found in https://forum.arduino.cc/index.php?topic=90.0 */
uint8_t CmdCli::atoh(const char *str, bool prefix)
{
	uint8_t b, lo, i = 0;

	/* check for existing prefix '0x' for hex or dec */
	if (str[1] != 0 && str[i] == '0' && str[1] == 'x')
		i += 2;
	if (prefix && i == 0)
		return atoi(str);
	b = toupper(str[i++]);
	if (isxdigit(b)) {
		if (b > '9')
			// software offset for A-F
			b -= 7;
		// subtract ASCII offset
		b -= 0x30;
		lo = toupper(str[i]);
		if (lo != 0 && isxdigit(lo)) {
			b = b << 4;
			if (lo > '9')
				lo -= 7;
			lo -= 0x30;
			b = b + lo;
		}
		return b;
	}

	return 0;
}

/*
* Function declarations
*/
void CmdCli::begin(OwDevices* devs)
{
	me = this;
	ow = devs;
	curBus = 0;
	cmdCallback.addCmd("srch", &funcSearch);
	cmdCallback.addCmd("pset", &funcPinSet);
	cmdCallback.addCmd("stat", &funcStatus);
	cmdCallback.addCmd("mode", &funcMode);
	cmdCallback.addCmd("pio", &funcPio);
	cmdCallback.addCmd("data", &funcData);
	cmdCallback.addCmd("bus", &funcBus);
	cmdCallback.addCmd("alarm", &funcAlarmSrch);
	cmdCallback.addCmd("cfg", &funcCfg);
	cmdCallback.addCmd("cmd", &funcCmd);
	cmdCallback.addCmd("sw", &funcSwCmd);

	//cmdCallback.addCmd("test", &funcTest);
	// reserve bytes for the inputString
	inputString.reserve(MAX_CMD_BUFSIZE);
	resetInput();
}

void CmdCli::loop()
{
	unsigned char cmdBuf[MAX_CMD_BUFSIZE];

	if (stringComplete) {
		// issue with trailing 0, maybe using c_str
		inputString.getBytes(cmdBuf, inputString.length() + 1);

		if (cmdParser.parseCmd(cmdBuf,
			inputString.length()) != CMDPARSER_ERROR) {
			cmdCallback.processCmd(&cmdParser);
		} else
		{
			Serial.println(F("err"));
		}
		
		resetInput();
	}
}

void CmdCli::funcBus(CmdParser *myParser)
{
	int i;
	
	if (myParser->getParamCount() > 0) {
		i = atoi(myParser->getCmdParam(1));
		ds = bus[i];
		curBus = i;
	} else {
		bool p;

		Serial.print("status=");
		Serial.print(ds->status);
		Serial.print(", address=");
		Serial.print(ds->mAddress, HEX);
		Serial.print(", bus=");
		Serial.println(curBus);
		p = wdt.lineRead();
		Serial.print(F("  ow_pin="));
		Serial.println(p);
	}
}

void CmdCli::funcSearch(CmdParser *myParser)
{
	int i;

	for (i = 0; i < 2; i++) {
		Serial.print("Ch ");
		Serial.print(i);
		ow->search(ds, i);
	}
}

void CmdCli::funcStatus(CmdParser *myParser)
{
	bool res;

	if (myParser->getParamCount() > 0) {
		byte adr[8];
		adr[1] = atoi(myParser->getCmdParam(1));
		if (myParser->getParamCount() > 1)
			res = atoi(myParser->getCmdParam(2));
		else 
			res = false;
		ow->ds2408Status (ds, curBus, adr, res);
	} else
		ow->statusRead(ds);
}

void CmdCli::funcPio(CmdParser *myParser)
{
	uint8_t pio, i;
	byte adr[8], data[10];
	bool res;
	union d_adr d;

	// channel first
	adr[1] = me->atoh(myParser->getCmdParam(1), false);
	pio = atoi(myParser->getCmdParam(2));
	if (myParser->getParamCount() > 1)
		res = atoi(myParser->getCmdParam(3));
	else
		res = true;
	d.da.type = 0;
	d.da.bus = curBus;
	d.da.adr = adr[1];
	d.da.pio = pio - 1;
	Serial.print(F("Target="));
	Serial.println(d.data, HEX);
	switch (adr[1]) {
		case 10:
			static uint8_t target[8] = { 0x3A, 0x01, 0xDA, 0x84, 0x00, 0x00, 0x05, 0xA3 };
			ow->toggleDs2413 (ds, curBus, target);
			break;
		default:
			ow->adrGen(ds, curBus, adr, adr[1]);
			ow->ds2408RegRead(ds, curBus, adr, data, false);
			for (i = 0; i < 9; i++) {
				Serial.print(data[i], HEX);
				Serial.print(F(" "));
			}
			Serial.println(data[i], HEX);
			ow->ds2408TogglePio(ds, curBus, adr, pio, data);
			ow->ds2408Status (ds, curBus, adr, res);
	}
}

void CmdCli::funcData(CmdParser *myParser)
{
	uint8_t len;
	byte adr[8];

	adr[1] = atoi(myParser->getCmdParam(1));
	len = atoi(myParser->getCmdParam(2));
	ow->ds2408Data(ds, curBus, adr, len);
}

void CmdCli::funcTest(CmdParser *myParser)
{
#if 0
	if (myParser->getParamCount() > 0) {
		uint8_t slot, w1l, w0l, rc, msr; 

		slot = atoi(myParser->getCmdParam(1));
		w0l = atoi(myParser->getCmdParam(3));
		w1l = atoi(myParser->getCmdParam(2));
		msr = atoi(myParser->getCmdParam(4));
		rc = atoi(myParser->getCmdParam(5));
		test_detect (ds, slot, w1l, w0l, msr, rc);
	} else
		ow_line_check(ds);
#endif
}

void CmdCli::funcMode(CmdParser *myParser)
{
	if (myParser->getParamCount() > 0)
		mode = atoi(myParser->getCmdParam(1));
	Serial.print(F(" mode="));
	Serial.print(mode);
}

void CmdCli::funcPinSet(CmdParser *myParser)
{
	uint8_t pin, val;

	Serial.print(F("pset: "));
	pin = atoi(myParser->getCmdParam(1));
	Serial.print(F(" pin: "));
	Serial.print(pin);
	Serial.print(F(" val: "));
	val = atoi(myParser->getCmdParam(2));
	Serial.println(val);
	pinMode(pin, OUTPUT);
	digitalWrite (pin, val);
}

void CmdCli::funcAlarmSrch(CmdParser *myParser)
{
	for (int i = 0; i < 2;i++)
		alarm_handler(i);
}

void CmdCli::funcCfg(CmdParser *myParser)
{
	uint8_t i;
	byte adr[8], data[24];

	// channel first
	adr[1] = atoi(myParser->getCmdParam(1));
	
	for (i = 0; i < 24;i++) {
		if (myParser->getParamCount() > (uint8_t)(1 + i))
			data[i] = me->atoh(myParser->getCmdParam(2+i), false);
		else
			break;
	}
	ow->adrGen(ds, curBus, adr, adr[1]);
	ow->ds2408Cfg(ds, curBus, adr, data, i);
}

void CmdCli::funcCmd(CmdParser *myParser)
{
	byte cmd, data;

	cmd = atoi(myParser->getCmdParam(1));
	if (myParser->getParamCount() > 1)
		data = atoi(myParser->getCmdParam(2));
	hostCommand (cmd, data);
}

extern byte sw_tbl[MAX_BUS][MAX_SWITCHES][2];

void CmdCli::dumpSwTbl(void) 
{
	byte j, i;
	union s_adr src;
	union d_adr dst;

	for (j = 0; j < MAX_BUS; j++) {
		for (i = 0; i < MAX_SWITCHES; i++) {
			src.data = sw_tbl[j][i][0];
			dst.data = sw_tbl[j][i][1];
			if (src.data == 0)
				continue;
			if (src.data == 0xff && dst.data == 0xff)
				continue;
			Serial.print(j);
			Serial.print(".");
			Serial.print(src.sa.adr, HEX);
			Serial.print(".");
			Serial.print(src.sa.latch, HEX);
			Serial.print(" -> ");
			Serial.print(dst.da.bus, HEX);
			Serial.print(".");
			Serial.print(dst.da.adr, HEX);
			Serial.print(".");
			Serial.println(dst.da.pio, HEX);
		}
	}
}

extern void initSwTable();

void CmdCli::funcSwCmd(CmdParser *myParser)
{
	byte bus, i;
	union s_adr src;
	union d_adr dst;

	if (myParser->getParamCount() == 0) {
		me->dumpSwTbl();
	}
	if (myParser->getParamCount() == 1) {
		byte cmd = atoi(myParser->getCmdParam(1));
		uint16_t len;
		uint8_t vers;

		switch (cmd) {
			case 1: 	// read eeprom
			{
				initSwTable();
				break;
			}
			case 2:		// write eeprom
				vers = 1;
				len = sizeof(sw_tbl);
				eeprom_write_byte((uint8_t*)0, vers);
				eeprom_write_word((uint16_t*)2, len);
				eeprom_write_block((const void*)sw_tbl, (void*)4, len);
				break;
			case 3:
				memset (sw_tbl, 0, sizeof(sw_tbl));
				break;
		}
		// cmd like store to eeprom
	}
	if (myParser->getParamCount() == 3) {
		bus = atoi(myParser->getCmdParam(1));
		src.data = me->atoh(myParser->getCmdParam(2), false);
		dst.data = me->atoh(myParser->getCmdParam(3), false);
		for (i = 0; i < MAX_SWITCHES; i++) {
			if (sw_tbl[bus][i][0] == 0 || sw_tbl[bus][i][0] == src.data) {
				sw_tbl[bus][i][0] = src.data;
				sw_tbl[bus][i][1] = dst.data;
				break;
			}
		}
		me->dumpSwTbl();
	}
}

void CmdCli::resetInput()
{
	// clear the string:
	inputString = "";
	stringComplete = false;
	Serial.print(F("\now:# "));
	Serial.flush();
}

/*
 * SerialEvent occurs whenever a new data comes in the hardware serial RX. This
 * routine is run between each time loop() runs, so using delay inside loop can
 * delay response. Multiple bytes of data may be available.
 */
void serialEvent() {
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		// ESC
		if (inChar == 27) {
			Serial.println();
			CmdCli::resetInput();
			return;
		}
		if (inChar == 0xE0) {
			inChar = (char)Serial.read();
			Serial.print("cursor ");
			Serial.println(inChar, HEX);
		}
		// if the incoming character is a newline, set a flag so the
		// main loop can do something about it:
		if (inChar == 0x0d) {
			stringComplete = true;
			Serial.println();
			return;
		}
		if (inChar == 0x08 ||
				(inChar > 0x1f && inChar < 0x7f)) {
			// add it to the inputString:
			inputString += inChar;
			Serial.print(inChar);
			Serial.flush();
			return;
		}
	}
}

