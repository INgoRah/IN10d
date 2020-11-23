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
// cfg 7 w FF 3 FF FF FF FF FF FF FF FF FF FF FF FF FF FF 1 FF FF FE FF 1
#define MAX_CMD_BUFSIZE 80

extern byte mode;
extern OneWireBase *ds;
//extern OneWireBase* bus[];
extern WireWatchdog* wdt[MAX_BUS];

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
static int sw_tbl_len = 0;

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
	cmdCallback.addCmd("temp", &funcTemp);

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
		//ds = bus[i];
		curBus = i;
	} else {
		bool p;

		Serial.print(F("status="));
		Serial.print(ds->status);
		Serial.print(F(", address="));
		Serial.print(ds->mAddress, HEX);
		Serial.print(", bus=");
		Serial.println(curBus);
		Serial.print(F("  ow_pin="));
		for (i = 0; i < MAX_BUS; i++) {
			p = wdt[i]->lineRead();
			Serial.print(p);
		}
		Serial.println();
	}
}

void CmdCli::funcSearch(CmdParser *myParser)
{
	int i;

	for (i = 0; i < 4; i++) {
		Serial.print("Ch ");
		Serial.print(i);
		Serial.print(": ");
		ow->search(ds, i);
	}
}

void CmdCli::funcStatus(CmdParser *myParser)
{
	bool res;
	byte data[10], i;

	if (myParser->getParamCount() > 0) {
		byte adr[8];
		adr[1] = atoi(myParser->getCmdParam(1));
		if (myParser->getParamCount() > 1)
			res = atoi(myParser->getCmdParam(2));
		else 
			res = false;
		ow->adrGen(ds, curBus, adr, adr[1]);
		ow->ds2408RegRead(ds, curBus, adr, data, res);
		Serial.print(F("Data "));
		for (i = 0; i < 9; i++) {
			Serial.print(data[i], HEX);
			Serial.print(F(" "));
		}
		Serial.println(data[i], HEX);

	} else
		ow->statusRead(ds);
}

void CmdCli::funcPio(CmdParser *myParser)
{
	uint8_t pio, i, level;
	byte adr[8], data[10];

	// channel first
	adr[1] = me->atoh(myParser->getCmdParam(1), false);
	pio = atoi(myParser->getCmdParam(2));
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
			if (myParser->getParamCount() > 2) {
				level = atoi(myParser->getCmdParam(3));
				Serial.print(F("Level: "));
				Serial.println(level);
				ow->ds2408PioSet(ds, curBus, adr, level);

			} else
				ow->ds2408TogglePio(ds, curBus, adr, pio, data);
			//ow->ds2408Status (ds, curBus, adr, true);
	}
}

void CmdCli::funcTemp(CmdParser *myParser)
{
	byte adr[8], data[10];
	int i;

	if (myParser->getParamCount() == 0) {
		uint8_t adrt[8] = { 0x28, 0x65, 0x0E, 0xFD, 0x05, 0x00, 0x00, 0x4D };
		float temp;
		byte bus;

		bus = 0;
		ow->tempRead (ds, bus, adrt);
		delay(800);
		temp = ow->tempRead (ds, bus, adrt);
		Serial.print(temp);
		Serial.println(F(" C"));

		bus = 1;
		adr[1] = 7;	
		ow->adrGen(ds, bus, adr, adr[1]);
		ow->ds2408PioSet(ds, bus, adr, 0x44);
		delay(200);
		ow->ds2408RegRead (ds, bus, adr, data, true);
		Serial.print(data[6]);
		Serial.println(F(" C"));

		return;
	}
	// channel first
	adr[1] = me->atoh(myParser->getCmdParam(1), false);
	if (adr[1] > 10) {
		uint8_t adrt[8] = { 0x28, 0x65, 0x0E, 0xFD, 0x05, 0x00, 0x00, 0x4D };
		float temp;

		ow->tempRead (ds, curBus, adrt);
		delay(800);
		temp = ow->tempRead (ds, curBus, adrt);
		Serial.print(temp);
		Serial.println(F(" C"));
		return;
	}
	ow->adrGen(ds, curBus, adr, adr[1]);
	ow->ds2408PioSet(ds, curBus, adr, 0x44);
	delay(200);
	ow->ds2408RegRead (ds, curBus, adr, data, false);
	Serial.print(F("Data "));
	for (i = 0; i < 9; i++) {
		Serial.print(data[i], HEX);
		Serial.print(F(" "));
	}
	Serial.println(data[i], HEX);
	Serial.print(data[6]);
	Serial.println(F(" C"));

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
	for (int i = 0; i < MAX_BUS;i++)
		alarm_handler(i);
}

void CmdCli::funcCfg(CmdParser *myParser)
{
	uint8_t i, len;
	byte adr[8], data[MAX_CFG_SIZE];
	char *c;

	c = myParser->getCmdParam(1);
	if (myParser->getParamCount() < 2 || *c == '?') {
		Serial.println (F("s: save"));
		Serial.println (F("w: write"));
		Serial.println (F("r: read"));
		return;
	}
	// channel first
	adr[1] = atoi(myParser->getCmdParam(1));
	c = myParser->getCmdParam(2);
	ow->adrGen(ds, curBus, adr, adr[1]);
	if (*c == 'w') {
		for (i = 0; i < MAX_CFG_SIZE; i++) {
			if (myParser->getParamCount() > (uint8_t)(1 + i))
				data[i] = me->atoh(myParser->getCmdParam(3+i), false);
			else
				break;
		}
		i--;
		Serial.print(F("Cfg write ("));
		Serial.print(i);
		Serial.print(") ");
		ow->ds2408CfgWrite(ds, curBus, adr, data, i);
	}
	if (*c == 'r') {
		Serial.print(F("Cfg Read ("));
		len = ow->ds2408CfgRead(ds, curBus, adr, data);
		Serial.print(len);
		Serial.print(F(") "));
		for (i = 0; i < len - 1; i++) {
			Serial.print(data[i], HEX);
			Serial.print(F(" "));
		}
		Serial.println(data[i], HEX);
	}
	if (*c == 's') {
		Serial.print(F("Cfg save ("));
		len = ow->ds2408CfgRead(ds, curBus, adr, data);
		data[21] = 0x55;
		len = 24;
		ow->ds2408CfgWrite(ds, curBus, adr, data, len);
		Serial.print(len);
		Serial.print(F(")"));
	}
}

void CmdCli::funcCmd(CmdParser *myParser)
{
	byte cmd, data;

	cmd = me->atoh(myParser->getCmdParam(1));
	if (myParser->getParamCount() > 1)
		data = atoi(myParser->getCmdParam(2));
	hostCommand (cmd, data);
}

extern struct _sw_tbl1 sw_tbl1[MAX_BUS][MAX_SWITCHES];
extern struct _sw_tbl sw_tbl[MAX_BUS * MAX_SWITCHES];

void CmdCli::dumpSwTbl(void) 
{
	byte i;
	union d_adr dst;
	int size = 0;
#if 0
	byte j;
	union s_adr src;

	for (j = 0; j < MAX_BUS; j++) {
		for (i = 0; i < MAX_SWITCHES; i++) {
			src.data = sw_tbl[j][i].src.data;
			dst.data = sw_tbl[j][i].dst.data;
			if (src.data == 0)
				continue;
			if (src.data == 0xff && dst.data == 0xff)
				continue;
			size += 2;
			Serial.print(j);
			Serial.print(".");
			Serial.print(src.sa.adr);
			Serial.print(".");
			Serial.print(src.sa.latch);
			Serial.print(" -> ");
			Serial.print(dst.da.bus);
			Serial.print(".");
			Serial.print(dst.da.adr);
			Serial.print(".");
			Serial.println(dst.da.pio);
		}
	}
#else
	union s_adr src;

	for (i = 0; i < (MAX_BUS * MAX_SWITCHES); i++) {
		src.data = sw_tbl[i].src.data;
		dst.data = sw_tbl[i].dst.data;
		if (src.data == 0)
			continue;
		if (src.data == 0xffff && dst.data == 0xff)
			continue;
		size += sizeof(struct _sw_tbl);
		src.sa.res = 0;
		Serial.print(src.sa.bus);
		Serial.print(".");
		Serial.print(src.sa.adr);
		Serial.print(".");
		Serial.print(src.sa.press * 10 + src.sa.latch);
		Serial.print(" -> ");
		Serial.print(dst.da.bus);
		Serial.print(".");
		Serial.print(dst.da.adr);
		Serial.print(".");
		Serial.print(dst.da.pio);
		Serial.print(" (");
		Serial.print(src.data, HEX);
		Serial.print(" | ");
		Serial.print(dst.data, HEX);
		Serial.println(")");
	}
#endif	
	Serial.print("Size=");
	Serial.print(size);
	Serial.print("/");
	Serial.println(sizeof(sw_tbl));
	sw_tbl_len = size;
}

extern void initSwTable();

void CmdCli::funcSwCmd(CmdParser *myParser)
{
	byte i;
	union s_adr src;
	union d_adr dst;
	char *c;

	if (myParser->getParamCount() == 0) {
		me->dumpSwTbl();
	}
	if (myParser->getParamCount() == 1) {
		c = myParser->getCmdParam(1);
		if (*c == '?') {
			Serial.println ("1: read");
			Serial.println ("2: save");
			Serial.println ("3: clear");
			return;
		}
		byte cmd = atoi(myParser->getCmdParam(1));
		uint8_t vers;

		switch (cmd) {
			case 1: 	// read eeprom
			{
				initSwTable();
				break;
			}
			case 2:		// write eeprom
				vers = 2;
				me->dumpSwTbl();
				eeprom_write_byte((uint8_t*)0, vers);
				eeprom_write_word((uint16_t*)2, sw_tbl_len);
				eeprom_write_block((const void*)sw_tbl, (void*)4, sw_tbl_len);
				Serial.print(F("EEPROM saved ("));
				Serial.print(sizeof(sw_tbl));
				Serial.println(F(" bytes)"));
				break;
			case 3:
				memset (sw_tbl, 0, sizeof(sw_tbl));
				sw_tbl_len = 0;
				break;
		}
		// cmd like store to eeprom
	}
	src.data = 0;
	dst.data = 0;
	if (myParser->getParamCount() == 3 || myParser->getParamCount() > 5 ) {
		uint8_t latch;

		src.sa.bus = atoi(myParser->getCmdParam(1));
		src.sa.adr = atoi(myParser->getCmdParam(2)); 
		latch = atoi(myParser->getCmdParam(3));
		if (latch - 30 > 0) {
			/* pressing */
			src.sa.press = 2;
			src.sa.latch = latch - 30;
		} else if (latch - 20 > 0) {
			/* press long */
			src.sa.press = 1;
			src.sa.latch = latch - 20;
		} else {
			src.sa.latch = latch;
			src.sa.press = 0;
		}
		Serial.print(src.sa.bus, HEX);
		Serial.print(".");
		Serial.print(src.sa.adr, HEX);
		Serial.print(".");
		Serial.print(src.sa.latch, HEX);
		Serial.print(" ");
		Serial.print(src.sa.press, HEX);
		Serial.print(" | ");
		Serial.print(src.data, HEX);
	}
	if (myParser->getParamCount() == 3) {
		for (i = 0; i < (MAX_BUS * MAX_SWITCHES); i++) {
			if (sw_tbl[i].src.data == src.data) {
				/* move following or last to here */
				sw_tbl[i].src.data = 0;
				sw_tbl[i].dst.data = 0xff;
				break;
			}
		}
	}
	if (myParser->getParamCount() > 5) {
		dst.da.bus = atoi(myParser->getCmdParam(4));
		dst.da.adr = atoi(myParser->getCmdParam(5));
		dst.da.pio = atoi(myParser->getCmdParam(6));
		if (myParser->getParamCount() > 6)
			dst.da.type = atoi(myParser->getCmdParam(7));
		for (i = 0; i < (MAX_BUS * MAX_SWITCHES); i++) {
			if (sw_tbl[i].src.data == 0 || sw_tbl[i].src.data == src.data) {
				sw_tbl[i].src.data = src.data;
				sw_tbl[i].dst.data = dst.data;
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
			return;
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
		if (inChar > 0x7F) {
			Serial.println(inChar, HEX);
			return;
		}
	}
}

