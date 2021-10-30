/* LICENSE
 *
 */
/*
 * Includes
 */
#include <main.h>
#include "CmdCli.h"
#include <avr/pgmspace.h>
/*
 * Library classs includes
 */
#include <Wire.h>
#include <OneWireBase.h>
#include <DS2482.h>
#include <OwDevices.h>
#include "WireWatchdog.h"
#include "SwitchHandler.h"

/*
 * Local constants
 */
// cfg 7 w FF 3 FF FF FF FF FF FF FF FF FF FF FF FF FF FF 1 FF FF FE FF 1
#define MAX_CMD_BUFSIZE 80

extern OneWireBase *ds;
//extern OneWireBase* bus[];
extern WireWatchdog* wdt[MAX_BUS];
extern SwitchHandler swHdl;
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
static byte curBus, curAdr, curPio;

void printDst8(union d_adr_8 dst)
{
	Serial.print(dst.da.bus, HEX);
	Serial.print(F("."));
	Serial.print(dst.da.adr, HEX);
	Serial.print(F("."));
	Serial.print(dst.da.pio, HEX);
}

void printSrc(union s_adr src)
{
	Serial.print(src.sa.bus);
	Serial.print(F("."));
	Serial.print(src.sa.adr);
	Serial.print(F("."));
	Serial.print(src.sa.press * 10 + src.sa.latch);
}

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
	curAdr = 0;
	curPio = 0;
	cmdCallback.addCmd("srch", &funcSearch);// 1
	cmdCallback.addCmd("s", &funcStatus); 	// 2
	cmdCallback.addCmd("mode", &funcMode);	// 3
	cmdCallback.addCmd("p", &funcPio);		// 4
	cmdCallback.addCmd("bus", &funcBus); 	// 5
	cmdCallback.addCmd("cfg", &funcCfg);	// 6
	cmdCallback.addCmd("c", &funcCmd);		// 7
	cmdCallback.addCmd("sw", &funcSwCmd);	// 8
	cmdCallback.addCmd("t", &funcTemp);		// 9
#ifdef CHGID_CMD
	cmdCallback.addCmd("id", &funcChgId);		// 10
#endif

	//cmdCallback.addCmd("time", &funcTime);
#ifdef EXT_DEBUG
	cmdCallback.addCmd("pset", &funcPinSet);
	cmdCallback.addCmd("pget", &funcPinGet);
	cmdCallback.addCmd("log", &funcLog);
#endif
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
		curBus = i;
	} else {
		Serial.print(F("bus="));
		Serial.println(curBus);
		Serial.println();
	}
}

void CmdCli::funcSearch(CmdParser *myParser)
{
	int i, res;

	(void)myParser;
	for (i = 0; i < 4; i++) {
		Serial.print(F("== Ch "));
		Serial.print(i);
		Serial.println(F(" =="));
		wdt_reset();
		res = ow->search(i);
		if (res > 0) {
			Serial.print(res);
			Serial.println(F(" devs found"));
		} else {
			Serial.println(F("no devs!"));
		}
	}
}

void CmdCli::funcStatus(CmdParser *myParser)
{
	bool res;
	byte data[10], i;
	byte adr[8];

	if (myParser->getParamCount() > 1) {

		curBus = atoi(myParser->getCmdParam(1));
		curAdr = atoi(myParser->getCmdParam(2));
		if (myParser->getParamCount() > 2)
			res = atoi(myParser->getCmdParam(3));
		else
			res = false;
	}
	ow->adrGen(curBus, adr, curAdr);
#if EXT_DEBUG
	if (debug) {
		for (i = 0; i < 7; i++) {
			Serial.print(adr[i], HEX);
			Serial.print(F(" "));
		}
		Serial.println(adr[i], HEX);
	}
#endif
	ow->ds2408RegRead(curBus, adr, data, res);
	Serial.print(F("Data "));
	for (i = 0; i < 9; i++) {
		Serial.print(data[i], HEX);
		Serial.print(F(" "));
	}
	Serial.println(data[i], HEX);
}

void CmdCli::funcPio(CmdParser *myParser)
{
	uint8_t level;
	union pio dst;

	if (myParser->getParamCount() != 4 && myParser->getParamCount() != 1) {
		Serial.println(F("(bus adr pio) level !"));
		return;
	}
	if (myParser->getParamCount() == 4) {
		curBus = atoi(myParser->getCmdParam(1));
		curAdr = atoi(myParser->getCmdParam(2));
		curPio = atoi(myParser->getCmdParam(3));
		level = atoi(myParser->getCmdParam(4));
	} else
		level = atoi(myParser->getCmdParam(1));
	dst.da.bus = curBus;
	dst.da.adr = curAdr;
	dst.da.pio = curPio;
	// convenience: 1 instead of typing 100
	if (level == 1)
		level = 100;
	swHdl.switchLevel(dst, level);
}

void CmdCli::funcTemp(CmdParser *myParser)
{
	byte adr[8];
	uint16_t temp;
#if 0
	if (myParser->getParamCount() == 0) {
		uint8_t adrt[8] = { 0x28, 0x65, 0x0E, 0xFD, 0x05, 0x00, 0x00, 0x4D };
		float temp;
		byte bus;

		bus = 0;
		ow->tempRead (bus, adrt, 0);
		delay(200);
		temp = ow->tempRead (bus, adrt, 1);
		Serial.print(temp / 16);
		Serial.println(F(" C"));

		return;
	}
#endif
	if (myParser->getParamCount() == 2) {
		curBus = atoi(myParser->getCmdParam(1));
		curAdr = atoi(myParser->getCmdParam(2));
	}
#if 0
	if (adr[1] > 10) {
		uint8_t adrt[8] = { 0x28, 0x65, 0x0E, 0xFD, 0x05, 0x00, 0x00, 0x4D };

		ow->tempRead (curBus, adrt, 0);
		delay(800);
		temp = ow->tempRead (curBus, adrt, 1);
		Serial.print((float)(temp / 16));
		Serial.println(F(" C"));
		return;
	}
#endif
	adr[0] = 0x28;
	adr[1] = curAdr;
	ow->adrGen(curBus, adr, adr[1]);
#if EXT_DEBUG
	if (debug) {
		int i;
		for (i = 0; i < 7; i++) {
			Serial.print(adr[i], HEX);
			Serial.write(' ');
		}
		Serial.println(adr[i], HEX);
	}
#endif
	ow->tempRead (curBus, adr, 0);
	delay(100);
	temp = ow->tempRead (curBus, adr, 1);
	Serial.print((float)(temp / 16));
	Serial.println(F(" C"));

}

void CmdCli::funcMode(CmdParser *myParser)
{
	if (myParser->getParamCount() > 0)
		swHdl.mode = atoi(myParser->getCmdParam(1));
	Serial.print(F(" mode="));
	Serial.print(swHdl.mode);
	if (myParser->getParamCount() > 1)
		debug = atoi(myParser->getCmdParam(2));
	if (myParser->getParamCount() > 2)
		swHdl.light_thr = atoi(myParser->getCmdParam(3));

	Serial.print(F(" debug="));
	Serial.print(debug);
	Serial.print(F(" light="));
	Serial.print(light);
	Serial.print(F(" light sensor="));
	Serial.print(light_sensor);
	Serial.print(F(" light thr ="));
	Serial.print(swHdl.light_thr);
	Serial.print(F(" sun="));
	Serial.print(sun);
}

void CmdCli::funcCfg(CmdParser *myParser)
{
	uint8_t i, len;
	byte adr[8], data[MAX_CFG_SIZE];
	char *c;

	c = myParser->getCmdParam(1);
	if (myParser->getParamCount() < 2 && *c == '?') {
		Serial.println (F("s: save"));
		Serial.println (F("w: write"));
		Serial.println (F(" : read"));
		return;
	}
	// channel first
	adr[1] = atoi(myParser->getCmdParam(1));
	if (adr[1] > 20) {
		adr[1] = adr[1] - 20;
		adr[0] = 0x28;
	}
	ow->adrGen(curBus, adr, adr[1]);
	if (myParser->getParamCount() == 1) {
		if (adr[0] == 0x28)
			Serial.println(F(" OFF |FACT |S |IO|TH|TL"));
		else
			Serial.println(F("R |R |R |SW 1  2  3  4  5  6  7 |CFG 1 2  3  4  5  6  7 |FEA|OFF|MAJ|MIN|TYP"));
		len = ow->ds2408CfgRead(curBus, adr, data);
		for (i = 0; i < len - 1; i++) {
			if (data[i] < 10)
				Serial.print(F("0"));
			Serial.print(data[i], HEX);
			Serial.print(F(" "));
		}
		Serial.println(data[i], HEX);
		return;
	}
	c = myParser->getCmdParam(2);
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
		ow->ds2408CfgWrite(curBus, adr, data, i);
	}
	if (*c == 's') {
		Serial.print(F("Cfg save ("));
		len = ow->ds2408CfgRead(curBus, adr, data);
		data[21] = 0x55;
		ow->ds2408CfgWrite(curBus, adr, data, len);
		Serial.print(len);
		Serial.print(F(")"));
	}
}

/*

Examples:
c 7 1 f8 1f ff ff WOHNEN
c 7 2 00 00 f8 00 18.6
c 7 4 07 e0 00 00 n.a.
c 7 6 08 61 07 e0 21.5

c 7 6 ff ff 00 1f kalt
c 7 8 07 e0 00 00 22.1

c 7 9 07 ff 00 00 CONTROL
c 7 c 07 e0 00 00 "WLAN AN"

c 7 4 00 00 07 e0 " 22.3"
c 7 2 07 e0 08 61 18.3
c 7 3 00 1e 00 00 HALLO
c 7 4 00 1e 00 00 IRENE
c 7 5 07 ff 00 00 11:42

c 7 a 84 10 00 00 11:42

c 7 8 07 ff 00 00 STURM
*/
void CmdCli::funcCmd(CmdParser *myParser)
{
	byte d;
	byte adr[8];
	char *txt;
	byte data[10];
	int i;

	adr[1] = me->atoh(myParser->getCmdParam(1), false);
	d = me->atoh(myParser->getCmdParam(2), false);
	ow->adrGen(curBus, adr, adr[1]);
#if 0
	ow->ds2408PioSet(curBus, adr, d);
#else
	data[0] = d;
#endif

	if (myParser->getParamCount() > 2) {
		for (i = 0; i < 4; i++) {
			d = me->atoh(myParser->getCmdParam(3 + i), false);
#if 0
			ow->ds2408PioSet(curBus, adr, d);
#else
			data[1 + i] = d;
#endif
		}
	}
	if (myParser->getParamCount() > 6) {
		txt = myParser->getCmdParam(7);
		for (i = 0; i < 8; i++) {
			if (txt[i] == 0) {
#if 1
				data[5 + i] = 0;
				ow->ds2408ChWrite(curBus, adr, data, 5 + i);
#else
				ow->ds2408PioSet(curBus, adr, 0);
#endif
				Serial.println('<');

				return;
			} else
#if 1
				data[5 + i] = txt[i];
#else
				ow->ds2408PioSet(curBus, adr, txt[i]);
#endif
			Serial.print(txt[i], HEX);
			Serial.print(' ');
		}
	}
	Serial.println();
}

void CmdCli::dumpSwTbl(void)
{
	byte i;
	union d_adr_8 dst;
	int size = 0;
	union s_adr src;

	Serial.println(F("= Switches ="));
	for (i = 0; i < MAX_SWITCHES; i++) {
		src.data = sw_tbl[i].src.data;
		dst.data = sw_tbl[i].dst.data;
		if (src.data == 0)
			continue;
		if (src.data == 0xffff && dst.data == 0xff)
			continue;
		size += sizeof(struct _sw_tbl);
		src.sa.res = 0;
		printSrc(src);
		Serial.print(F(" -> "));
		printDst8(dst);
		Serial.print(F(" ("));
		Serial.print(src.data, HEX);
		Serial.print(F(" | "));
		Serial.print(dst.data, HEX);
		Serial.println(F(")"));
	}
	Serial.print(F("Size="));
	Serial.print(size);
	Serial.print(F("/"));
	Serial.println((int)sizeof(sw_tbl));
	size = 0;
	Serial.println(F("= Timed ="));
	for (i = 0; i < MAX_TIMED_SWITCH; i++) {
		src.data = timed_tbl[i].src.data;
		dst.data = timed_tbl[i].dst.data;
		if (src.data == 0 || dst.data == 0)
			continue;
		if (src.data == 0xffff && dst.data == 0xff)
			continue;
		size += sizeof(struct _sw_tbl);
		src.sa.res = 0;
		Serial.print(F("Type "));
		Serial.print(timed_tbl[i].type);
		Serial.print(F(" "));
		printSrc(src);
		Serial.print(F(" -> "));
		printDst8(dst);
#if 0
		Serial.print(F(" ("));
		Serial.print(src.data, HEX);
		Serial.print(F(" | "));
		Serial.print(dst.data, HEX);
		Serial.print(F(")"));
#endif
		Serial.println();
	}
	Serial.print(F("Size="));
	Serial.print(size);
	Serial.print(F("/"));
	Serial.println((int)sizeof(timed_tbl));
}

/*
sw : dumps the table
sw r : reads the table from eeprom again
sw s : saves the table into eeprom
sw c : clears the table
sw s <bus> <adr> <latch> <dst bus> <dst adr> <dst pio> [type]
	switch table
sw t <timer type> <bus> <adr> <latch> <dst bus> <dst adr> <dst pio> [type]
	Timed table	timer type: see enum tim_type
latch 0 .. 7: normal press
latch 10 .. 17: long press
latch 20 .. 27: pressing
*/
void CmdCli::funcSwCmd(CmdParser *myParser)
{
	int i;
	uint16_t off;
	union s_adr src;
	union d_adr_8 dst;
	char *c;
	uint8_t ttype;

	if (myParser->getParamCount() == 0) {
		me->dumpSwTbl();
		return;
	}
	if (myParser->getParamCount() == 1) {
		c = myParser->getCmdParam(1);
#ifdef EXT_DEBUG
		if (*c == '?') {
			Serial.println (F("r: read"));
			Serial.println (F("s: save"));
			Serial.println (F("c: clear"));
			Serial.println (F("d: dump eeprom"));
			return;
		}
#endif
		switch (*c) {
			case 'r': 	// read eeprom
				swHdl.initSwTable();
				break;
			case 's':		// write eeprom
				swHdl.saveSwTable();
				break;
			case 'd':		// dump eeprom
			{
				uint8_t buf[60];

				eeprom_read_block((void*)buf, (const void*)0, 20);
				for (i = 0; i < 19; i++){
					Serial.print(buf[i], HEX);
					Serial.print(F(" "));
					if ((i % 20) == 0)
						Serial.println();
				}
				Serial.println(buf[i], HEX);

				break;
			}
			case 'c':
				memset (sw_tbl, 0, sizeof(sw_tbl));
				memset (timed_tbl, 0, sizeof(timed_tbl));
				break;
			default:
				Serial.println (F("?"));
		}
		// cmd like store to eeprom
		return;
	}
	src.data = 0;
	dst.data = 0;
	if (myParser->getParamCount() > 3) {
		uint8_t latch;

		off = 2;
		c = myParser->getCmdParam(1);
		/* s: switch table
		 * t: timed table
		 * d: delete switch (from all dsts)
		 */
		if (*c == 't') {
			ttype = atoi(myParser->getCmdParam(off++));
			Serial.print(F("Timer Type "));
			Serial.print(ttype);
			Serial.print(F(" "));
		}
		if (myParser->getParamCount() < (uint16_t)(off + 2))  {
			Serial.println(F("?"));
			Serial.print(myParser->getParamCount());
			Serial.print(F(" <> "));
			Serial.println((uint8_t)(off + 2));
			return;
		}
		src.sa.bus = atoi(myParser->getCmdParam(off++));
		src.sa.adr = atoi(myParser->getCmdParam(off++));
		latch = atoi(myParser->getCmdParam(off++));
		if (latch - 20 > 0) {
			/* pressing */
			src.sa.press = 2;
			src.sa.latch = latch - 20;
		} else if (latch - 10 > 0) {
			/* press long */
			src.sa.press = 1;
			src.sa.latch = latch - 10;
		} else {
			src.sa.latch = latch;
			src.sa.press = 0;
		}
		printSrc(src);
		Serial.print(" | ");
		Serial.println(src.data, HEX);
	}
	if (myParser->getParamCount() == 4) {
		if (*c == 'd') {
			/* select table ... */
			for (i = 0; i < MAX_SWITCHES; i++) {
				if (sw_tbl[i].src.data == src.data) {
					Serial.print(F("  deleting "));
					printDst8(sw_tbl[i].dst);
					Serial.println();
					/* move following or last to here */
					sw_tbl[i].src.data = 0;
					sw_tbl[i].dst.data = 0xff;
				}
			}
			for (i = 0; i < MAX_TIMED_SWITCH; i++) {
				if (timed_tbl[i].src.data == src.data) {
					Serial.print(F("  deleting "));
					printDst8(timed_tbl[i].dst);
					Serial.println();
					timed_tbl[i].src.data = 0;
					timed_tbl[i].dst.data = 0xff;
				}
			}
		}
		return;
	}

	// byte time;
	// 1:cmd, 2:bus, 3:adr, 4:latch
	dst.da.bus = atoi(myParser->getCmdParam(off++));
	dst.da.adr = atoi(myParser->getCmdParam(off++));
	dst.da.pio = atoi(myParser->getCmdParam(off++));
	/*if (myParser->getParamCount() > 7)
		time = atoi(myParser->getCmdParam(8));
	*/
	if (myParser->getParamCount() > off)
		dst.da.type = atoi(myParser->getCmdParam(off));
	switch (*c) {
		case 's':
			for (i = 0; i < MAX_SWITCHES; i++) {
				if (sw_tbl[i].src.data == 0 || sw_tbl[i].src.data == src.data) {
					sw_tbl[i].src.data = src.data;
					sw_tbl[i].dst.data = dst.data;
					break;
				}
			}
			break;
		case 't':
			for (i = 0; i < MAX_TIMED_SWITCH; i++) {
				if (timed_tbl[i].src.data == 0 || timed_tbl[i].src.data == src.data) {
					timed_tbl[i].src.data = src.data;
					timed_tbl[i].dst.data = dst.data;
					timed_tbl[i].type = ttype;
					break;
				}
			}
			break;
		default:
			Serial.println(F("?"));
			break;
	}
	me->dumpSwTbl();
}

extern uint8_t sec;
extern uint8_t min;
extern uint8_t hour;
extern uint8_t sun;
#ifdef EXT_DEBUG
void CmdCli::funcPinSet(CmdParser *myParser)
{
	uint8_t pin, val;

	pin = atoi(myParser->getCmdParam(1));
	Serial.print(F(" val: "));
	val = atoi(myParser->getCmdParam(2));
	Serial.println(val);
	if (pin == 5)
		analogWrite(5, val);
	else {
		pinMode(pin, OUTPUT);
		digitalWrite (pin, val);
	}
}

void CmdCli::funcPinGet(CmdParser *myParser)
{
	int pin, val;

	pin = atoi(myParser->getCmdParam(1));
	if (pin == 20) {
		ADCSRA = (1<<ADPS2) | (1<<ADPS1) | (1<<ADEN);
		delay(10);
		val = analogRead(A6);
		light = val;
		ADCSRA = 0;
	}
	else
		val = digitalRead(pin);
	Serial.print(F(" val: "));
	Serial.print(val);
}

#include "TwiHost.h"
extern TwiHost host;

void CmdCli::funcLog(CmdParser *myParser)
{
	struct logData d;
	union s_adr src;
	int i;

	(void)*myParser;
	for (i = 0; i < host.events.size(); i++) {
		d = host.events[i];
		src.data = d.source;
		Serial.print(d.h);
		Serial.print(F(":"));
		Serial.print(d.min);
		Serial.print(F(":"));
		Serial.print(d.sec);
		Serial.print(F(" "));
		Serial.print(d.type);
		Serial.print(F(" "));
		Serial.print(src.sa.bus, HEX);
		Serial.print(F("."));
		Serial.print(src.sa.adr, HEX);
		if (d.type == 0 || d.type == 1) {
			Serial.print(F("."));
			Serial.print(src.sa.latch, HEX);
		}
		Serial.print(F(" "));
		Serial.println(d.data);
	}
}
#endif

#ifdef CHGID_CMD
void CmdCli::funcChgId(CmdParser *myParser)
{
	byte id, bus, i;
	byte adr[8];
	byte newAdr[8];

	bus = me->atoh(myParser->getCmdParam(1), false);
	adr[1] = me->atoh(myParser->getCmdParam(2), false);
	// first address with bus and adr
	ow->adrGen(bus, adr, adr[1]);
	bus = me->atoh(myParser->getCmdParam(3), false);
	id = me->atoh(myParser->getCmdParam(4), false);
	// first address with bus and adr
	ow->adrGen(bus, newAdr, id);
#if EXT_DEBUG
	if (debug) {
		for (i = 0; i < 7; i++) {
			Serial.print(newAdr[i], HEX);
			Serial.write(' ');
		}
		Serial.println(newAdr[i], HEX);
	}
#endif
	// use current bus otherwise change wrong bus ids not working
	ds->selectChannel(curBus);
	ds->reset();
	ds->select(adr);
	// Master sendet Befehl Write-New-ID
	ds->write (0x75);
	// Tx	64-bit New ID	Master sendet neue ID
	for (i = 0; i < 8; i++)
		ds->write (newAdr[i]);

	ds->reset();
	// Tx	64-bit 1-Wire Alte-ID	Master sendet alte ID
	ds->select(adr);
	// Tx	0xA7	Master sendet Befehl Read-New-ID
	ds->write (0xA7);
	byte err = 0;
	for (i = 0; i < 8; i++) {
		byte d = ds->read();
		if (d != newAdr[i]) {
			err = 1;
			Serial.print("!! ");
			Serial.print(d, HEX);
			Serial.print(" <> ");
			Serial.println(newAdr[i], HEX);
		} else {
			Serial.print(d, HEX);
			Serial.write(' ');
		}
	}
	if (err == 0) {
		Serial.println("Success");
		ds->reset();
		// Tx	64-bit 1-Wire Alte-ID	Master sendet alte ID
		ds->select(adr);
		// Tx 0x79	Master sendet Befehl Read-New-ID
		ds->write (0x79);
		// TxByte 2 der alten ID	Master sendet das zweite Byte der alten ID
		ds->write (adr[1]);
		// Tx	Byte 6 der alten ID	Master sendet das 6. Byte der alten ID
		ds->write (adr[5]);
		//Tx Byte 7 der alten ID	Master sendet das 7. Byte der alten ID
		ds->write (adr[6]);
	}
}
#endif

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

