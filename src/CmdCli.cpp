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

#define DEBUG
/*
 * Local constants
 */
#define MAX_CMD_BUFSIZE 16

extern byte mode;
extern OneWireBase *ds;
extern OneWireBase* bus[];

extern bool alarm_handler(OneWireBase *ds);

/*
 * Objects
 */

/*
 * Local variables
 */
static boolean stringComplete = false;  // whether the string is complete
static String inputString = "";         // a String to hold incoming data
static OwDevices* ow;

extern void wdtCommand(uint8_t cmd, uint8_t data);

/*
* Function declarations
*/
void CmdCli::begin(OwDevices* devs)
{
	ow = devs;
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
	} else {
		Serial.print("status=");
		Serial.println(ds->status);
		Serial.print("address=");
		Serial.println(ds->mAddress, HEX);
	}
}

void CmdCli::funcSearch(CmdParser *myParser)
{
	ow->search(ds);
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
		ow->ds2408Status (ds, adr, res);
	} else
		ow->statusRead(ds);
}

void CmdCli::funcPio(CmdParser *myParser)
{
	uint8_t pio, i;
	byte adr[8], data[10];
	bool res;

	// channel first
	adr[1] = atoi(myParser->getCmdParam(1));
	pio = atoi(myParser->getCmdParam(2));
	if (myParser->getParamCount() > 1)
		res = atoi(myParser->getCmdParam(3));
	else
		res = true;
	switch (adr[1]) {
		case 10:
			static uint8_t target[8] = { 0x3A, 0x01, 0xDA, 0x84, 0x00, 0x00, 0x05, 0xA3 };
			ow->toggleDs2413 (ds, target);
			break;
		default:
			ow->adrGen(ds, adr, adr[1]);
			ow->ds2408RegRead(ds, adr, data, false);
			for (i = 0; i < 9; i++) {
				Serial.print(data[i], HEX);
				Serial.print(F(" "));
			}
			Serial.println(data[i], HEX);
			ow->ds2408TogglePio(ds, adr, pio, data);
			ow->ds2408Status (ds, adr, res);
	}
}

void CmdCli::funcData(CmdParser *myParser)
{
	uint8_t len;
	byte adr[8];

	adr[1] = atoi(myParser->getCmdParam(1));
	len = atoi(myParser->getCmdParam(2));
	ow->ds2408Data(ds, adr, len);
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
		alarm_handler(bus[i]);
}

void CmdCli::funcCfg(CmdParser *myParser)
{
	uint8_t i;
	byte adr[8], data[24];

	// channel first
	adr[1] = atoi(myParser->getCmdParam(1));
	
	for (i = 0; i < 24;i++) {
		if (myParser->getParamCount() > (uint8_t)(1 + i))
			data[i] = atoi(myParser->getCmdParam(2+i));
		else
			break;
	}
	ow->adrGen(ds, adr, adr[1]);
	ow->ds2408Cfg(ds, adr, data, i);
}

void CmdCli::funcCmd(CmdParser *myParser)
{
	byte cmd, data;

	cmd = atoi(myParser->getCmdParam(1));
	if (myParser->getParamCount() > 1)
		data = atoi(myParser->getCmdParam(2));
	wdtCommand (cmd, data);
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

