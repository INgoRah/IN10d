/* LICENSE
 *
 */
/*
 * Includes
 */
#include "Arduino.h"			 // for delayMicroseconds, digitalPinToBitMask, etc
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include "DS2482.h"
#include "OwDevices.h"

/*
 * Library class includes
 */

/*
 * Local constants
 */
#define READSCRATCH     0xBE  // Read from scratchpad
#define WRITESCRATCH    0x4E  // Write to scratchpad
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3

#if defined(AVRSIM)
uint8_t	pio_data[0x0f];
#endif

void adrGen(byte adr[8], uint8_t id);

/*
 * Objects
 */

/*
 * Local variables
 */

/*
* Function declarations
*/
void OwDevices::begin(OneWireBase *ds)
{
	ow = ds;
	ow->resetDev();
	ow->configureDev(DS2482_CONFIG_APU);
}

void OwDevices::adrGen(uint8_t bus, uint8_t adr[8], uint8_t id)
{
	adr[1] = id; // id selector
	adr[2] = bus;
	adr[3] = (uint8_t)~id;
	adr[4] = (uint8_t)~bus;
	if (adr[0] != 0x28) {
		/* set me up */
		adr[0] = 0x29; // type
	}
	adr[5] = 0x66;
	adr[6] = 0x77;
	adr[7] = ow->crc8 (adr, 7);
#if 0
	int i;
	for (i = 0; i < 7; i++) {
		Serial.print(adr[i], HEX);
		Serial.print(F(' '));
	}
	Serial.println(adr[i], HEX);
#endif
}

/* requires bus selected already before */
uint8_t OwDevices::ds2408LatchReset(uint8_t* addr)
{
	uint8_t retry;
	uint8_t tmp;

#if defined(AVRSIM)
	return 0xAA;
#endif
	retry = 5;
	do {
		ow->reset();
		ow->select(addr);
		ow->write (0xC3);
		tmp = ow->read();
		if (tmp == 0xAA)
			break;
	} while (retry-- > 0);

	return tmp;
}

/* Read DS2408 registers
 * [0] PIO Logic State
 * [1] Output latch
 * [2] Activity latch state
 * [3] Conditional search channel selection (unused)
 * [4] Conditional search polarity selection (unused)
 * [5] Status
 * [6] Status ext 1, used for press time detection
 * [7] Status ext 2
 * */
uint8_t OwDevices::ds2408RegRead(byte bus, uint8_t* addr, uint8_t* data, bool latch_reset)
{
	uint8_t tmp;
	uint8_t buf[3];  // Put everything in the buffer so we can compute CRC easily.

#if defined(AVRSIM)
	data[1] = 0x55;
	return 0xAA;
#endif
	uint8_t retry = REG_RETRY;
	do {
		/* read latch */
		ow->selectChannel(bus);
		ow->reset();
		ow->select(addr);
		// read data registers
		buf[0] = 0xF0;    // Read PIO Registers
		buf[1] = 0x88;    // LSB address
		buf[2] = 0x00;    // MSB address
		ow->write (buf, 3);
		// 3 cmd bytes, 6 data bytes, 2 0xFF, 2 CRC16
		// 1:
		ow->read (data, 10);
		/* check for valid status register */
		if (data[5] != 0xff)
			break;
	} while (retry-- > 0);
	if (data[5] == 0xff)
		return 0xff;
	if (!latch_reset)
		return 0xaa;

	tmp = ds2408LatchReset(addr);
	if (tmp != 0xAA)
		Serial.println(F("latch reset error"));

	return tmp;
}

uint8_t OwDevices::ds2408ChWrite(byte bus, uint8_t* addr, uint8_t* data, int cnt)
{
	uint8_t r;
	int i;

	ow->selectChannel(bus);
	ow->reset();
	ow->select(addr);
	ow->write (0x5A);
	for (i = 0; i < cnt; i++) {
		ow->write (data[i]);
		r = ow->read();
		if (r != 0xAA) {
			// should be data[i] ... but the implementation seems wrong
			Serial.print(r, HEX);
		}
	}

	return 0xaa;
}

void OwDevices::toggleDs2413(byte bus, uint8_t* addr)
{
	unsigned char pio, pion;
	int cnt = 10;

	ow->selectChannel(bus);
	do {
		ow->reset();
		ow->select(addr);
		ow->write (0xF5);
		pio = ow->read();
		if (cnt-- == 0)
			return;
	} while (((pio & 0xF) != (~(pio >> 4) & 0xF)));
	if ((pio & 2) == 0)
		pion = 3;
	else
		pion = 2;
	pion |= 0x2;
	cnt = 10;
	do {
		ow->reset();
		ow->select(addr);
		ow->write (0x5A);
		ow->write (0xFC | pion);
		ow->write (0xFF & (~(0xFC | pion)));
		pio = ow->read();
		if (cnt-- == 0)
			return;
	} while (pio != 0xAA);
}

/* reads and returns the Output latch state register

TODO make sure, that the data is correct ... how? */
uint8_t OwDevices::ds2408PioGet(byte bus, uint8_t* addr)
{
	uint8_t res, d[10];

#if defined(AVRSIM)
	return pio_data[addr[1] & 0x0f];
#endif

	res = ds2408RegRead(bus, addr, d, false);
	if (res != 0xaa) {
		Serial.print("Read error: ");
		Serial.println(res, HEX);
	}

	return d[1];
#if 0
	uint8_t buf[3];
	uint8_t pio;

	ow->selectChannel(bus);
	ow->reset();
	ow->select(addr);
	// read data registers
	buf[0] = 0xF0;    // Read PIO Registers
	buf[1] = 0x89;    // LSB address
	buf[2] = 0x00;    // MSB address
	ow->write (buf, 3);
	pio = ow->read();

	return pio;
#endif
}

uint8_t OwDevices::ds2408PioSet(byte bus, uint8_t* addr, uint8_t pio)
{
#if defined(AVRSIM)
	pio_data[addr[1] & 0x0f] = pio;
	return 0xAA;
#else
	uint8_t r, retry;

	ow->selectChannel(bus);
	retry = 5;
	do {
		wdt_reset();
		ow->reset();
		ow->select(addr);
		ow->write (0x5A);
		ow->write (pio);
		ow->write (0xFF & ~(pio));
		r = ow->read();

		if (r == 0xAA)
			break;
	} while (retry-- > 0);

	if (r != 0xAA) {
		Serial.print(F("data "));
		Serial.print(pio, HEX);
		Serial.println(F(" write error"));
	} else
		ds2408LatchReset(addr);

	return r;
#endif
}

/**
 * data - array of min 24 items
 */
int OwDevices::ds2408CfgRead(byte bus, byte adr[8], uint8_t* data)
{
	int i, len = MAX_CFG_SIZE;

	ow->selectChannel(bus);
	ow->reset();
	ow->select(adr);
	ow->write (0x85);

	for (i = 0; i < len - 1; i++)
		data[i] = ow->read ();

	return len;
}

void OwDevices::ds2408CfgWrite(byte bus, byte adr[8], uint8_t* d, uint8_t len)
{
	int i;

	ow->selectChannel(bus);
	ow->reset();
	ow->select(adr);
	ow->write (0x86);
	for (i = 0; i < len; i++)
		ow->write(d[i]);
}

/**
 * Returns temperature raw. celsius = tempRead * 16
 * mode 0: start conversion
 * 1: set alarms and reset
 * 2: read temperature and scratchpad
*/
int16_t OwDevices::tempRead(byte busNr, byte addr[8], byte mode)
{
	uint8_t scratchPad[9];

	ow->selectChannel(busNr);
	ow->reset();
	ow->select(addr);
	switch (mode) {
	case 0:
		ow->write(STARTCONVO);
		return 0;
	case 2:
		ow->write(WRITESCRATCH);
		ow->write(35); // high alarm temp
		ow->write(10); // low alarm temp
		return 0;
	case 1:
	default:
		/* read temp */
		break;
	}
	ow->write(READSCRATCH);
	// Read all registers in a simple loop
	// byte 0: temperature LSB
	// byte 1: temperature MSB
	// byte 2: high alarm temp
	// byte 3: low alarm temp
	// byte 4: DS18S20: store for crc
	//         DS18B20 & DS1822: configuration register
	// byte 5: internal use & crc
	// byte 6: DS18S20: COUNT_REMAIN
	//         DS18B20 & DS1822: store for crc
	// byte 7: DS18S20: COUNT_PER_C
	//         DS18B20 & DS1822: store for crc
	// byte 8: SCRATCHPAD_CRC
#if 0
	Serial.print(F(" "));
	for (uint8_t i = 0; i < 9; i++) {
		scratchPad[i] = ow->read();
		Serial.print(scratchPad[i], HEX);
		Serial.print(F(" "));
	}
	Serial.println();
#else
	for (uint8_t i = 0; i < 9; i++)
		scratchPad[i] = ow->read();
#endif
	int16_t raw = (scratchPad[1] << 8) | scratchPad[0];
#define cfg  (scratchPad[4] & 0x60)

	// at lower res, the low bits are undefined, so let's zero them
	if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
	else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
	else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
	//// default is 12 bit resolution, 750 ms conversion time
	// to be done by caller if needed
  	// celsius = (float)raw / 16.0;

	return raw;
}
