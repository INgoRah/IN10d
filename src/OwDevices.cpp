/* LICENSE
 *
 */
/*
 * Includes
 */
#include "Arduino.h"			 // for delayMicroseconds, digitalPinToBitMask, etc
#include <avr/pgmspace.h>
#include "DS2482.h"
#include "OwDevices.h"

/*
 * Library classs includes
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

void adrGen(OneWireBase *ds, byte adr[8], uint8_t id);

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
	ds->resetDev();
	ds->configureDev(DS2482_CONFIG_APU);
}

void OwDevices::adrGen(OneWireBase *ds, uint8_t bus, uint8_t adr[8], uint8_t id)
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
	adr[7] = ds->crc8 (adr, 7);
#if 0
	int i;
	for (i = 0; i < 7; i++) {
		Serial.print(adr[i], HEX);
		Serial.print(F(' '));
	}
	Serial.println(adr[i], HEX);
#endif
}

int OwDevices::search(OneWireBase *ds, byte bus)
{
	byte adr[8];
	byte j = 0;

	ds->selectChannel(bus);
	ds->reset_search();
	if (ds->reset() == 0) {
		return -1;
	}

	while (ds->search(adr)) {
		Serial.print(F("#"));
		Serial.print(j++);
		Serial.print(F(":"));
		for (int i = 0; i < 8; i++) {
			Serial.print(F(" "));
			Serial.print(adr[i], HEX);
		}
		Serial.println();
	}

	return j;
}

/* requires bus selected already before */
uint8_t OwDevices::ds2408LatchReset(OneWireBase *ds, uint8_t* addr)
{
	uint8_t retry;
	uint8_t tmp;

#if defined(AVRSIM)
	return 0xAA;
#endif
	retry = 5;
	do {
		ds->reset();
		ds->select(addr);
		ds->write (0xC3);
		tmp = ds->read();
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
uint8_t OwDevices::ds2408RegRead(OneWireBase *ds, byte bus, uint8_t* addr, uint8_t* data, bool latch_reset)
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
		ds->selectChannel(bus);
		ds->reset();
		ds->select(addr);
		// read data registers
		buf[0] = 0xF0;    // Read PIO Registers
		buf[1] = 0x88;    // LSB address
		buf[2] = 0x00;    // MSB address
		ds->write (buf, 3);
		// 3 cmd bytes, 6 data bytes, 2 0xFF, 2 CRC16
		// 1:
		ds->read (data, 10);
		/* check for valid status register */
		if (data[5] != 0xff)
			break;
	} while (retry-- > 0);
	if (data[5] == 0xff)
		return 0xff;
	if (!latch_reset)
		return 0xaa;

	tmp = ds2408LatchReset(ds, addr);
	if (tmp != 0xAA)
		Serial.println(F("latch reset error"));

	return tmp;
}

uint8_t OwDevices::ds2408ChWrite(OneWireBase *ds, byte bus, uint8_t* addr, uint8_t* data, int cnt)
{
	uint8_t r;
	int i;

	ds->selectChannel(bus);
	ds->reset();
	ds->select(addr);
	ds->write (0x5A);
	Serial.print(F("data"));
	for (i = 0; i < cnt; i++) {
		Serial.print(F(" "));
		Serial.print(data[i], HEX);
		Serial.print(F(" "));
		ds->write (data[i]);
		r = ds->read();
		if (r != 0xAA) {
			// should be data[i] ... but the implementation seems wrong
			Serial.print(r, HEX);
		}
	}

	return 0xaa;
}

void OwDevices::toggleDs2413(OneWireBase *ds, byte bus, uint8_t* addr)
{
	unsigned char pio, pion;
	int cnt = 10;

	ds->selectChannel(bus);
	do {
		ds->reset();
		ds->select(addr);
		ds->write (0xF5);
		pio = ds->read();
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
		ds->reset();
		ds->select(addr);
		ds->write (0x5A);
		ds->write (0xFC | pion);
		ds->write (0xFF & (~(0xFC | pion)));
		pio = ds->read();
		if (cnt-- == 0)
			return;
	} while (pio != 0xAA);
}

uint8_t OwDevices::ds2408PioGet(OneWireBase *ds, byte bus, uint8_t* addr)
{
	uint8_t pio;
	uint8_t buf[3];

#if defined(AVRSIM)
	return 0x56;
#endif
	ds->selectChannel(bus);
	ds->reset();
	ds->select(addr);
	// read data registers
	buf[0] = 0xF0;    // Read PIO Registers
	buf[1] = 0x89;    // LSB address
	buf[2] = 0x00;    // MSB address
	ds->write (buf, 3);
	pio = ds->read();

	return pio;
}

uint8_t OwDevices::ds2408PioSet(OneWireBase *ds, byte bus, uint8_t* addr, uint8_t pio)
{
#if defined(AVRSIM)
	return 0xAA;
#else
	uint8_t r;

	ds->selectChannel(bus);
	ds->reset();
	ds->select(addr);
	ds->write (0x5A);
	ds->write (pio);
	ds->write (0xFF & ~(pio));
	r = ds->read();
	if (r != 0xAA) {
		Serial.print(F("data "));
		Serial.print(pio, HEX);
		Serial.println(F(" write error"));
	} else
		ds2408LatchReset(ds, addr);

	return r;
#endif
}

/**
 * pio - bit mask of the PIO
 * data - optional data from a read before, avoid reading again
 * */
uint8_t OwDevices::ds2408TogglePio(OneWireBase *ds, byte bus, uint8_t* addr, uint8_t pio, uint8_t* data)
{
	uint8_t d;

	if (data == NULL)
		d = ds2408PioGet(ds, bus, addr);
	else
		d = data[1];
	if (d & pio)
		d &= ~(pio);
	else
		d |= pio;

	d = ds2408PioSet(ds, bus, addr, d);

	return d;
}

/**
 * data - array of min 24 items
 */
int OwDevices::ds2408CfgRead(OneWireBase *ds, byte bus, byte adr[8], uint8_t* data)
{
	int i, len = MAX_CFG_SIZE;

	ds->selectChannel(bus);
	ds->reset();
	ds->select(adr);
	ds->write (0x85);

	for (i = 0; i < len - 1; i++)
		data[i] = ds->read ();

	return len;
}

void OwDevices::ds2408CfgWrite(OneWireBase *ds, byte bus, byte adr[8], uint8_t* d, uint8_t len)
{
	int i;

	ds->selectChannel(bus);
	ds->reset();
	ds->select(adr);
	ds->write (0x86);
	for (i = 0; i < len; i++)
		ds->write(d[i]);
}

/**
 * Returns temperature raw. celsius = tempRead * 16
 * mode 0: start conversion
 * 1: set alarms and reset
 * 2: read temperature and scratchpad
*/
uint16_t OwDevices::tempRead(OneWireBase *ds, byte busNr, byte addr[8], byte mode)
{
	uint8_t scratchPad[9];

	ds->selectChannel(busNr);
	ds->reset();
	ds->select(addr);
	switch (mode) {
	case 0:
		ds->write(STARTCONVO);
		return 0;
	case 2:
		ds->write(WRITESCRATCH);
		ds->write(35); // high alarm temp
		ds->write(10); // low alarm temp
		return 0;
	case 1:
	default:
		/* read temp */
		break;
	}
	ds->write(READSCRATCH);
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
		scratchPad[i] = ds->read();
		Serial.print(scratchPad[i], HEX);
		Serial.print(F(" "));
	}
	Serial.println();
#else
	for (uint8_t i = 0; i < 9; i++)
		scratchPad[i] = ds->read();
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
