/* LICENSE
 *
 */
/*
 * Includes
 */
#if ARDUINO >= 100
#include "Arduino.h"			 // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"			// for delayMicroseconds
#include "pins_arduino.h"	// for digitalPinToBitMask, etc
#endif
#include <avr/pgmspace.h>
#include "OwDevices.h"

/*
 * Library classs includes
 */

/*
 * Local constants
 */

void owStatusRead(OneWireBase *ds);
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

void OwDevices::loop()
{
}

void OwDevices::adrGen(OneWireBase *ds, uint8_t bus, uint8_t adr[8], uint8_t id)
{
	int i;

	adr[1] = id; // id selector
	adr[2] = bus;
	adr[3] = (uint8_t)~id;
	adr[4] = (uint8_t)~bus;
	if (adr[5] != 0x66) {
		/* set me up */
		adr[0] = 0x29; // type
		adr[5] = 0x66;
		adr[6] = 0x77;
	}
	adr[7] = ds->crc8 (adr, 7);
	for (i = 0; i < 7; i++) {
		Serial.print(adr[i], HEX);
		Serial.write(' ');
	}
	Serial.println(adr[i], HEX);
}

void OwDevices::search(OneWireBase *ds, byte bus)
{
	byte adr[8];
	byte j = 0;

	ds->selectChannel(bus);
	ds->reset_search();
	if (ds->reset() == 0) {
		Serial.println(F("no devs!"));
		return;
	} 
	Serial.println(F("success!"));

	while (ds->search(adr)) {
		Serial.print("#");
		Serial.print(j++);
		Serial.print(":");
		for (int i = 0; i < 8; i++) {
			Serial.write(' ');
			Serial.print(adr[i], HEX);
		}
		Serial.println();
	}
	Serial.print(j);
	Serial.println(" sensors found");
}

uint8_t OwDevices::ds2408RegRead(OneWireBase *ds, byte bus, uint8_t* addr, uint8_t* data, bool latch_reset)
{
	uint8_t tmp;
	uint8_t buf[13];  // Put everything in the buffer so we can compute CRC easily.
	uint8_t retry;

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
	ds->read (data, 10);

	if (!latch_reset)
		return 0xaa;
	retry = 5;
	do {
		ds->reset();
		ds->select(addr);
		ds->write (0xC3);
		tmp = ds->read();
		if (tmp == 0xAA)
			break;
	} while (retry-- > 0);
	if (tmp != 0xAA)
		Serial.println(F("latch reset error"));

	return tmp;
}

void OwDevices::ds2408Status(OneWireBase *ds, byte bus, byte adr[8], bool latch_reset)
{
	byte data[10], i;

	adrGen (ds, bus, adr, adr[1]);
	Serial.print(adr[1], HEX);
	Serial.print("..");
	Serial.print(adr[6], HEX);
	Serial.print(' ');
	Serial.println(adr[7], HEX);
	ds->selectChannel(bus);
	ds2408RegRead(ds, bus, adr, data, latch_reset);
	Serial.print(F("Data "));
	for (i = 0; i < 9; i++) {
		Serial.print(data[i], HEX);
		Serial.print(F(" "));
	}
	Serial.println(data[i], HEX);
	uint16_t crc = ds->crc16(data, 11, 0);
   	Serial.print(F("CRC calc "));
	Serial.println(crc, HEX);
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

uint8_t OwDevices::ds2408TogglePio(OneWireBase *ds, byte bus, uint8_t* addr, uint8_t pio, uint8_t* data)
{
	uint8_t d, r, retry;
	uint8_t buf[3];  // Put everything in the buffer so we can compute CRC easily.

	ds->selectChannel(bus);
	if (data == NULL) {
		ds->reset();
		ds->select(addr);
		// read data registers
		buf[0] = 0xF0;    // Read PIO Registers
		buf[1] = 0x89;    // LSB address
		buf[2] = 0x00;    // MSB address
		ds->write (buf, 3);
		//delayMicroseconds (100);
		d = ds->read();
	} else {
		d = data[1];
		Serial.print("change from ");
		Serial.print(d, HEX);
		Serial.println("..");
	}

	if (d & pio)
		d &= ~(pio);
	else
		d |= pio;

	ds->reset();
	ds->select(addr);
	ds->write (0x5A);
	ds->write (d);
	ds->write (0xFF & ~(d));
	//delayMicroseconds (100);
	r = ds->read();
	if (r != 0xAA) {
		Serial.println(F("data write error"));
		Serial.print(d, HEX);
		Serial.print(' ');
		Serial.print(0xFF & ~(d), HEX);
		Serial.print(' ');
		Serial.println(r, HEX);
	}
	retry = 5;
	do {
		ds->reset();
		ds->select(addr);
		ds->write (0xC3);
		if (ds->read() == 0xAA)
			break;
	} while (retry-- > 0);

	return r;
}

void OwDevices::ds2408Data(OneWireBase *ds, byte bus, byte adr[8], uint8_t len)
{
	int i;

	adrGen(ds, bus, adr, adr[1]);
	ds->selectChannel(bus);
	ds->reset();
	ds->select(adr);
	// Read data
	ds->write (0xF5);
	Serial.print(F("Data "));
	for (i = 0; i < len - 1; i++) {
		Serial.print(ds->read (), HEX);
		Serial.print(F(" "));
	}
	Serial.println(ds->read (), HEX);
}

void OwDevices::ds2408Cfg(OneWireBase *ds, byte bus, byte adr[8], uint8_t* d, uint8_t len)
{
	int i;
	uint8_t data[24];

	ds->selectChannel(bus);
	ds->reset();
	ds->select(adr);
	ds->write (0x85);

	Serial.print(F("Cfg Data Read  "));
	for (i = 0; i < 23; i++) {
		data[i] = ds->read ();
		Serial.print(data[i], HEX);
		Serial.print(F(" "));
	}
	data[i] = ds->read ();
	Serial.println(data[i], HEX);
	for (i = 0; i < len; i++) {
		data[i] = d[i];
	}
	ds->reset();
	ds->select(adr);
	ds->write (0x75);
	for (i = 0; i < len; i++)
		ds->write(data[i]);
	Serial.print(F("Cfg Data write ("));
	Serial.print(len);
	Serial.print(") ");
	for (i = 0; i < 23; i++) {
		Serial.print(data[i], HEX);
		Serial.print(F(" "));
	}
	Serial.println(data[i], HEX);
}

void OwDevices::statusPrint(OneWireBase *ds, byte adr[8])
{
	byte data[10], i;

	Serial.print(adr[1], HEX);
	Serial.print("..");
	Serial.print(adr[6], HEX);
	Serial.print(' ');
	Serial.println(adr[7], HEX);
	ds2408RegRead(ds, 1, adr, data);
	Serial.print(F("Data "));
	for (i = 0; i < 7; i++) {
		Serial.print(data[i], HEX);
		Serial.print(F(" "));
	}
	Serial.println(data[i], HEX);
}

void OwDevices::statusRead(OneWireBase *ds)
{
	byte adr[8];

	Serial.println(F("Data read"));

	adr[1] = 0x6;
	adr[7] = ds->crc8 (adr, 7);
	statusPrint(ds, adr);

	adr[1] = 0x3;
	adr[7] = ds->crc8 (adr, 7);
	statusPrint(ds, adr);
#if 0
	adr[1] = 0xA2;
	adr[2] = 0xd9;
	adr[3] = 0x84;
	adr[4] = 0x0;
	adr[5] = 0x16;
	adr[6] = 0x4;
	adr[7] = ds->crc8 (adr, 7);
	statusPrint(ds, adr);

	adr[6] = 0x2;
	adr[7] = ds->crc8 (adr, 7);
	statusPrint(ds, adr);
#endif
}
