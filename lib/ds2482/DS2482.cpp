/*
  DS2482 library for Arduino
  Copyright (C) 2009-2010 Paeae Technologies
  Copyright (C) 2020 INgo.Rah@gmx.net

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have readd a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

	crc code is from OneWire library

	-Updates:
		* fixed wireread busyWait (thanks Mike Jackson)
		* Modified search function (thanks Gary Fariss)
		* adapted the class/function layout to have a common function API with OneWire.h

  https://github.com/paeaetech/paeae.git
*/
#include <main.h>
#include "Arduino.h"  // according http://blog.makezine.com/2011/12/01/arduino-1-0-is-out-heres-what-you-need-to-know/

#include "DS2482.h"
#include "Wire.h"

#define PTR_STATUS 0xf0
#define PTR_READ 0xe1
#define PTR_CONFIG 0xc3

DS2482::DS2482(uint8_t addr)
{
	mAddress = 0x18 | addr;
	ch = 0xff;
}

//-------helpers
void DS2482::begin()
{
	status = stOk;
	// timeout 10 ms
	Wire.setWireTimeout(10000, true);
	Wire.beginTransmission(mAddress);
}

void DS2482::end()
{
	uint8_t ret;

	ret = Wire.endTransmission();
	if (ret)
		status = stTimeout;
}


/* This function is called only from this class
   Error handling (bus error from end) needs
   to be handled inside by checking the status
   member
*/
void DS2482::setReadPtr(uint8_t readPtr)
{
	begin();
	Wire.write(0xe1);  // changed from 'send' to 'write' according http://blog.makezine.com/2011/12/01/arduino-1-0-is-out-heres-what-you-need-to-know/'
	Wire.write(readPtr);
	end();
}

uint8_t DS2482::_read()
{
	uint8_t ret;

	/* this can result in a timeout (ret == 0) */
	ret = Wire.requestFrom(mAddress,(uint8_t)1);
	if (ret == 0) {
		status = stTimeout;
		return 0xff;
	}

	return Wire.read();
}

uint8_t DS2482::wireReadStatus(bool setPtr)
{
	if (setPtr)
		setReadPtr(PTR_STATUS);

	return _read();
}

uint8_t DS2482::busyWait(bool setPtr)
{
	uint8_t res;
	int loopCount = 500;

	if (setPtr) {
		setReadPtr(PTR_STATUS);
		if (status == stTimeout) {
			log_time();
			Serial.println(F("ReadPtr TO"));
			return DS2482_STATUS_INVAL;
		}
	} else
		/* in the other case this is done in the begin */
		status = stOk;

	while((res = _read()) & DS2482_STATUS_BUSY)
	{
		if (--loopCount == 0 || status == stTimeout) {
			log_time();
			Serial.println(F("BusyWait TO"));
			status = stTimeout;
			return DS2482_STATUS_INVAL;
		}
		delayMicroseconds(20);
	}
	return res;
}

//----------interface
void DS2482::resetDev()
{
	ch = 0xff;
	begin();
	Wire.write(0xf0);
	end();
}

bool DS2482::configureDev(uint8_t config)
{
	ch = 0xff;
	busyWait(true);
	begin();
	Wire.write(0xd2);
	Wire.write(config | (~config)<<4);
	end();

	return _read() == config;
}

// channel must be between 0 and 7
bool DS2482::selectChannel(uint8_t channel)
{
	static const byte chan_r[8] = { 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87 };
	static const byte chan_w[8] = { 0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87 };

	if (ch == channel)
		return true;
	if (busyWait(true) == DS2482_STATUS_INVAL)
		return false;
	begin();
	Wire.write(0xc3);
	Wire.write(chan_w[channel]);
	end();
	if (status == stTimeout) {
		Serial.println("selCh TO");
		return false;
	}
	/* after channel selection the read pointer points
	to the channel register */

	uint8_t check = _read();
	if (check != chan_r[channel])
		return false;

	ch = channel;
	return true;
}

bool DS2482::reset()
{
	if (busyWait(true) == DS2482_STATUS_INVAL)
		return false;
	begin();
	Wire.write(0xb4);
	end();
	if (status == stTimeout) {
		Serial.println(F("reset (2) TO"));
		return false;
	}

	uint8_t stat = busyWait();

	return stat & DS2482_STATUS_PPD ? true : false;
}

uint8_t DS2482::write(uint8_t b, uint8_t power)
{
	(void)power;
	if (busyWait(true) == DS2482_STATUS_INVAL)
		return 0xff;
	begin();
	Wire.write(0xa5);
	Wire.write(b);
	end();
	if (status == stTimeout) {
		Serial.println(F("Write Err"));
		return 0xff;
	}

	return b;
}

uint8_t DS2482::read()
{
	if (busyWait(true) == DS2482_STATUS_INVAL)
		return 0xff;
	begin();
	Wire.write(0x96);
	end();
	if (status == stTimeout) {
		Serial.println(F("Read Err (1)"));
		return 0xff;
	}
	if (busyWait(true) == DS2482_STATUS_INVAL)
		return 0xff;
	setReadPtr(PTR_READ);
	return _read();
}

void DS2482::wireWriteBit(uint8_t bit)
{
	busyWait(true);
	begin();
	Wire.write(0x87);
	Wire.write(bit ? 0x80 : 0);
	end();
}

uint8_t DS2482::wireReadBit()
{
	wireWriteBit(1);
	uint8_t stat = busyWait(true);
	return stat & DS2482_STATUS_SBR ? 1 : 0;
}

void DS2482::skip()
{
	write(OW_SKIP_ROM);
}

void DS2482::select(const uint8_t rom[8])
{
	write(OW_MATCH_ROM);
	for (int i=0;i<8;i++)
		write(rom[i]);
}


#if ONEWIRE_SEARCH
void DS2482::reset_search()
{
	searchExhausted = 0;
	searchLastDisrepancy = 0;

	for(uint8_t i = 0; i<8; i++)
		searchAddress[i] = 0;
	status = stOk;
}

bool DS2482::search(uint8_t *newAddr, bool search_mode)
{
	uint8_t i;
	uint8_t direction;
	uint8_t last_zero=0;
	uint8_t stat;

	if (searchExhausted)
		return false;

	if (!reset())
		return false;

	busyWait();
	if (search_mode == true)
		// NORMAL SEARCH
		write(OW_SEARCH_ROM);
	else
		// CONDITIONAL SEARCH
		write(OW_COND_SEARC_ROM);

	for(i=1;i<65;i++)
	{
		int romByte = (i-1)>>3;
		int romBit = 1<<((i-1)&7);

		if (i < searchLastDisrepancy)
			direction = searchAddress[romByte] & romBit;
		else
			direction = i == searchLastDisrepancy;

		busyWait();
		if (status == stTimeout) {
			return false;
		}
		begin();
		Wire.write(0x78);
		Wire.write(direction ? 0x80 : 0);
		end();
		stat = busyWait();
		if (status == stTimeout) {
			return false;
		}

		uint8_t id = stat & DS2482_STATUS_SBR;
		uint8_t comp_id = stat & DS2482_STATUS_TSB;
		direction = stat & DS2482_STATUS_DIR;

		if (id && comp_id) {
			// no devices on 1-wire
			status = stNoDevs;
			return false;
		}
		else
		{
			if (!id && !comp_id && !direction)
				last_zero = i;
		}

		if (direction)
			searchAddress[romByte] |= romBit;
		else
			searchAddress[romByte] &= (uint8_t)~romBit;
	}

	searchLastDisrepancy = last_zero;

	if (last_zero == 0)
		searchExhausted = 1;

	for (i=0;i<8;i++)
		newAddr[i] = searchAddress[i];

	return true;
}
#endif
