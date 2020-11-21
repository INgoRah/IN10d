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
#include "Arduino.h"  // according http://blog.makezine.com/2011/12/01/arduino-1-0-is-out-heres-what-you-need-to-know/

#include "DS2482.h"
#include "Wire.h"

#define PTR_STATUS 0xf0
#define PTR_READ 0xe1
#define PTR_CONFIG 0xc3

DS2482::DS2482(uint8_t addr)
{
	mAddress = 0x18 | addr;
}

//-------helpers
void DS2482::begin()
{
	Wire.beginTransmission(mAddress);
}

void DS2482::end()
{
	Wire.endTransmission();
}

void DS2482::setReadPtr(uint8_t readPtr)
{
	begin();
	Wire.write(0xe1);  // changed from 'send' to 'write' according http://blog.makezine.com/2011/12/01/arduino-1-0-is-out-heres-what-you-need-to-know/'
	Wire.write(readPtr);
	end();
}

uint8_t DS2482::_read()
{
	Wire.requestFrom(mAddress,(uint8_t)1);
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

	if (setPtr)
		setReadPtr(PTR_STATUS);
	while((res = _read()) & DS2482_STATUS_BUSY)
	{
		if (--loopCount == 0) {
			status = stTimeout;
			break;
		}
		delayMicroseconds(20);
	}
	return res;
}

//----------interface
void DS2482::resetDev()
{
	status = stOk;
	begin();
	Wire.write(0xf0);
	end();
}

bool DS2482::configureDev(uint8_t config)
{
	busyWait(true);
	begin();
	Wire.write(0xd2);
	Wire.write(config | (~config)<<4);
	end();
	
	return _read() == config;
}

bool DS2482::selectChannel(uint8_t channel)
{
	const byte R_chan[8] = { 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87 };
	uint8_t ch;

	switch (channel)
	{
		case 0:
		default:
			ch = 0xf0;
			break;
		case 1:
			ch = 0xe1;
			break;
		case 2:
			ch = 0xd2;
			break;
		case 3:
			ch = 0xc3;
			break;
		case 4:
			ch = 0xb4;
			break;
		case 5:
			ch = 0xa5;
			break;
		case 6:
			ch = 0x96;
			break;
		case 7:
			ch = 0x87;
			break;
	};

	busyWait(true);
	begin();
	Wire.write(0xc3);
	Wire.write(ch);
	end();
	busyWait();

	uint8_t check = _read();
 
	return check == R_chan[ch];
}

bool DS2482::reset()
{
	busyWait(true);
	begin();
	Wire.write(0xb4);
	end();

	uint8_t stat = busyWait();

	return stat & DS2482_STATUS_PPD ? true : false;
}

uint8_t DS2482::write(uint8_t b, uint8_t power)
{
	busyWait(true);
	begin();
	Wire.write(0xa5);
	Wire.write(b);
	end();

	return b;
}

uint8_t DS2482::read()
{
	busyWait(true);
	begin();
	Wire.write(0x96);
	end();
	busyWait();
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
#ifdef DEBUG
		Serial.print(".");
#endif		
		if (i < searchLastDisrepancy)
			direction = searchAddress[romByte] & romBit;
		else
			direction = i == searchLastDisrepancy;

		busyWait();
		if (status == stTimeout) {
			Serial.println("Search Timeout");
			return false;
		}
		begin();
		Wire.write(0x78);
		Wire.write(direction ? 0x80 : 0);
		end();
		stat = busyWait();
		if (status == stTimeout) {
			Serial.println("Search Timeout");
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

#ifdef DEBUG
	Serial.println();
#endif
	return true;
}
#endif
