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
	ch = 0xff;
	last_err = 0;
}

//-------helpers
void DS2482::begin()
{
	status = stOk;
	last_err = 0;
	// timeout 10 ms
	Wire.setWireTimeout(10000, true);
	Wire.beginTransmission(mAddress);
}

void DS2482::end()
{
	uint8_t ret;

	ret = Wire.endTransmission();
	if (ret) {
		status = stTimeout;
		/*  1 .. length to long for buffer
		 *  2 .. address send, NACK received
		 *  3 .. data send, NACK received
		 *  4 .. other twi error (lost bus arbitration, bus error, ..)
		 *  5 .. timeout
		 */
		last_err = ret;
	}
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
		last_err = ERR_READ;
		status = stTimeout;
		return 0xff;
	}

	return Wire.read();
}

/* initializes the error code and set it to:
 * 0 .. success
 * 12 .. address send, NACK received
 * 13 .. data send, NACK received
 * 14 .. other twi error (lost bus arbitration, bus error, ..)
 * 15 .. timeout
 * 17 .. read error / timeout
 * 18 .. timeout waiting for ready
  */
uint8_t DS2482::busyWait(bool setPtr)
{
	uint8_t res;
	int loopCount = 500;

	if (setPtr) {
		/* resets the last_err */
		setReadPtr(PTR_STATUS);

		if (last_err != ERR_NONE) {
			/* can only be 1 .. 5
			if (last_err != 0 && last_err <= ERR_WIRE_TO) */
			last_err += ERR_BUSYWAIT;

			return DS2482_STATUS_INVAL;
		}
	} else {
		/* in the other case this is done in the begin */
		status = stOk;
		last_err = ERR_NONE;
	}
	while((res = _read()) & DS2482_STATUS_BUSY) {
		if (last_err == ERR_READ) {
			last_err = ERR_BUSYWAIT_RD;
			return DS2482_STATUS_INVAL;
		}
		if (--loopCount == 0 || status == stTimeout) {
			status = stTimeout;
			last_err = ERR_BUSYWAIT_TO;
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

/* channel must be between 0 and 7
 * 32 .. address send, NACK received
 * 33 .. data send, NACK received
 * 34 .. other twi error (lost bus arbitration, bus error, ..)
 * 35 .. timeout
 * 37 .. read error / timeout
 * 38 .. timeout waiting for ready
 * 41..45 write error
 * 47 read error on cross check
 */
bool DS2482::selectChannel(uint8_t channel)
{
	static const byte chan_r[8] = { 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87 };
	static const byte chan_w[8] = { 0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87 };

	if (ch == channel)
		return true;
	if (busyWait(true) == DS2482_STATUS_INVAL) {
		/* err can be 11..18 */
		last_err += ERR_CHSEL1;
		return false;
	}
	begin();
	Wire.write(0xc3);
	Wire.write(chan_w[channel]);
	end();
	if (last_err != ERR_NONE) {
		/* err can be 1..5 : 41 ..45*/
		last_err += ERR_CHSEL2;
		return false;
	}
	/* after channel selection the read pointer points
	to the channel register */

	uint8_t check = _read();
	if (check != chan_r[channel]) {
		last_err = ERR_CHCHK;
		return false;
	}

	ch = channel;
	return true;
}

/* 52 .. address send, NACK received (on busy wait)
 * 53 .. data send, NACK received
 * 54 .. other twi error (lost bus arbitration, bus error, ..)
 * 55 .. timeout
 * 57 .. read error / timeout
 * 58 .. timeout waiting for ready

 * 62 .. address send, NACK received (after first write)
 * 63 .. data send, NACK received
 * 64 .. other twi error (lost bus arbitration, bus error, ..)
 * 65 .. timeout

 * 66 .. address send, NACK received (after first write)
 * 67 .. data send, NACK received
 * 68 .. other twi error (lost bus arbitration, bus error, ..)
 * 69 .. timeout
 * 71 .. read error / timeout
 * 72 .. timeout waiting for ready
 */
bool DS2482::reset()
{
	if (busyWait(true) == DS2482_STATUS_INVAL) {
		/* err can be 11..18 */
		last_err += ERR_RESET1;
		return false;
	}
	begin();
	Wire.write(0xb4);
	end();
	if (last_err != ERR_NONE) {
		/* err can be 1..5 */
		last_err += ERR_RESET2;
		return false;
	}
	uint8_t stat = busyWait();
	if (last_err != ERR_NONE) {
		/* err can be 11..18 */
		last_err += ERR_RESET3;
		return false;
	}
	return stat & DS2482_STATUS_PPD ? true : false;
}

/* 75 .. address send, NACK received (on busy wait)
 * 76 .. data send, NACK received
 * 77 .. other twi error (lost bus arbitration, bus error, ..)
 * 78 .. timeout
 * 80 .. read error / timeout
 * 81 .. timeout waiting for ready
 * 82 .. address send, NACK received (after write)
 * 83 .. data send, NACK received
 * 84 .. other twi error (lost bus arbitration, bus error, ..)
 * 85 .. timeout
 * */
uint8_t DS2482::write(uint8_t b, uint8_t power)
{
	(void)power;
	if (busyWait(true) == DS2482_STATUS_INVAL) {
		/* err can be 11..18 */
		last_err += ERR_WRITE1;
		return 0xff;
	}
	begin();
	Wire.write(0xa5);
	Wire.write(b);
	end();
	if (last_err != ERR_NONE) {
		/* err can be 1..5 */
		last_err += ERR_WRITE2;
		return 0xff;
	}

	return b;
}

/*
* 87 .. address send, NACK received
* 88 .. data send, NACK received
* 89 .. other twi error (lost bus arbitration, bus error, ..)
* 90 .. timeout
* 92 .. waiting error, address send, NACK received
* 93 .. data send, NACK received
* 94 .. other twi error (lost bus arbitration, bus error, ..)
* 95 .. timeout
* 97 .. read error / timeout
* 108 .. waiting2 error, timeout waiting for ready
* 102 .. address send, NACK received
* 103 .. data send, NACK received
* 104 .. other twi error (lost bus arbitration, bus error, ..)
* 105 .. timeout
* 107 .. read error / timeout in 2nd busywait
*/
uint8_t DS2482::read()
{
	if (busyWait(true) == DS2482_STATUS_INVAL) {
		/* err can be 11..18 */
		last_err += ERR_READ1;
		return 0xff;
	}
	begin();
	Wire.write(0x96);
	end();
	if (last_err != ERR_NONE) {
		last_err += ERR_READ2;
		return 0xff;
	}
	if (busyWait(true) == DS2482_STATUS_INVAL) {
		last_err += ERR_READ3;
		return 0xff;
	}
	setReadPtr(PTR_READ);

	return _read();
}

void DS2482::skip()
{
	write(OW_SKIP_ROM);
}

void DS2482::select(const uint8_t rom[8])
{
	write(OW_MATCH_ROM);
	if (last_err != ERR_NONE) {
		last_err += ERR_SELECT1;
		return;
	}
	for (int i = 0; i < 8; i++) {
		write(rom[i]);
		if (last_err != ERR_NONE) {
			last_err += ERR_SELECT2;
			return;
		}
	}
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

	if (!reset()) {
		last_err += ERR_SRCH1;
		//Serial.println(F("TO in search from reset"));
		return false;
	}

	if (search_mode == true)
		// NORMAL SEARCH
		write(OW_SEARCH_ROM);
	else
		// CONDITIONAL SEARCH
		write(OW_COND_SEARC_ROM);

	if (last_err != ERR_NONE) {
		last_err += ERR_SRCH2;
		return false;
	}
	for(i = 1; i < 65; i++)
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
