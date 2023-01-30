/*
  DS2482 library for Arduino
  Copyright (C) 2009-2010 Paeae Technologies

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __DS2482_H__
#define __DS2482_H__

#include <inttypes.h>
#include <OneWireBase.h>

// you can exclude onewire_search by defining that to 0
#ifndef ONEWIRE_SEARCH
#define ONEWIRE_SEARCH 1
#endif

// You can exclude CRC checks altogether by defining this to 0
#ifndef ONEWIRE_CRC
#define ONEWIRE_CRC 1
#endif

#define DS2482_CONFIG_APU (1<<0)
#define DS2482_CONFIG_PPM (1<<1)
#define DS2482_CONFIG_SPU (1<<2)
#define DS2484_CONFIG_WS  (1<<3)

#define DS2482_STATUS_BUSY 	(1<<0)
/* presence bit */
#define DS2482_STATUS_PPD 	(1<<1)
#define DS2482_STATUS_SD	(1<<2)
#define DS2482_STATUS_LL	(1<<3)
#define DS2482_STATUS_RST	(1<<4)
#define DS2482_STATUS_SBR	(1<<5)
#define DS2482_STATUS_TSB	(1<<6)
#define DS2482_STATUS_DIR	(1<<7)
#define DS2482_STATUS_INVAL	0xfd

enum DS2482_ERR {
  ERR_NONE = 0,
  /* length to long for buffer, should never occur */
  ERR_WIRE_LEN = 1,
  /* address send, NACK received */
  ERR_WIRE_ADRNACK = 2,
  /* data send, NACK received */
  ERR_WIRE_DATANACK = 3,
  /* other twi error (lost bus arbitration, bus error, ..) */
  ERR_WIRE_GEN = 4,
  /* Timeout */
  ERR_WIRE_TO = 5,
  /* requestFrom return 0: no data cause of timeout */
  ERR_READ = 6,

  ERR_BUSYWAIT = 10,
  /* 12, 13, 14, 15 */
  ERR_BUSYWAIT_RD = 17,
  ERR_BUSYWAIT_TO = 18,

  ERR_CHSEL1 = 20,
  /* ..1, ..2, ..3, ..4, ..5 */
  ERR_CHSEL2 = 40,
  /* ..1, ..2, ..3, ..4, ..5 */
  ERR_CHCHK = 47,

  ERR_RESET1 = 40,
  ERR_RESET2 = 60,
  ERR_RESET3 = 55,

  ERR_WRITE1 = 64,
  ERR_WRITE2 = 82,
  ERR_READ1 = 80,
  ERR_READ2 = 85,
  ERR_READ3 = 90,
  ERR_SELECT1 = 30, /* = 100 - 110 (30 + write 1,2) */
  ERR_SELECT2 = 40, /* = 110 - 120 (40 + write 1,2) */
  ERR_SRCH1 = 100, /* (reset + 100) */
  ERR_SRCH2 = 140,  /* write + 150 */
  ERR_NEXT
};

class DS2482 : public OneWireBase
{
public:
	//Address is 0-3
	DS2482() {}
	DS2482(uint8_t address);
	~DS2482() {}

  bool configureDev(uint8_t config);
	void resetDev();

	bool reset(); // return true if presence pulse is detected
	bool selectChannel(uint8_t channel);

	uint8_t write(uint8_t b, uint8_t power = 0 );
	uint8_t read();

    // Issue a 1-Wire rom select command, you do the reset first.
    void select(const  uint8_t rom[8]);
	// Issue skip rom
	void skip();

	uint8_t hasTimeout() { return status == stTimeout; }
#if ONEWIRE_SEARCH
    // Clear the search state so that if will start from the beginning again.
    void reset_search();

    // Look for the next device. Returns 1 if a new address has been
    // returned. A zero might mean that the bus is shorted, there are
    // no devices, or you have already retrieved all of them.  It
    // might be a good idea to check the CRC to make sure you didn't
    // get garbage.  The order is deterministic. You will always get
    // the same devices in the same order.
    bool search(uint8_t *newAddr, bool search_mode = true);
#endif

private:
  uint8_t ch;
	uint8_t _read();
	void setReadPtr(uint8_t readPtr);

	uint8_t busyWait(bool setPtr=false); //blocks until
	void begin();
	void end();

};

#endif