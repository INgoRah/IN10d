#include "Arduino.h"
#include "OneWireBase.h"
#if defined(__AVR__)
#include <util/crc16.h>
#endif

#define ONEWIRE_CRC8_TABLE
#define ONEWIRE_CRC16

#if 0
//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OneWire::reset_search() to
// start over.
//
// searchJunction is the branch on the current address at which the
//    current search is to continue -
//    prior to this point the path follows the previous address
//    after this point, at each junction, 0 is selected
// lastJunction is the last unexplored junction on the present path.
//    the next search will continue from here
//
bool  OneWire::search(uint8_t *newAddr)
{
 uint8_t i = 0;
 char lastJunction = -1;

 if ( searchExhausted) return 0;
 searchExhausted = 1; // the search is exhausted unless we find another junction

 if ( !reset()) return 0;
 write( 0xf0, 0);    // send search ROM

 for( i = 0; i < 64; i++) {
   uint8_t a = read_bit( );
   uint8_t nota = read_bit( );
   uint8_t ibyte = i/8;
   uint8_t ibit = 1<<(i&7);

   if ( a && nota) return 0;  // I don't think this should happen, this means nothing responded, but maybe if
   // something vanishes during the search it will come up.
   if ( !a && !nota) {                    // if at a junction, then
     if ( i < searchJunction) {           // if before search junction
       if ( address[ ibyte]&ibit)  a = 1; // follow previous address
       else a = 0;
     }
     else {
       if ( i == searchJunction) a = 1;// at the search junction take the new branch.
       else a = 0;                     // past the search junction, on a new branch so select 0
     }
     if (!a) {             // if 0 is chosen
       lastJunction = i;   // set last junction
       searchExhausted = 0; // continue the search
     }
   }

   if (a) address[ibyte] |= ibit; // set address bit
   else address [ibyte] &= ~ibit;
   write_bit( a);
 } // end of address bit loop
 searchJunction = lastJunction;                    // set searchJunction for the next call to Search

 for ( i = 0; i < 8; i++) newAddr[i] = address[i]; // copy found address into output address
 return 1;
}
#endif

void OneWireBase::write(const uint8_t *buf, uint16_t count, uint8_t power/* = 0 */)
{
	for (uint16_t i = 0 ; i < count ; i++)
		write(buf[i]);

}

void OneWireBase::read(uint8_t *buf, uint16_t count) {
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = read();
}

// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

#ifdef ONEWIRE_CRC8_TABLE
// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t PROGMEM dscrc2x16_table[] = {
	0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
	0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
	0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
	0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.  (Use tiny 2x16 entry CRC table)
uint8_t OneWireBase::crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
		pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
	}

	return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but a little smaller, than the lookup table.
//
static uint8_t OneWireBase::crc8(const uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

	while (len--) {
#if defined(__AVR__)
		crc = _crc_ibutton_update(crc, *addr++);
#else
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
#endif
	}
	return crc;
}
#endif

// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//
bool OneWireBase::check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc)
{
#ifdef ONEWIRE_CRC16
    crc = ~crc16(input, len, crc);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
#endif
}

uint16_t OneWireBase::crc16(const uint8_t* input, uint16_t len, uint16_t crc)
{
#ifdef ONEWIRE_CRC16
#if defined(__AVR__)
    for (uint16_t i = 0 ; i < len ; i++) {
        crc = _crc16_update(crc, input[i]);
    }
#else
    static const uint8_t oddparity[16] =
        { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    for (uint16_t i = 0 ; i < len ; i++) {
      // Even though we're just copying a byte from the input,
      // we'll be doing 16-bit computation with it.
      uint16_t cdata = input[i];
      cdata = (cdata ^ crc) & 0xff;
      crc >>= 8;

      if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
          crc ^= 0xC001;

      cdata <<= 6;
      crc ^= cdata;
      cdata <<= 1;
      crc ^= cdata;
    }
#endif
    return crc;
#else
	return 0;
#endif
}
