#ifndef OneWireBase_h
#define OneWireBase_h

#ifdef __cplusplus
#include <inttypes.h>

class OneWireBase
{
  public:
		// Perform a 1-Wire reset cycle. Returns 1 if a device responds
		// with a presence pulse.  Returns 0 if there is no device or the
		// bus is shorted or otherwise held low for more than 250uS
		virtual bool reset(void);
		// Write a byte. If 'power' is one then the wire is held high at
		// the end for parasitically powered devices. You are responsible
		// for eventually depowering it by calling depower() or doing
		// another read or write.
		virtual uint8_t write(uint8_t b, uint8_t power = 0 ) = 0;
		// Read a byte.
		virtual uint8_t read() = 0;
		// Issue a 1-Wire rom select command, you do the reset first.
		virtual void select(const uint8_t rom[8]) = 0;
		// Clear the search state so that if will start from the beginning again.
		virtual void reset_search();
		virtual bool search(uint8_t *newAddr, bool search_mode = true) = 0;
		// Issue a 1-Wire rom skip command, to address all on bus.
		virtual void skip(void);
		uint8_t crc8(const uint8_t *addr, uint8_t len);

};
#endif // __cplusplus
#endif // OneWire_h
