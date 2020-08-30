#ifndef OneWireBase_h
#define OneWireBase_h

#ifdef __cplusplus
#include <inttypes.h>

#define stOk 0
#define stTimeout 1
#define stNoDevs 2

#define OW_MATCH_ROM    0x55
#define OW_SKIP_ROM     0xCC
#define OW_SEARCH_ROM   0xF0
#define OW_COND_SEARC_ROM  0xEC
/* start new search */
#define OW_SEARCH_FIRST 0xFF

class OneWireBase
{
	protected:
		uint8_t searchAddress[8];
		uint8_t searchExhausted;
	
	public:
		uint8_t status;
		uint8_t mAddress;
		uint8_t searchLastDisrepancy;

		// Perform a 1-Wire reset cycle. Returns 1 if a device responds
		// with a presence pulse.  Returns 0 if there is no device or the
		// bus is shorted or otherwise held low for more than 250uS
		virtual bool reset(void);
		bool configureDev(uint8_t config) { return true; };
		void resetDev() {};
		// DS2482-800 only
		bool selectChannel(uint8_t channel) { return false; };

		// Write a byte. If 'power' is one then the wire is held high at
		// the end for parasitically powered devices. You are responsible
		// for eventually depowering it by calling depower() or doing
		// another read or write.
		virtual uint8_t write(uint8_t b, uint8_t power = 0 ) = 0;
		void write(const uint8_t *buf, uint16_t count, uint8_t power = 0);
		// Read a byte.
		virtual uint8_t read() = 0;
	    void read(uint8_t *buf, uint16_t count);
		// Issue a 1-Wire rom select command, you do the reset first.
		virtual void select(const uint8_t rom[8]) = 0;
		// Clear the search state so that if will start from the beginning again.
		virtual void reset_search();
		virtual bool search(uint8_t *newAddr, bool search_mode = true) = 0;
		// Issue a 1-Wire rom skip command, to address all on bus.
		virtual void skip(void);
		uint8_t crc8(const uint8_t *addr, uint8_t len);
		uint16_t crc16(const uint8_t* input, uint16_t len, uint16_t crc);
		bool check_crc16(const uint8_t* input, uint16_t len, const uint8_t* inverted_crc, uint16_t crc);
};
#endif // __cplusplus
#endif // OneWire_h
