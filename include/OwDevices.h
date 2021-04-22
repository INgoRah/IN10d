#ifndef _OWDEVICES_H
#define _OWDEVICES_H

#include <OneWireBase.h>

#define MAX_CFG_SIZE 26
/** Retries for register read */
#define REG_RETRY 3;
/** Retries for activity latch reset */
#define ACTRES_RETRY 5;

union s_adr {
	uint16_t data;
	struct {
		unsigned int res : 1;
		// pio/latch number 0..7
		unsigned int bus : 3;  /* 1..7 */
		/** Press time
		 * 0: short press
		 * 2: button pushed (for long press detection)
		 * 1: long press released */
		unsigned int press : 2;
		/** Latch number (only one)
		 * 0: invalid
		 * 1..7: valid
		 * */
		unsigned int latch : 4; /* 1..15 */
		unsigned int adr : 6; /* 1..3F (65) */
	} sa;
};

// todo: add force on? or based on src like PIR?
union d_adr {
	uint8_t data;
	struct {
		/* PIO0 = 0, PIO1 = 1 */
		unsigned int pio : 1;
		unsigned int adr : 4;
		unsigned int type : 1;
		unsigned int bus : 2;
	} da;
};

class OwDevices
{
	private:
		OneWireBase *ow;
	public:
		OwDevices() {;}
		void begin(OneWireBase *ds);

		void adrGen(byte bus, byte adr[8], uint8_t id);
		uint8_t ds2408LatchReset(uint8_t* addr);
		uint8_t ds2408RegRead(byte bus, uint8_t* addr, uint8_t* data, bool latch_reset = true);
		uint8_t ds2408ChWrite(byte bus, uint8_t* addr, uint8_t* data, int cnt);
		void toggleDs2413(byte bus, uint8_t* addr);
		uint8_t ds2408PioGet(byte bus, uint8_t* addr);
		uint8_t ds2408PioSet(byte bus, uint8_t* addr, uint8_t pio);
		uint8_t ds2408TogglePio(byte bus, uint8_t* addr, uint8_t pio, uint8_t* data = NULL);
		void ds2408CfgWrite(byte bus, byte adr[8], uint8_t* d, uint8_t len);
		int ds2408CfgRead(byte bus, byte adr[8], uint8_t* data);
		uint16_t tempRead(byte busNr, byte addr[8], byte mode = 2);
		int search(byte bus);
};

#endif