#ifndef _OWDEVICES_H
#define _OWDEVICES_H

#include <OneWireBase.h>

#define MAX_BUS 4
#define MAX_SWITCHES 2 * 8

#define MAX_CFG_SIZE 26

union s_adr {
	uint8_t data;
	struct {
		// pio/latch number 0..7 
		unsigned int latch : 3;
		unsigned int adr : 5;
	} sa;
};

union d_adr {
	uint8_t data;
	struct {
		unsigned int pio : 1;
		unsigned int adr : 4;
		unsigned int type : 1;
		unsigned int bus : 2;
	} da;
};


class OwDevices
{
	private:

	public:
		OwDevices() {;}
		void begin(OneWireBase *ds);
		void end() { ; };
		void loop();

		void adrGen(OneWireBase *ds, byte bus, byte adr[8], uint8_t id);		
		uint8_t ds2408LatchReset(OneWireBase *ds, uint8_t* addr);
		uint8_t ds2408RegRead(OneWireBase *ds, byte bus, uint8_t* addr, uint8_t* data, bool latch_reset = true);
		void ds2408Status(OneWireBase *ds, byte bus, byte adr[8], bool latch_reset = true);
		void toggleDs2413(OneWireBase *ds, byte bus, uint8_t* addr);
		void ds2408Data(OneWireBase *ds, byte bus, byte adr[8], uint8_t len);
		uint8_t ds2408PioSet(OneWireBase *ds, byte bus, uint8_t* addr, uint8_t pio);
		uint8_t ds2408TogglePio(OneWireBase *ds, byte bus, uint8_t* addr, uint8_t pio, uint8_t* data = NULL);
		void ds2408CfgWrite(OneWireBase *ds, byte bus, byte adr[8], uint8_t* d, uint8_t len);
		int ds2408CfgRead(OneWireBase *ds, byte bus, byte adr[8], uint8_t* data);
		float tempRead(OneWireBase *ds, byte busNr, byte addr[8]);
		void statusPrint(OneWireBase *ds, byte adr[8]);
		void statusRead(OneWireBase *ds);
		void search(OneWireBase *ds, byte bus);
};

#endif