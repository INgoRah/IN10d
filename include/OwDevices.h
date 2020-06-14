#ifndef _OWDEVICES_H
#define _OWDEVICES_H

#include <OneWireBase.h>

class OwDevices
{
	private:

	public:
		OwDevices() {;}
		void begin() { ; };
		void end() { ; };
		void loop();

		void adrGen(OneWireBase *ds, byte adr[8], uint8_t id);		
		uint8_t ds2408RegRead(OneWireBase *ds, uint8_t* addr, uint8_t* data, bool latch_reset = true);
		void ds2408Status(OneWireBase *ds, byte adr[8], bool latch_reset = true);
		void toggleDs2413(OneWireBase *ds, uint8_t* addr);
		void ds2408Data(OneWireBase *ds, byte adr[8], uint8_t len);
		uint8_t ds2408TogglePio(OneWireBase *ds, uint8_t* addr, uint8_t pio, uint8_t* data = NULL);
		void ds2408Cfg(OneWireBase *ds, byte adr[8], uint8_t* d, uint8_t len);

		void statusPrint(OneWireBase *ds, byte adr[8]);
		void statusRead(OneWireBase *ds);
		void search(OneWireBase *ds);
};

#endif