#ifndef _OWDEVICES_H
#define _OWDEVICES_H

#include <OneWireBase.h>
#include "main.h"

#define MAX_CFG_SIZE 26
/** Retries for register read */
#define REG_RETRY 3;
/** Retries for activity latch reset */
#define ACTRES_RETRY 5;

class OwDevices
{
	private:
		OneWireBase *ow;
		uint8_t	pio_data[MAX_BUS][MAX_ADR];

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
		int16_t tempRead(byte busNr, byte addr[8], byte mode = 2);
};

#endif