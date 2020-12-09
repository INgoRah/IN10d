#include "Arduino.h"			 // for delayMicroseconds, digitalPinToBitMask, etc
#include "SwitchHandler.h"  
#include "OwDevices.h"
#include <TwiHost.h>

#define MAX_TIMER 10

extern TwiHost host;
extern byte debug;

struct _timer_list {
	/* timestamp of start */
	unsigned long ms;
	/* seconds to switch off */
	uint16_t secs;
	/* switch off: */
	union d_adr dst;
}tmr_list[MAX_TIMER];

#if 0
/* Experimental: for PWM controlled lights, we need to have a level */
struct _dim_tbl {
	/* timestamp of start */
	unsigned long ms;
	union d_adr dst;
	byte level;
}dim_tbl[MAX_DIMMER];
#endif

struct _dim_tbl dim_tbl[MAX_DIMMER];

struct _sw_tbl sw_tbl[MAX_SWITCHES];

/* need switch off time! */
struct _sw_tbl timed_tbl[MAX_TIMED_SWITCH];

static uint8_t bitnumber(uint8_t bitmap)
{
	int i, res = 1;

	for (i = 0x1; i <= 0x80; i = i << 1) {
		if (bitmap & i)
			return res;
		res++;
	}
	/* invalid */
	return 0xff;
}

void SwitchHandler::initSwTable()
{
	uint16_t len, pos;
	uint8_t vers;

	vers = eeprom_read_byte((const uint8_t*)0);
	len = eeprom_read_word((const uint16_t*)2);
	pos = 4;
	if (len != 0xFFFF && vers != 0xff) {
		Serial.print(F("vers="));
		Serial.print(vers);
		Serial.print(F(" len="));
		Serial.print(len);
		switch (vers) {
			case 1:
				Serial.println(F("invalid sw tbl"));
				break;
			case 2:
				eeprom_read_block((void*)sw_tbl, (const void*)pos, len);
				break;
		}
	}
	pos += len;
	vers = eeprom_read_byte((const uint8_t*)pos);
	len = eeprom_read_word((const uint16_t*)pos + 2);
	pos += 4;
	if (len != 0xFFFF && vers != 0xff) {
		Serial.print(F("timed vers="));
		Serial.print(vers);
		Serial.print(F(" len="));
		Serial.print(len);
		if (len > sizeof(timed_tbl))
			len = sizeof(timed_tbl);
		eeprom_read_block((void*)timed_tbl, (const void*)pos, len);
	}
}

void SwitchHandler::begin(OneWireBase *ds)
{
    this->ds = ds;
	initSwTable();
	memset (dim_tbl, 0, sizeof(dim_tbl));
	dim_tbl[0].dst.da.bus = 2;
	dim_tbl[0].dst.da.adr = 7;
	dim_tbl[0].lvl.level = 0;
	for (int i = 0; i < MAX_TIMER; i++) {
		struct _timer_list* tmr = &tmr_list[i];
		tmr->secs = 0;
	}
}

bool SwitchHandler::timerUpdate(union d_adr dst, uint16_t secs)
{
	int i;

	for (i = 0; i < MAX_TIMER; i++) {
		struct _timer_list* tmr = &tmr_list[i];

		if (tmr->secs == 0 || tmr->dst.data == dst.data) {
			tmr->secs = secs;
			tmr->dst.data = dst.data;
			tmr->ms = millis();
#ifdef DEBUG
			if (debug) {
				Serial.print(F("start/update timer "));
				Serial.print(dst.da.bus);
				Serial.print(F("."));
				Serial.print(dst.da.adr);
				Serial.print(F("."));
				Serial.println(dst.da.pio);
			}
#endif

			return true;
		}
	}

	return false;
}

void SwitchHandler::loop()
{
	int i;

	for (i = 0; i < MAX_TIMER; i++) {
		struct _timer_list* tmr = &tmr_list[i];
		if (tmr->secs != 0 && (millis() > tmr->ms + (tmr->secs * 1000))) {
			/* stop timer */
			tmr->secs  = 0;
#ifdef DEBUG
			if (debug) {
				Serial.print(F("expired "));
				Serial.print(tmr->dst.da.bus);
				Serial.print(".");
				Serial.print(tmr->dst.da.adr);
				Serial.print(".");
				Serial.println(tmr->dst.da.pio);
			}
#endif
			/* action with dst */
			switchPio(tmr->dst, OFF);
		}
	}
#if 0
	// go over all timer in list...
	if (tmr[0] && millis() > tmr[0] + 300) {
		int level;
		byte adr[8];
		union d_adr dst = dim_tbl[0].dst;

		dim_tbl[0].level = dim_tbl[0].level + 1;
		if (dim_tbl[0].level == 0xf)
			tmr[0] = 0;
		else
			tmr[0] = millis();
		if (dim_tbl->level == 0xf) {
			return;
		}
		level = (dim_tbl[0].level << 4) | 1;
		ow->adrGen(ds, dst.da.bus, adr, dst.da.adr);
		ow->ds2408PioSet(ds, dst.da.bus, adr, level);
	}
#endif
}

static byte dimLevel(union d_adr dst)
{
	for (uint8_t i = 0; i < MAX_DIMMER; i++) {
		/*if (dst.da.bus == 2 && dst.da.adr == 7)
			return 5;*/
		if (dst.data == dim_tbl[i].dst.data) {
			return dim_tbl[i].lvl.level;
		}
	}

	return 0xFF;
}

static byte getVersion(byte bus, byte adr)
{
	switch (bus) {
		case 0:
			switch(adr) {
				case 1:
				case 3:
				case 8:
					return 2;
			}
			break;
		case 1:
			switch(adr) {
				case 1:
				case 11:
					return 1;
				case 7:
					return 2;
			}
			break;
		case 2:
			switch(adr) {
				case 1:
				case 2:
				case 3:
					return 2;
			}
			break;
	}
	return 32;
}

/* Convert from alarm location to a lookup table format (16 bit)
 * latch - bit mask to be converted to bit number
 * press - 0: pressing, > 0: press time
 */
static uint16_t srcData(uint8_t busNr, uint8_t adr1, uint8_t latch, uint8_t press)
{
	union s_adr src;
	byte v;

	src.data = 0;
	src.sa.bus = busNr;
	src.sa.adr =  adr1 & 0x3f;
	src.sa.latch = bitnumber(latch);
	if (debug > 0) {
		Serial.print(src.sa.bus, HEX);
		Serial.print(F("."));
		Serial.print(src.sa.adr, HEX);
		Serial.print(F("."));
		Serial.print(src.sa.latch, HEX);
		Serial.print(F(" "));
	}
	v = getVersion(src.sa.bus, src.sa.adr);

	if (v < 3 || press == 0xff)
		src.sa.press = 0;
	else {
		/* first two latches are usually output
		* signaled from auto switch. The corresponding latch
		* is not signaled!
		* */
		if (press == 0) {
			src.sa.press = 2;
			if (debug > 0)
				Serial.println(F(" pressing"));
			// find free timer and start it
			// tmr.src = src...
			/*
			tmr[0] = millis();
			dim_tbl[0].level = 0;
			dim_tbl[0].dst.da.bus = 2;
			dim_tbl[0].dst.da.adr = 3;
			*/
		} else if (press > (400 / 32)) {
			src.sa.press = 1;
			if (debug > 0) {
				Serial.print(F(" time="));
				Serial.println(press * 32);
			}
		} else {
			src.sa.press = 0;
			if (debug > 0)
				Serial.println();
		}
	}
	if (debug > 0) {
		Serial.println(src.data, HEX);
	}

	return src.data;
}

byte dimStage(byte dim)
{
	switch (dim) {
		case 0:
			return 5;
		case 5:
			return 10;
		case 10:
			return 15;
		case 15:
			return 0;
	}
	return 0;
}

bool SwitchHandler::switchPio(union d_adr dst, enum _pio_mode mode = TOGGLE)
{
	byte pio, retry, d;
	byte adr[8], bus;

	bus = dst.da.bus;
#ifdef DEBUG
	if (debug) {
		Serial.print(bus);
		Serial.print(".");
		Serial.print(dst.da.adr);
		Serial.print(".");
		Serial.println(dst.da.pio);
	}
#endif
	 pio = 1 << dst.da.pio;
	// type 0:
	ow->adrGen(ds, bus, adr, dst.da.adr);
#if 0
	// type 1:
	static uint8_t target[8] = { 0x3A, 0x01, 0xDA, 0x84, 0x00, 0x00, 0x05, 0xA3 };
	ow->toggleDs2413 (bus[0], target);
#endif
	retry = 5;
	do {
		byte dim;

		d = ow->ds2408PioRead(ds, bus, adr);
		dim = dimLevel(dst);
		if (dim == 0xFF && (d & pio) && mode == OFF) {
			return false;
		}
		if (dim == 0xFF && (d & pio) == 0 && mode == ON) {
			return false;
		}
		switch (mode) {
			case ON:
				if (dim != 0xFF) {
					dim = dimStage(dim);
					dim_tbl[0].lvl.level = dim;
					d &= 0x0F;
					if (dim == 0)
						d &= ~(pio);
					else
						d |= (dim << 4) | pio;
				}
				else
					d &= ~(pio);
				break;
			case OFF:
				if (dim != 0xFF) {
					d &= ~(pio | 0xF0);
					dim_tbl[0].lvl.level = 0;
				}
				else 
					d |= pio;
				break;
			case TOGGLE:
				if (dim != 0xFF) {
					dim = dimStage(dim);
					dim_tbl[0].lvl.level = dim;
					d &= 0x0F;
					if (dim == 0)
						d &= ~(pio);
					else
						d |= (dim << 4) | pio;
				}
				else {
					if (d & pio)
						d &= ~(pio);
					else
						d |= pio;
				}
				break;
		}		
		if (debug) {
			Serial.print(" -> ");
			Serial.println(d, HEX);
		}
		d = ow->ds2408PioSet(ds, bus, adr, d);
		if (d == 0xAA) {
			break;
		}
	} while (retry-- > 0);
	if (d != 0xAA)
		return false;

	return true;
}

#define DEF_SECS 30

extern uint8_t sec;
extern uint8_t min;
extern uint8_t hour;

/* latch - bit mask to be converted to bit number */
bool SwitchHandler::switchHandle(uint8_t busNr, uint8_t adr1, uint8_t latch, uint8_t press, byte mode)
{
	union s_adr src;
	struct logData d;

	// busNr should be == addr[2]
	src.data = srcData(busNr, adr1, latch, press);
	// put into fifo
	d.data = src.data;
	d.h = hour;
	d.min = min;
	d.sec = sec;
	host.events.push(d);

	// todo: signal alarm only here once the data is available
	if ((mode & MODE_AUTO_SWITCH) == 0)
		return false;
	/* Timed handler: reset or start timer 
	 * based on PIR or standard light (bath)
	 * PIR needs 1 min of initialization, ignore after start
	 * Single trigger mode for 3 secs
	bool handled;?? */
	for (uint8_t i = 0; i < MAX_TIMED_SWITCH; i++) {
		if (src.data == timed_tbl[i].src.data) {
#ifdef DEBUG
			if (debug) {
				Serial.print(F("switch #"));
				Serial.print(i);
				Serial.print(" ");
				Serial.print(src.data, HEX);
				Serial.print(" -> ");
			}
#endif
			if (timed_tbl[i].dst.data == 0 || timed_tbl[i].dst.data == 0xFF)
				return false;
			/* switch press:
			 *   - if off: switch on and start timer
			 *   - if on with timer running: reset timer
			 *   - if on without timer: switch off
			 * High when motion is detected. Check before switching off
			 */
			/* default action: switch on and start timer for off */
			switchPio(timed_tbl[i].dst, ON);
			/*  if on/timer running: reset timer or start timer */
			timerUpdate(timed_tbl[i].dst, DEF_SECS);
		}
	}
	/*
	 * Standard handler: from 5 bit adr (1..1f) and 3 bit latch (0..7)
	 * Get target switch
	 */
	for (uint8_t i = 0; i < MAX_SWITCHES; i++) {
		if (src.data == sw_tbl[i].src.data) {
#ifdef DEBUG
			Serial.print(F("switch #"));
			Serial.print(i);
			Serial.print(" ");
			Serial.print(src.data, HEX);
			Serial.print(" -> ");
#endif
			/* toggle pio */
			switchPio(sw_tbl[i].dst);
		}
	}

    return false;
}

bool SwitchHandler::alarmHandler(byte busNr, byte mode)
{
	byte addr[8];
	byte j = 0;
	uint8_t i, latch, data[10], retry = 5;
	uint8_t cnt = 10;

	//ds = bus[busNr];
	ds->selectChannel(busNr);
	ds->reset();
	ds->reset_search();

	while (ds->search(addr, false)) {
		j++;
		if (debug > 0) {
			Serial.print(busNr);
			Serial.print(F("@"));
			for (i = 0; i < 8; i++) {
				if (addr[i] < 0x10)
					Serial.print(F("0"));
				Serial.print(addr[i], HEX);
			}
			Serial.print(F(" "));
		}
		if ((mode & MODE_ALRAM_HANDLING) == 0) {
			// interrupt to host
			digitalWrite (HOST_ALRM_PIN, LOW);
			return true;
		}
		latch = 0;
		switch (addr[0]) {
			case 0x29:
				retry = 5;
				do {
					latch = ow->ds2408RegRead(ds, busNr, addr, data);
					if (latch == 0xAA) {
						latch = data[2];
						break;
					}
				} while (retry-- > 0);
#ifdef DEBUG				
				Serial.print(" (");
				Serial.print(data[1], HEX);
				Serial.print(" ");
				Serial.print(data[2], HEX);
				Serial.print(") | ");
#endif				
				break;
			case 0x28:
				ow->tempRead(ds, busNr, addr);
				return true;
			case 0x35:
				break;
		}
		if (latch != 0 && latch != 0xff)
			switchHandle(busNr, addr[1], latch, data[6], mode);
		else
			return false;

		if (cnt-- == 0) {
			Serial.println(F("Error searching"));
			break;
		}
	}

	return j > 0 ? true : false;
}
