#include <main.h>
#include "SwitchHandler.h"
#include "OwDevices.h"
#include <TwiHost.h>

extern TwiHost host;

struct _timer_item {
	/* timestamp of start */
	unsigned long ms;
	/* seconds to switch off */
	uint16_t secs;
	/* switch off: */
	union d_adr_8 dst;
}tmr_list[MAX_TIMER];

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
#ifdef EXT_DEBUG
		Serial.print(F(" vers="));
		Serial.print(vers);
		Serial.print(F(" len="));
		Serial.print(len);
#endif
		if (vers == 2)
			eeprom_read_block((void*)sw_tbl, (const void*)pos, len);
	}
	pos += len;
	vers = eeprom_read_byte((const uint8_t*)pos);
	len = eeprom_read_word((const uint16_t*)pos + 2);
	pos += 4;
	if (len != 0xFFFF && vers != 0xff) {
#ifdef EXT_DEBUG
		Serial.print(F(" timed vers="));
		Serial.print(vers);
		Serial.print(F(" len="));
		Serial.print(len);
#endif
		if (len > sizeof(timed_tbl))
			len = sizeof(timed_tbl);
		eeprom_read_block((void*)timed_tbl, (const void*)pos, len);
	}
}

void SwitchHandler::begin(OneWireBase *ow)
{
    this->ds = ow;
	initSwTable();
	memset (dim_tbl, 0, sizeof(dim_tbl));
	/* Specific devices and our own pin having a dimmer feature */

	/* dimmer 1 / stairs OG */
	dim_tbl[0].dst.da.bus = 2;
	dim_tbl[0].dst.da.adr = 7;
	/* dimmer 2 / stairs UG, this is our own pin */
	dim_tbl[1].dst.da.bus = 0;
	dim_tbl[1].dst.da.adr = 9;
	dim_tbl[1].dst.da.pio = 0;
	/* dimmer 3 */
	dim_tbl[2].dst.da.bus = 3;
	dim_tbl[2].dst.da.adr = 3;
	for (int i = 0; i < MAX_TIMER; i++) {
		struct _timer_item* tmr = &tmr_list[i];
		tmr->secs = 0;
	}
}

bool SwitchHandler::timerUpdate(union d_adr_8 dst, uint16_t secs)
{
	int i;

	for (i = 0; i < MAX_TIMER; i++) {
		struct _timer_item* tmr = &tmr_list[i];

		if (tmr->secs == 0 || tmr->dst.data == dst.data) {
#ifdef EXT_DEBUG
			if (debug > 1) {
				if (tmr->secs)
					Serial.print(F("update timer "));
				else
					Serial.print(F("start timer "));
			}
#endif
			tmr->secs = secs;
			tmr->dst.data = dst.data;
			tmr->ms = millis();
#ifdef EXT_DEBUG
			if (debug > 1) {
				Serial.print(dst.da.bus);
				Serial.write('.');
				Serial.print(dst.da.adr);
				Serial.write('.');
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
		struct _timer_item* tmr = &tmr_list[i];
		if (tmr->secs != 0 && (millis() > tmr->ms + (tmr->secs * 1000))) {
			/* stop timer */
			tmr->secs  = 0;
			/* action with dst */
			actorHandle(tmr->dst, OFF);
		}
	}
}

static uint8_t dimLevel(union pio dst, uint8_t* id)
{
	for (uint8_t i = 0; i < MAX_DIMMER; i++) {
		if (dst.da.bus == dim_tbl[i].dst.da.bus &&
			dst.da.adr == dim_tbl[i].dst.da.adr &&
			dst.da.pio == dim_tbl[i].dst.da.pio) {
			*id = i;
			return dim_tbl[i].lvl.level;
		}
	}

	*id = 0xff;
	return 0xFF;
}

static uint8_t getVersion(uint8_t bus, uint8_t adr)
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
		default:
			switch(adr) {
				case 1:
				case 11:
					return 1;
			}
			break;
		case 2:
			switch(adr) {
				case 2:
				case 3:
					return 2;
			}
			break;
	}
	return 32;
}

static struct _timer_item* timerItem(uint8_t data)
{
	for (int i = 0; i < MAX_TIMER; i++) {
		struct _timer_item* tmr = &tmr_list[i];

		if (data == tmr->dst.data && tmr->secs) {
			return tmr;
		}
	}

	return NULL;
}

/* Convert from alarm location to a lookup table format (16 bit)
 * latch - bit mask to be converted to bit number
 * press - 0: pressing, > 0: press time
 *
 * latch = data[2]
 * press = data[6]
 */
uint16_t SwitchHandler::srcData(uint8_t busNr, uint8_t adr1)
{
	union s_adr src;
	uint8_t v;

	src.data = 0;
	src.sa.bus = busNr;
	src.sa.adr =  adr1 & 0x3f;
	src.sa.latch = bitnumber(data[2]);
	v = getVersion(src.sa.bus, src.sa.adr);

	if (v < 3 || data[6] == 0xff)
		src.sa.press = 0;
	else {
		/* first two latches are usually output
		* signaled from auto switch. The corresponding latch
		* is not signaled!
		* */
		if (data[6] == 0)
			src.sa.press = 2;
		else if (data[6] > (350 / 32))
			src.sa.press = 1;
		else
			src.sa.press = 0;
	}
#ifdef DEBUG
	if (debug > 0) {
		Serial.print(src.sa.bus, HEX);
		Serial.print(F("."));
		Serial.print(src.sa.adr, HEX);
		Serial.print(F("."));
		if (src.sa.press)
			Serial.print(10 * src.sa.press + src.sa.latch);
		else
			Serial.print(src.sa.latch);
		Serial.print(F(" "));
		if (debug > 1) {
			Serial.print(F(" time="));
			Serial.println(data[6] * 32);
		} else
			Serial.println();
	}
#endif

	return src.data;
}

uint8_t SwitchHandler::dimStage(uint8_t dim)
{
	switch (dim) {
		case 0:
			return 7;
		case 7:
			return 15;
		default:
			return 0;
	}
	return 0;
}

uint8_t SwitchHandler::getType(union pio dst)
{
	if (dst.da.bus == 0 && dst.da.adr == 9) {
		return 2;
	}
	return 0;
}

/* read data and generate address */
uint8_t SwitchHandler::dataRead(union pio dst, uint8_t adr[8])
{
	uint8_t d;

	_devs->adrGen(dst.da.bus, adr, dst.da.adr);
	d = _devs->ds2408PioGet(dst.da.bus, adr);
#ifdef EXT_DEBUG
	if (debug > 2) {
		Serial.print(F(" ("));
		Serial.print(d, HEX);
		Serial.print(F(") "));
	}
#endif

	return d;
}

/**
 * low level set the level for supporting PIOs
 *
 * @param id id in dimLevel cache array
 * @param d data read from PIO before
 * @param level level [0..15] converted from percent to 4 bit (level = level * 16 / 100)
 * @return true if successful otherwise false on error
 */
bool SwitchHandler::setLevel(union pio dst, uint8_t adr[8], uint8_t d, uint8_t id, uint8_t level)
{
	switch (dst.da.type) {
	case 0:
	default:
#ifdef EXT_DEBUG
		if (debug > 1) {
			Serial.print(dst.da.bus);
			Serial.print(F("."));
			Serial.print(adr[1]);
			Serial.print(F(" = "));
			Serial.print(d, HEX);
		}
#endif
		// clear level in data
		d &= 0xF;
		if (level == 0)
			d &= ~(1 << dst.da.pio);
		else
			// set level and switch PIO on
			d |= (level << 4) | (1 << dst.da.pio);
		if (_devs->ds2408PioSet(dst.da.bus, adr, d) != 0xAA)
			return false;
#ifdef EXT_DEBUG
		if (debug > 1) {
			Serial.print(" -> ");
			Serial.println(d, HEX);
		}
#endif
		break;
	case 2:
		if (dst.da.bus == 0 && dst.da.adr == 9) {
			if (dst.da.pio == 0) {
				level = level * 255 / 100;
				analogWrite(5, level);
			}
		}
		break;
	}
	dim_tbl[id].lvl.level = level;
	host.addEvent (dst, level);

	return true;
}

/**
 * Low level set the PIO on or off and checking the target state before
 *
 * @param id id in dimLevel cache array
 * @param d data read from PIO before
 * @return true if switched (was not in the target state), otherwise false
 */
/* TODO check for light at the light sensor ...
	bus == 0, adr == 1, pio = 1
	light_sensor = 1;
*/
bool SwitchHandler::setPio(union pio dst, uint8_t adr[8], uint8_t d, enum _pio_mode mode)
{
	uint8_t pio, r;

#ifdef EXT_DEBUG
	if (debug > 2) {
		Serial.print(dst.da.bus);
		Serial.print(F("."));
		Serial.print(dst.da.adr);
		Serial.print(F("."));
		Serial.print(dst.da.pio);
	}
#endif
	if (dst.da.type == 2) {
		// PIO0: PWM on pin 5
		// D3 - Relais ouput, negative polarity - active switching GND
		// D5 - PWM output (dimed LED) via open coollector transistor
		// D2 - PIR
		// D6 - used for alarm signal to host (class TwiHost)
		// 13 LED
		if (dst.da.pio == 0 || dst.da.pio == 2 || dst.da.pio == 5)
			return false;

		if (mode == OFF)
			digitalWrite(dst.da.pio, 1);
		else
			digitalWrite(dst.da.pio, 0);
#ifdef EXT_DEBUG
	if (debug > 2) {
		Serial.print(F(" pin "));
		Serial.print(dst.da.pio);
		if (mode == OFF)
			Serial.println(F(" OFF"));
		else
			Serial.println(F(" ON"));
	}
#endif
		return true;
	}
	/* turn to bitmask (da.pio = 0,1,2 >> pio = 1,2,4) */
	pio = 1 << dst.da.pio;

	switch (mode) {
		case ON:
			/* timer mode or force */
			if ((d & pio) == 0)
				return false;
			d &= ~(pio);
			break;
		case OFF:
			/* already off */
			if (d & pio)
				return false;
			d |= pio;
			break;
		case TOGGLE:
			/* button pressed: toggle or dim stages */
			if (d & pio)
				d &= ~(pio);
			else
				/* todo: check for timer, first press will
				   just stop the timer */
				d |= pio;
			break;
	}
#ifdef EXT_DEBUG
	if (debug > 2) {
		if (d & pio)
			Serial.print(F(" OFF"));
		else
			Serial.print(F(" ON"));
		Serial.print(F(" -> "));
		Serial.print(d, HEX);
	}
#endif

	r = _devs->ds2408PioSet(dst.da.bus, adr, d);
	if (r == 0xAA) {
		host.addEvent (dst, (uint16_t)d);
		return true;
	}

	return false;
}

/**
 * Called from top level control to switch to a dedicated level or
 * simply on or off.
 * If set already it will ignore and return false
 * level 0 .. 100 will be translated in 16 stages. If level < 16
 *       directly switch off
 * */
bool SwitchHandler::switchLevel(union pio dst, uint8_t level)
{
	uint8_t adr[8], d, id;

	dst.da.type = getType(dst);
	if (dst.da.type != 2)
		d = dataRead(dst, adr);
	dimLevel(dst, &id);
	if (id != 0xff) {
		Serial.println (F("Setting dimmer level"));
		return setLevel(dst, adr, d, id, level * 16 / 100);
	}
	else {
		Serial.println (F("Setting PIO"));
		if (level < 50)
			return setPio (dst, adr, d, OFF);
		else
			return setPio (dst, adr, d, ON);
	}
}

/* Reads out PIO data and checks for dimmer, calls toggle or set
   functions for PIO or dimmer.
 @return true if successfully switched, false in case of an error or
 no action was needed (if switch is already in that state)
*/
bool SwitchHandler::actorHandle(union d_adr_8 dst, enum _pio_mode mode)
{
	uint8_t adr[8], d, dim, id;
	union pio p;

	p.data = 0;
	p.da.bus = dst.da.bus;
	p.da.adr = dst.da.adr;
	p.da.pio = dst.da.pio;
	p.da.type = getType(p);

	if (p.da.type != 2)
		d = dataRead(p, adr);
	dim = dimLevel(p, &id);
	if (id == 0xff)
		// simply pass the mode
		return setPio (p, adr, d, mode);

	// this is a dimmer
	switch (mode) {
		case TOGGLE:
			// toggle level: off - 1 - 2 - off
			dim = dimStage(dim);
			return setLevel (p, adr, d, id, dim);
		case ON:
			if (dim != 0)
				/* was on before, don't start timer */
				return false;
			return setLevel(p, adr, d, id, 15);
		case OFF:
			return setLevel(p, adr, d, id, 0);
	}

	/* just for the compiler, never will get here */
	return false;
}

//bool SwitchHandler::actorHandle(union pio p, enum _pio_mode mode)

/* latch in data[2] - bit mask to be converted to bit number
   uses data[1] for actual IO status
   data[6] for press time */
bool SwitchHandler::switchHandle(uint8_t busNr, uint8_t adr1)
{
	union s_adr src;

	src.data = srcData(busNr, adr1);
	host.addEvent (0, src.data, data[1]);

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
#ifdef EXT_DEBUG
			if (debug > 1) {
				Serial.print(F("timer #"));
				Serial.print(i);
				Serial.print(" ");
				Serial.print(src.data, HEX);
				Serial.print(" -> ");
			}
#endif
			if (!(timed_tbl[i].dst.data == 0 ||
				  timed_tbl[i].dst.data == 0xFF) &&
				  (light > 210 /*|| sun == 0*/)) {
				/* switch press:
				*   - if off: switch on and start timer
				*   - if on with timer running: reset timer
				*   - if on without timer: switch off
				*/
				/* default action: switch on and start timer for off */
				if (actorHandle(timed_tbl[i].dst, ON)) {
					/*  on or timer running: set or reset timer */
					timerUpdate(timed_tbl[i].dst, DEF_SECS);
				} else {
					/* switched already on. Is timer running?
					   If yes, restart */
					struct _timer_item* tmr = timerItem(timed_tbl[i].dst.data);
					if (tmr != NULL) {
							tmr->secs = DEF_SECS;
							tmr->ms = millis();
					}
				}
			}
		}
	}
	wdt_reset();
	/*
	 * Standard handler: from 5 bit adr (1..1f) and 3 bit data[2] (0..7)
	 * Get target switch
	 */
	for (uint8_t i = 0; i < MAX_SWITCHES; i++) {
		if (src.data == sw_tbl[i].src.data) {
#ifdef EXT_DEBUG
			if (debug > 1) {
				Serial.print(F("switch #"));
				Serial.print(i);
				Serial.print(" ");
				Serial.print(src.data, HEX);
				Serial.print(" -> ");
			}
#endif
			struct _timer_item* tmr = timerItem(sw_tbl[i].dst.data);
			if (tmr != NULL && tmr->secs)
				/* timer running, stop it */
				tmr->secs = 0;
			else
				/* toggle io or select levels */
				actorHandle(sw_tbl[i].dst, TOGGLE);
		}
	}

    return false;
}

/* latch as bitmask */
bool SwitchHandler::switchHandle(uint8_t busNr, uint8_t adr1, uint8_t latch, uint8_t mode)
{
	this->mode = mode;
	/* fill data for use in switchHandle */
	this->data[2] = latch;
	// ignoring current state of PIOs
	this->data[1] = 0xff;
	// no press time info
	this->data[6] = 0xff;
	return switchHandle(busNr, adr1);
}

bool SwitchHandler::alarmHandler(uint8_t busNr, uint8_t mode)
{
	uint8_t adr[8];
	uint8_t j = 0;
	uint8_t i;
	uint8_t cnt = 10;

	//ds = bus[busNr];
	ds->selectChannel(busNr);
	ds->reset();
	ds->reset_search();
	this->mode = mode;

	while (ds->search(adr, false)) {
		j++;
#ifdef DEBUG
		if (debug > 0) {
			Serial.print(busNr);
			Serial.print(F("@"));
			for (i = 0; i < 8; i++) {
				if (adr[i] < 0x10)
					Serial.write('0');
				Serial.print(adr[i], HEX);
			}
			Serial.print(F(" "));
		}
#endif
		if ((mode & MODE_ALRAM_HANDLING) == 0) {
			// interrupt to host
			digitalWrite (HOST_ALRM_PIN, LOW);
			return true;
		}
		if (adr[0] == 0x29) {
			/* fill data for use in switchHandle */
			if (_devs->ds2408RegRead(busNr, adr, data) == 0xaa) {
				switchHandle(busNr, adr[1]);
			}
		}
		if (adr[0] == 0x28) {
				uint16_t c = _devs->tempRead(busNr, adr, 1);
				host.addEvent (2, busNr, adr[1], c);
#ifdef DEBUG
				if (debug) {
					Serial.print(busNr);
					Serial.print(F("."));
					Serial.print(adr[1]);
					Serial.print(F(": "));
					Serial.println(c);
				}
#endif
#if 0
				char s[6];
  				dtostrf(c,4, 1, s);
				s[4] = 0xf8;
				s[5] = 0;
				adr[0] = 0x29;
				_devs->adrGen(2, adr, 7);
				wdt_reset();
				if (_devs->ds2408PioSet(2, adr, 4) == 0xAA) {
					_devs->ds2408PioSet(2, adr, 7);
					_devs->ds2408PioSet(2, adr, 0xe0);
					_devs->ds2408PioSet(2, adr, 0);
					_devs->ds2408PioSet(2, adr, 0);

					wdt_reset();
					for (i = 0; i < 5; i++)
						_devs->ds2408PioSet(2, adr, s[i]);
				}
#endif
		}

		if (cnt-- == 0) {
			Serial.println(F("Error searching"));
			break;
		}
		wdt_reset();
	}

	return j > 0 ? true : false;
}
