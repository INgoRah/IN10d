#include <main.h>
#include "CmdCli.h"
#include "SwitchHandler.h"
#include "OwDevices.h"
#include <TwiHost.h>

extern TwiHost host;

struct _timer_item {
	/* timestamp of start */
	unsigned long ms;
	/* seconds to switch off */
	uint16_t secs;
	uint8_t base_type;
	uint8_t id;
	/* switch off: */
	union d_adr_8 dst;
}tmr_list[MAX_TIMER];

struct _dim_tbl dim_tbl[MAX_DIMMER];

struct _sw_tbl sw_tbl[MAX_SWITCHES];

/* need switch off time! */
struct _sw_tim_tbl timed_tbl[MAX_TIMED_SWITCH];

SwitchHandler::SwitchHandler()
{
	memset (dim_tbl, 0, sizeof(dim_tbl));
	
	for (int i = 0; i < MAX_TIMER; i++) {
		tmr_list[i].secs = 0;
	}
	cur_latch = 0;
	/* default level for PIR detection on */
	dim_on_lvl = 63;
	light_thr = 225;
}

SwitchHandler::SwitchHandler(OwDevices* devs)  : SwitchHandler()
{
	this->_devs = devs;
}

uint8_t SwitchHandler::bitnumber()
{
	int i, res = 1;

	for (i = 0x1; i <= 0x80; i = i << 1) {
		if (cur_latch & i) {
			cur_latch -= i;
			return res;
		}
		res++;
	}
	/* invalid */
	return 0xff;
}

extern byte pins;
void SwitchHandler::initSwTable()
{
	uint16_t len, pos, mylen;
	uint8_t vers;

	vers = eeprom_read_byte((const uint8_t*)0);
	if (vers > 2) {
		//  0      1     2       3          4          5     6  7      8 ...
		// vers | pos | mode | debug | ligthsensor | pins |  len    | tbl ... | ...
		pos = eeprom_read_byte((const uint8_t*)1); // should be 6
		mode = eeprom_read_byte((const uint8_t*)2);
		debug = eeprom_read_byte((const uint8_t*)3);
		light_sensor = eeprom_read_byte((const uint8_t*)4);
		pins = eeprom_read_byte((const uint8_t*)5);
	} else {
		//  0      1     2    3     4 ...
		// vers | res |   len    | tbl ... | ...
		pos = 2;
		mode = MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH;
		debug = 1;
		light_sensor = 1;
		pins = 0xff;
	}
	len = eeprom_read_word((const uint16_t*)pos);
	pos += 2;
	if (len > sizeof(sw_tbl)) {
		Serial.print(F("table exceed limit to "));
		mylen = sizeof(sw_tbl) - 1;
		Serial.println(mylen);
	} else
		mylen = len;
	if (len != 0xFFFF && vers != 0xff) {
#ifdef DEBUG
		Serial.print(F(" vers="));
		Serial.print(vers);
		Serial.print(F(" tbl@"));
		Serial.print(pos);
		Serial.print(F(" len="));
		Serial.print(len);
		Serial.print(F(" / Max "));
		Serial.print((uint16_t)sizeof(sw_tbl));

#endif
		eeprom_read_block((void*)sw_tbl, (const void*)pos, mylen);
	}
	/* read next (timer) table*/
	pos += len;
	vers = eeprom_read_byte((const uint8_t*)pos);
	if (pos % 2 != 0) {
		Serial.println(F("unaligned"));
	}
	len = eeprom_read_word((const uint16_t*)(pos + 2));
	if (len > sizeof(timed_tbl)) {
		Serial.print(F("tim table exceed, limit to "));
		mylen = sizeof(timed_tbl) - 1;
		Serial.println(mylen);
	} else
		mylen = len;
	pos += 4;
	if (len != 0xFFFF && vers != 0xff) {
#ifdef DEBUG
		Serial.print(F(" timed@"));
		Serial.print(pos);
		Serial.print(F(" timed vers="));
		Serial.print(vers);
		Serial.print(F(" len="));
		Serial.print(len);
		Serial.print(F(" / Max "));
		Serial.print((uint16_t)sizeof(timed_tbl));
#endif
		if (len > sizeof(timed_tbl))
			len = sizeof(timed_tbl);
		eeprom_read_block((void*)timed_tbl, (const void*)pos, mylen);
	}
	vers = eeprom_read_byte((const uint8_t*)0);
	if (vers > 2 && len != 0xffff) {
		pos += len;
		vers = eeprom_read_byte((const uint8_t*)pos);
		pos += 2;
		if (pos % 2 != 0) {
			Serial.println(F("unaligned"));
		}
		len = eeprom_read_word((const uint16_t*)pos);
#ifdef DEBUG
		Serial.print(F(" 16bit@"));
		Serial.print(pos);
		Serial.print(F(" vers="));
		Serial.print(vers);
		Serial.print(F(" len="));
		Serial.print(len);
		Serial.print(F(" / Max 0 "));
#endif
	}
#ifdef DEBUG
	Serial.println();
#endif
}

uint16_t SwitchHandler::getLen(uint8_t max, uint16_t elSize)
{
	uint16_t len = 0, i;
	union d_adr_8 dst;
	union s_adr src;

	for (i = 0; i < max; i++) {
		src.data = sw_tbl[i].src.data;
		dst.data = sw_tbl[i].dst.data;
		if (src.data == 0 ||
			(src.data == 0xffff && dst.data == 0xff))
			continue;
		len += elSize;
	}

	return len;
}

void SwitchHandler::saveSwTable(uint8_t vers_force = 0)
{
	uint16_t pos = 6;
	uint16_t sw_tbl_len = 0;
	uint16_t sw_tim_len = 0;

	sw_tbl_len = getLen(MAX_SWITCHES, sizeof(struct _sw_tbl));
	sw_tim_len = getLen(MAX_TIMED_SWITCH, sizeof(struct _sw_tim_tbl));
	Serial.println();
	// set version
	if (vers_force == 2) {
		// force to vers 2
		eeprom_update_byte((uint8_t*)0, (uint8_t)2);
		eeprom_update_word((uint16_t*)2, sw_tbl_len);
		eeprom_update_block((const void*)sw_tbl, (void*)4, sw_tbl_len);
		//  0      1     2    3     4 ...
		// vers | res |   len    | tbl ... | ...
		pos = 4;
	} else {
		//  0      1     2       3          4          5     6  7      8 ...    8 + len  
		// vers | pos | mode | debug | ligthsensor | pins |  len    | tbl ... | timver | timlen | timtbl ...
		eeprom_update_byte((uint8_t*)0, (uint8_t)3);
		eeprom_update_byte((uint8_t*)1, (uint8_t)6);
		//  0      1     2       3          4          5      6 
		// vers | pos | mode | debug | ligthsensor | pins |  len    | tbl ... | ...
		eeprom_update_byte((uint8_t*)2, (uint8_t)mode);
		eeprom_update_byte((uint8_t*)3, (uint8_t)debug);
		eeprom_update_byte((uint8_t*)4, (uint8_t)light_sensor);
		eeprom_update_byte((uint8_t*)5, (uint8_t)pins);
		eeprom_update_word((uint16_t*)6, sw_tbl_len);
		//pos += 2;
		pos = 8;
		eeprom_update_block((const void*)sw_tbl, (void*)8, sw_tbl_len);
	}
	/* write next (timer) table */
	pos += sw_tbl_len;
	/* timed table, version 2 ... */
	eeprom_update_byte((uint8_t*)pos, (uint8_t)2);
	pos += 2;
	eeprom_update_word((uint16_t*)pos, sw_tim_len);
	pos += 2;
	eeprom_update_block((const void*)timed_tbl, (void*)pos, sw_tim_len);
	pos += sw_tim_len;
	if (vers_force != 2) {
		/* 16 dst table, version 1 ... */
		eeprom_update_byte((uint8_t*)pos, (uint8_t)1);
		pos += 2;
		eeprom_update_word((uint16_t*)pos, 3);
		pos += 2;
		eeprom_update_block((const void*)timed_tbl, (void*)pos, 3);
	}
#ifdef EXT_DEBUG
	Serial.print(F("EEPROM saved, "));
	Serial.print(pos + sw_tim_len);
	Serial.println(F(" bytes"));
#endif
	/* dim table ... */
}

void SwitchHandler::begin(OneWireBase *ow)
{
	this->ds = ow;
	mode = MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH;

	initSwTable();
	/* Specific devices and our own pin having a dimmer feature */

	/* dimmer 1 / stairs OG */
	dim_tbl[0].dst.da.bus = 2;
	dim_tbl[0].dst.da.adr = 7;
	/* dimmer 2 / stairs UG, this is our own pin */
	dim_tbl[1].dst.da.bus = 0;
	dim_tbl[1].dst.da.adr = 9;
	dim_tbl[1].dst.da.pio = 0;
#if 0
	/* dimmer 3 */
	dim_tbl[2].dst.da.bus = 1;
	dim_tbl[2].dst.da.adr = 3;
#endif
}

bool SwitchHandler::timerUpdate(union d_adr_8 dst, uint8_t typ)
{
	int i;
	uint16_t secs;
	uint8_t t = typ % 10;
#define MIN * 60
#define HOUR * (60 * 60)

	switch (t) {
		case 0:
			secs = 1;
			break;
		default:
		case 1:
			secs = 20;
			break;
		case 2:
			secs = 30;
			break;
		case 3:
			secs = 1 MIN;
			break;
		case 4:
			secs = 2 MIN;
			break;
		case 5:
			secs = 5 MIN;
			break;
		case 6:
			secs = 10 MIN;
			break;
		case 7:
			secs = 15 MIN;
			break;
		case 8:
			secs = 30 MIN;
			break;
		case 9:
			secs = 1 HOUR;
			break;
	}
	for (i = 0; i < MAX_TIMER; i++) {
		struct _timer_item* tmr = &tmr_list[i];

		if (tmr->secs == 0 || tmr->dst.data == dst.data) {
			tmr->secs = secs;
			/* set the base type */
			tmr->base_type = typ - t;
			if (tmr->base_type == TYPE_DARK_SOFT)
				dimLevel(dst, &tmr->id);
			tmr->dst.data = dst.data;
			tmr->ms = millis();
#ifdef EXT_DEBUG
			if (debug > 2) {
				printDst8(dst);
				Serial.println();
			}
#endif
			return true;
		}
	}

	return false;
}

void SwitchHandler::status()
{
#ifdef EXT_DEBUG
	for (int i = 0; i < MAX_TIMER; i++) {
		struct _timer_item* tmr = &tmr_list[i];
		Serial.print(F("timer #"));
		Serial.print(i);
		Serial.print(F(": sec="));
		Serial.print(tmr->secs);
		Serial.print(F(" ms="));
		Serial.print(tmr->ms);
		Serial.print(F(" id="));
		Serial.print(tmr->id);
		Serial.print(F(" type="));
		Serial.println(tmr->base_type);
	}
#endif
}

/* Would be nice to suppress the logs here */
uint8_t SwitchHandler::dimDown(struct _timer_item* tmr)
{
	union pio p;
	uint8_t level, d;

	p.data = 0;
	p.da.bus = tmr->dst.da.bus;
	p.da.adr = tmr->dst.da.adr;
	p.da.pio = tmr->dst.da.pio;
	level = dim_tbl[tmr->id].level;

	if (level > 0)
		level--;
	if (p.da.adr == 9 && p.da.bus == 0) {
		analogWrite(5, level * 4);
		d = level * 4;
	}
	else {
		uint8_t adr[8];

		p.da.type = TYPE_DS29X;
		d = dataRead(p, adr);
		setLevel(p, adr, &d, tmr->id, level);
	}
	dim_tbl[tmr->id].level = level;
	if (level == 0) {
		/* final off stop timer */
		tmr->secs  = 0;
		host.addEvent (p, d);
	}

	return level;
}

/**
 * Responsible for timer handling and dimming down if required
 */
void SwitchHandler::loop()
{
	int i;
	static unsigned long hmsec_time = 0;

	if (millis() > (hmsec_time + 95))
		hmsec_time = millis();
	else
		return;

	for (i = 0; i < MAX_TIMER; i++) {
		struct _timer_item* tmr = &tmr_list[i];

		wdt_reset();
		if (tmr->ms == 0) {
			if (tmr->secs == 0)
				/* nothing to do */
				continue;
			else
				dimDown(tmr);
			continue;
		}
		/* this is the off state handling, timer expired */
		if (millis() > tmr->ms + (tmr->secs * 1000)) {
#ifdef DEBUG
			if (debug > 2) {
				log_time();
				Serial.print(F("timer off "));
				Serial.println(tmr->secs);
			}
#endif
			// off state, could be soft off
			tmr->ms = 0;
			/* not yet off, dimming or blinking? */
			if (tmr->base_type == TYPE_DARK_SOFT) {
				dimDown(tmr);
				host.addEvent (tmr->dst, 255, DIMMING_DOWN);
			}
			else {
				/* final off stop timer */
				tmr->secs  = 0;
				/* action with dst */
				actorHandle(tmr->dst, OFF);
			}
		}
	}
}

uint8_t SwitchHandler::dimLevel(union d_adr_8 dst, uint8_t* id)
{
	for (uint8_t i = 0; i < MAX_DIMMER; i++) {
		if (dst.da.bus == dim_tbl[i].dst.da.bus &&
			dst.da.adr == dim_tbl[i].dst.da.adr &&
			dst.da.pio == dim_tbl[i].dst.da.pio) {
			*id = i;
			return dim_tbl[i].level;
		}
	}

	*id = 0xff;
	return 0xFF;
}

uint8_t SwitchHandler::dimLevel(union pio dst, uint8_t* id)
{
	for (uint8_t i = 0; i < MAX_DIMMER; i++) {
		if (dst.da.bus == dim_tbl[i].dst.da.bus &&
			dst.da.adr == dim_tbl[i].dst.da.adr &&
			dst.da.pio == dim_tbl[i].dst.da.pio) {
			*id = i;
			return dim_tbl[i].level;
		}
	}

	*id = 0xff;
	return 0xFF;
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
 * latch = cur_latch (from data[2])
 * press = data[6]
 */
uint16_t SwitchHandler::srcData(uint8_t busNr, uint8_t adr1)
{
	union s_adr src;
	uint8_t v;

	src.data = 0;
	src.sa.bus = busNr;
	src.sa.adr =  adr1 & 0x3f;
	// get (the first if multiple) bit which is set
	// TODO returns 0xff if invalid -> check here
	// or return immediately if cur_latch == 0
	src.sa.latch = bitnumber();
	v = _devs->getVersion(src.sa.bus, src.sa.adr);

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
	if (debug > 1) {
		Serial.print(src.sa.bus, HEX);
		Serial.print(F("."));
		Serial.print(src.sa.adr, HEX);
		Serial.print(F("."));
		if (src.sa.press)
			Serial.print(10 * src.sa.press + src.sa.latch);
		else
			Serial.print(src.sa.latch);
		Serial.print(F(" "));
		if (debug > 1 && data[6] != 0xff) {
			Serial.print(F(" time="));
			Serial.print(data[6] * 32);
		}
		Serial.println();
	}
#endif

	return src.data;
}

uint8_t SwitchHandler::dimStage(uint8_t dim)
{
	switch (dim) {
		case 0:
			return 31;
		case 31:
			return 63;
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
 * @param level level [0..63] converted from percent to 6 bit (level = level * 64 / 100)
 * @return true if successful otherwise false on error
 *
 * @remark This function does not notify the host for any change. Needs to be done by the
 * calling function using the updated value in "d"
 */
bool SwitchHandler::setLevel(union pio dst, uint8_t adr[8], uint8_t* d, uint8_t id, uint8_t level)
{
	uint8_t v;
#ifdef DEBUG
	if (debug > 1) {
		printDst(dst);
		Serial.print(F(" = "));
		Serial.print(*d, HEX);
	}
#endif
	switch (dst.da.type) {
	case TYPE_DS29X:
	default:
		v = _devs->getVersion(dst.da.bus, dst.da.adr);
		if (v > 6) {
			if (level != 0)
				level = level * 4 + 3;
			if (_devs->ds2408xPinSet(dst.da.bus, adr, dst.da.pio, level) != 0xAA)
				return false;
		} else {
			// clear level in data
			*d &= 0x3;
			if (level > 0)
				// set level and switch PIO on
				*d |= (level << 2);
			if (_devs->ds2408PioSet(dst.da.bus, adr, *d) != 0xAA)
				return false;
		}
		break;
	case TYPE_INTERN:
		if (dst.da.bus == 0 && dst.da.adr == 9) {
			if (dst.da.pio == 0) {
				// scale up 0..15 to 0..255
				if (level > 62) {
					digitalWrite(5, HIGH);
					*d = HIGH;
				} else
					analogWrite(5, level * 4);
			}
		}
		break;
	}
#ifdef DEBUG
	if (debug > 1) {
		Serial.print(F(" -> "));
		Serial.println(*d, HEX);
	}
#endif
	dim_tbl[id].level = level;

	return true;
}

/**
 * Low level set the PIO on or off and checking the target state before
 *
 * @param id id in dimLevel cache array
 * @param d data read from PIO before
 * @return true if switched (was not in the target state), otherwise false
 * @remark This function notifies the host for any change.

 */
/* TODO check for light at the light sensor ...
	bus == 0, adr == 1, pio = 1
	light_sensor = 1;
*/
bool SwitchHandler::setPio(union pio dst, uint8_t adr[8], uint8_t d, enum _pio_mode state)
{
	uint8_t pio, r;

#ifdef EXT_DEBUG
	if (debug > 2)
		printDst(dst);
#endif
	if (dst.da.type == TYPE_INTERN) {
		/* special case: our own pin, is a dimmer, no need to handle here */
		// PIO0: PWM on pin 5
		// D3 - Relais ouput, negative polarity - active switching GND
		// D5 - PWM output (dimed LED) via open coollector transistor
		// D2 - PIR (Note: PIO2 maps to D10)
		// D6 - used for alarm signal to host (class TwiHost)
		// 13 LED
		if (dst.da.pio == 0 || dst.da.pio == 5)
			return false;

		uint8_t pin = dst.da.pio;
		/* special mapping */
		switch (dst.da.pio) {
			case 1:
				pin = 9;
				break;
			case 2:
				pin = 12;
				break;
			case 3:
				pin = 10;
				break;
			case 4:
				pin = 11;
				break;
		}
#ifdef EXT_DEBUG
		if (debug > 2) {
			Serial.print(F(" pin "));
			Serial.print(pin);
			if (state == OFF)
				Serial.println(F(" OFF"));
			else
				Serial.println(F(" ON"));
		}
#endif
		/* TODO State TOGGLE not supported! */
		if (state == OFF)
			digitalWrite(pin, 1);
		else
			digitalWrite(pin, 0);
		// todo use cache as well
		host.addEvent (dst, (1 << pin));
		return true;
	}
	/* turn to bitmask (da.pio = 0,1,2 >> pio = 1,2,4) */
	pio = 1 << dst.da.pio;

	switch (state) {
		case ON:
			/* timer mode or force */
			if ((d & pio) == 0) {
				return false;
			}
			d &= ~(pio);
			break;
		case OFF:
			/* already off */
			if (d & pio) {
				return false;
			}
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
#ifdef DEBUG
	if (debug > 1) {
		printDst(dst);
		if (d & pio)
			Serial.print(F(" OFF"));
		else
			Serial.print(F(" ON"));
		Serial.print(F(" -> "));
		Serial.println(d, HEX);
	}
#endif
	r = _devs->ds2408PioSet(dst.da.bus, adr, d);

	// read back for cross check in upper layer
	return (r == 0xAA);
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
	// TODO new devices will get a value 0..254
	return switchLevelStep(dst, level * 63 / 100);
}

/**
 * Switch to a dedicated level by steps (0..63) or simply on or off.
 * If set already it will ignore and return false
 * level 0 .. 63 64 stages.
 * */
bool SwitchHandler::switchLevelStep(union pio dst, uint8_t level)
{
	uint8_t adr[8], d, id;
	bool ret = false;

	dst.da.type = getType(dst);
	if (dst.da.type != TYPE_INTERN) {
		_devs->adrGen(dst.da.bus, adr, dst.da.adr);
		d = _devs->ds2408PioGet(dst.da.bus, adr);
	}
	dimLevel(dst, &id);
	if (id != 0xff) {
		ret = setLevel(dst, adr, &d, id, level);
	} else {
		// 	addEvent is done in setPio
		if (level < 32)
			ret = setPio (dst, adr, d, OFF);
		else
			ret = setPio (dst, adr, d, ON);
	}
	if (dst.da.type != TYPE_INTERN)
		// forced read back for cross check
		d = _devs->ds2408PioGet(dst.da.bus, adr, true);
	else
		d = level;
	host.addEvent (dst, d);

	return ret;
}

/* if any timer was running (unplanned reset) force switch off */
bool SwitchHandler::initialStates()
{
	union pio p;

	if (_devs->getVersion(2,7) != 0xff) {
		p.da.bus = 2;
		p.da.adr = 7;
		p.da.pio = 0;
		p.da.type = getType(p);
		wdt_reset();
		switchLevelStep(p, 0);
	}
	if (_devs->getVersion(0,2) != 0xff) {
		p.da.bus = 0;
		p.da.adr = 2;
		p.da.pio = 0;
		p.da.type = getType(p);
		wdt_reset();
		switchLevelStep(p, 0);
	}
#if 0
	p.da.bus = 3;
	p.da.adr = 1;
	if (_devs->getVersion(2,7) != 0xff) {
		wdt_reset();
		switchLevelStep(p, 0);
	}
#endif
	return true;
}

/* Reads out PIO data and checks for dimmer, calls toggle or set
   functions for PIO or dimmer.
 @return true if successfully switched, false in case of an error or
 no action was needed (if switch is already in that state)
*/
bool SwitchHandler::actorHandle(union d_adr_8 dst, enum _pio_mode state)
{
	uint8_t adr[8], d, dim, id;
	union pio p;
	bool ret = false;

	p.data = 0;
	p.da.bus = dst.da.bus;
	p.da.adr = dst.da.adr;
	p.da.pio = dst.da.pio;
	p.da.type = getType(p);

	if (p.da.type != TYPE_INTERN)
		d = dataRead(p, adr);
	dim = dimLevel(p, &id);

	if (id == 0xff) {
		// simply pass the state
		ret = setPio (p, adr, d, state);
		goto actor_out;
	}
	// this is a dimmer
	switch (state) {
		case TOGGLE:
			// toggle level: off - 1 - 2 - off
			dim = dimStage(dim);
			if (dim == 0){
				timerUpdate(dst, TYPE_DARK_SOFT);
				return true;
			} else
				ret = setLevel (p, adr, &d, id, dim);
			break;
		case ON:
			if (dim > 0) {
				/* was on before, don't start timer */
				return false;
			}
			ret = setLevel(p, adr, &d, id, dim_on_lvl);
			break;
		case OFF:
		default:
			ret = setLevel(p, adr, &d, id, 0);
	}

actor_out:
	if (ret) {
		d = _devs->ds2408PioGet(p.da.bus, adr);
		host.addEvent (dst, (uint16_t)d);
	}
	return ret;
}

//bool SwitchHandler::actorHandle(union pio p, enum _pio_mode state)

/* latch in cur_latch - bit mask to be converted to bit number
   uses data[1] for actual IO status
   data[6] for press time */
bool SwitchHandler::switchHandle(uint8_t busNr, uint8_t adr1)
{
	union s_adr src;
	uint8_t i;

	src.data = srcData(busNr, adr1);
	host.addEvent (SRC_CHANGE, src.data, data[0] | (data[1] << 8));

	// todo: signal alarm only here once the data is available
	//mode = MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH;
	if ((mode & MODE_AUTO_SWITCH) == 0)
		return false;
	/* Timed handler: reset or start timer
	 * based on PIR or standard light (bath)
	 * PIR needs 1 min of initialization, ignore after start
	 * Single trigger mode for 3 secs
	bool handled;?? */
	for (i = 0; i < MAX_TIMED_SWITCH; i++) {
		struct _sw_tim_tbl* p = &timed_tbl[i];
		if (src.data == p->src.data) {
			if (!(p->dst.data == 0 ||
				  p->dst.data == 0xFF))  {
				/* get the type and check for light if needed */
				if (light < light_thr && p->type > TYPE_DARK) {
#ifdef EXT_DEBUG
					if (debug > 3) {
						Serial.print(F("not dark "));
						Serial.print(light);
						Serial.print(F(" < "));
						Serial.print(light_thr);
						Serial.print(F(", TYP "));
						Serial.println(p->type);
					}
#endif
					continue;
				}
#ifdef EXT_DEBUG
				if (debug > 2) {
					Serial.print(F("timer #"));
					Serial.print(i);
					Serial.print(F(" -> "));
				}
#endif
				/* switch press:
				*   - if off: switch on and start timer
				*   - if on with timer running: reset timer
				*   - if on without timer: switch off
				*/
				/* default action: switch on and start timer for off */
				if (actorHandle(p->dst, ON)) {
					/*  on or timer running: set or reset timer */
					timerUpdate(p->dst, p->type);
				} else {
					/* switched already on. Is timer running?
					   If yes, restart */
					struct _timer_item* tmr = timerItem(p->dst.data);
					if (tmr != NULL)
						tmr->ms = millis();
				}
			}
		}
	}
	wdt_reset();
	/*
	 * Standard handler: from 5 bit adr (1..1f) and 3 bit data[2] (0..7)
	 * Get target switch
	 */
	for (i = 0; i < MAX_SWITCHES; i++) {
		if (src.data == sw_tbl[i].src.data) {
#ifdef EXT_DEBUG
			if (debug > 2) {
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

/* latch as bitmask with only one bit set */
bool SwitchHandler::switchHandle(uint8_t busNr, uint8_t adr1, uint8_t latch)
{
	/* fill data for use in switchHandle */
	cur_latch = latch;
	// ignoring current state of PIOs
	this->data[1] = 0xff;
	// no press time info
	this->data[6] = 0xff;
	return switchHandle(busNr, adr1);
}

bool SwitchHandler::alarmHandler(uint8_t busNr)
{
	uint8_t adr[8];
	uint8_t j = 0;
	uint8_t cnt = 10;
	bool ret;

	//ds = bus[busNr];
	ret = ds->selectChannel(busNr);
	if (!ret)
		// this could be a timeout or other issue
		// must be repeated
		return false;
	ret = ds->reset();
	if (!ret)
		return false;
	ds->reset_search();
	while (ds->search(adr, false)) {
		j++;
#ifdef DEBUG
		if (debug > 0 && adr[0] != 0x28) {
			log_time();
			Serial.print(busNr);
			Serial.print(F("@"));
			for (uint8_t i = 0; i < 8; i++) {
				if (adr[i] < 0x10)
					Serial.write('0');
				Serial.print(adr[i], HEX);
			}
		}
#endif
		//mode = MODE_ALRAM_HANDLING | MODE_ALRAM_POLLING | MODE_AUTO_SWITCH;
		if ((mode & MODE_ALRAM_HANDLING) == 0) {
			// interrupt to host
			digitalWrite (HOST_ALRM_PIN, LOW);
			return true;
		}
		if (adr[0] == 0x29) {
			uint8_t res, to = 30;

			res = _devs->ds2408RegRead(busNr, adr, data);
			/* fill data for use in switchHandle */
			if (res == 0xaa || res == 0xff) {
				// loop over all set bits
				// cur_latch is reduced by each call to bitnumber
				cur_latch = data[2];
#ifdef DEBUG
				if (debug > 0) {
					Serial.print(F(" = "));
					Serial.print(cur_latch, HEX);
					Serial.print(F(" S="));
					Serial.println(data[5], HEX);
				}
#endif
				if (data[5] & 0x40) {
					/* status in 5 signals a dimming down */
					host.addEvent (DIMMING_DOWN, busNr, adr[1], 255);
				}
				while (cur_latch != 0 && to > 0) {
					wdt_reset();
					switchHandle(busNr, adr[1]);
					to--;
				}
			}
		}
		if (adr[0] == 0x28) {
				uint8_t hum;
				uint16_t c = _devs->tempRead(busNr, adr, 1, &hum);
				if (c == 0xffff)
					return false;
				host.addEvent (TEMP_CHANGE, busNr, adr[1], c);
				if (hum != 0xff)
					host.addEvent (HUMIDITY_CHANGE, busNr, adr[1], hum);
#ifdef DEBUG
				if (debug > 2) {
					Serial.print(busNr);
					Serial.print(F("."));
					Serial.print(adr[1]);
					Serial.print(F(": "));
					Serial.print(c / 16.0);
					Serial.println(F(" C"));
				}
#endif
		}
		cnt--;
		if (ds->last_err || cnt == 0) {
#ifdef DEBUG
			log_time();
			Serial.print(F("Error searching = "));
			Serial.println(ds->last_err);
#endif
		}
		wdt_reset();
	}

	return j > 0 ? true : false;
}
