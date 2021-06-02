#include <OneWireBase.h>
#include "OwDevices.h"

/* config */
#define MAX_BUS 3
#ifdef AVRSIM
#define MAX_TIMER 2
#define MAX_SWITCHES 4
#define MAX_TIMED_SWITCH 2
#else
#define MAX_TIMER 10
/* per bus 12 addresses and each 5 latches, sometimes long presses additionally */
#define MAX_SWITCHES MAX_BUS * 12 * 5
#define MAX_TIMED_SWITCH 10
#endif
#define MAX_DIMMER 3
#define DEF_SECS 30

/* modes */
#define MODE_ALRAM_POLLING 0x2
#define MODE_ALRAM_HANDLING 0x4
#define MODE_AUTO_SWITCH 0x8

struct _sw_tbl {
	union s_adr src;
	union d_adr dst;
};

struct _dim_tbl {
	union d_adr dst;
	struct {
		unsigned int level : 4;
		unsigned int up : 1;
		unsigned int dn : 1;
	}lvl;
};

enum _pio_mode {
	ON,
	OFF,
	TOGGLE
};

class SwitchHandler
{
	private:
		OwDevices* _devs;
		OneWireBase *ds;
		uint8_t data[10];
		uint8_t mode;
		uint16_t srcData(uint8_t busNr, uint8_t adr1);
		uint8_t dataRead(union d_adr dst, uint8_t adr[8]);
		uint8_t dimStage(uint8_t dim);
		bool actorHandle(union d_adr dst, enum _pio_mode mode);

	public:

		SwitchHandler(OwDevices* devs) { this->_devs = devs; };
		void begin(OneWireBase *ow);
		void loop();
		void initSwTable();
		bool alarmHandler(uint8_t busNr, uint8_t mode);
		bool switchHandle(uint8_t busNr, uint8_t adr1);
		bool switchHandle(uint8_t busNr, uint8_t adr1, uint8_t latch, uint8_t mode);
		bool timerUpdate(union d_adr dst, uint16_t secs);
		bool switchLevel(union d_adr dst, uint8_t level);
		bool setPio(union d_adr dst, uint8_t adr[8], uint8_t d, enum _pio_mode mode);
		bool setLevel(union d_adr dst, uint8_t adr[8], uint8_t id, uint8_t d, uint8_t level);
};