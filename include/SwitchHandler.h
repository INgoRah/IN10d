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
/* optimized 8 bit PIO destination with only 8 bit for size optimized
switch table. */
union d_adr_8 {
	uint8_t data;
	struct {
		/* PIO0 = 0, PIO1 = 1 */
		unsigned int pio : 1;
		unsigned int adr : 4;
		unsigned int type : 1;
		unsigned int bus : 2;
	} da;
};

/** target PIO with type and address using the full
 * possible range of PIOs and address. 16 bit */
union pio {
	uint16_t data;
	struct {
		/* PIO0 = 0, PIO1 = 1...PI07 = 7 */
		unsigned int pio : 3;
		unsigned int adr : 8;
		unsigned int bus : 3;
		unsigned int type : 2;
	} da;
};

struct _sw_tbl {
	union s_adr src;
	union d_adr_8 dst;
};

struct _dim_tbl {
	union d_adr_8 dst;
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
		// current latch data to be hanled
		uint8_t cur_latch;
		uint8_t mode;
		uint16_t srcData(uint8_t busNr, uint8_t adr1);
		uint8_t dataRead(union pio dst, uint8_t adr[8]);
		uint8_t dimStage(uint8_t dim);
		uint8_t getType(union pio dst);

	public:

		SwitchHandler(OwDevices* devs) { this->_devs = devs; };
		bool actorHandle(union d_adr_8 dst, enum _pio_mode mode);
		void begin(OneWireBase *ow);
		void loop();
		void initSwTable();
		bool alarmHandler(uint8_t busNr, uint8_t mode);
		bool switchHandle(uint8_t busNr, uint8_t adr1);
		bool switchHandle(uint8_t busNr, uint8_t adr1, uint8_t latch, uint8_t mode);
		bool timerUpdate(union d_adr_8 dst, uint16_t secs);
		bool switchLevel(union pio dst, uint8_t level);
		bool setPio(union pio dst, uint8_t adr[8], uint8_t d, enum _pio_mode mode);
		bool setLevel(union pio dst, uint8_t adr[8], uint8_t id, uint8_t d, uint8_t level);
};