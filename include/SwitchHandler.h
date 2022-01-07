#include <OneWireBase.h>
#include "OwDevices.h"

/* config */
#ifdef AVRSIM
#define MAX_TIMER 2
#define MAX_SWITCHES 4
#define MAX_TIMED_SWITCH 3
#else
#define MAX_TIMER 6
/* per bus 12 addresses and each 5 latches, sometimes long presses additionally */
#define MAX_SWITCHES MAX_BUS * 12 * 5
#define MAX_TIMED_SWITCH 10
#endif
#define MAX_DIMMER 4
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

enum tim_type {
	/** Timer always, hard off per time */
	TYPE_DEF,
	TYPE_30S		/* 1 */,
	TYPE_1MIN		/* 2 */,
	TYPE_2MIN		/* 3 */,
	TYPE_5MIN		/* 4 */,
	TYPE_10MIN		/* 5 */,
	TYPE_15MIN		/* 6 */,
	TYPE_30MIN		/* 7 */,
	TYPE_1H 		/* 8 */,
	/** Timer on darkness with hard off per time */
	TYPE_DARK = 10,
	TYPE_DARK_30S	/* 11 */,
	TYPE_DARK_1MIN	/* 12 */,
	TYPE_DARK_2MIN	/* 13 */,
	TYPE_DARK_5MIN	/* 14 */,
	TYPE_DARK_10MIN	/* 15 */,
	TYPE_DARK_15MIN	/* 16 */,
	TYPE_DARK_30MIN	/* 17 */,
	TYPE_DARK_1H 	/* 18 */,
	/** Timer on darkness with soft off per time
	 *  if supported (dimmable), time 5 secs */
	TYPE_DARK_SOFT = 20,
	TYPE_DARK_SOFT_30S		/* 21 */,
	TYPE_DARK_SOFT_1MIN		/* 22 */,
	TYPE_DARK_SOFT_2MIN		/* 23 */,
	TYPE_DARK_SOFT_5MIN		/* 24 */,
	TYPE_DARK_SOFT_10MIN	/* 25 */,
	TYPE_DARK_SOFT_15MIN	/* 26 */,
	TYPE_DARK_SOFT_30MIN	/* 27 */,
	TYPE_DARK_SOFT_1H 		/* 28 */,
	TYPE_DARK_SOFT_INVAL	/* 29 */,
	/** Timer on darkness with blinking off per time */
	TYPE_DARK_BLINK = 30,
	TYPE_DARK_BLINK_30S		/* 31 */,
	TYPE_DARK_BLINK_1MIN	/* 32 */,
	TYPE_DARK_BLINK_2MIN	/* 33 */,
	TYPE_DARK_BLINK_5MIN	/* 34 */,
	TYPE_DARK_BLINK_10MIN	/* 35 */,
	TYPE_DARK_BLINK_15MIN	/* 36 */,
	TYPE_DARK_BLINK_30MIN	/* 37 */,
	TYPE_DARK_BLINK_1H 		/* 38 */,
};

/* 5 seconds soft off time */
#define SOFT_TIME 5

struct _sw_tim_tbl {
	uint8_t type;
	union s_adr src;
	union d_adr_8 dst;
};

struct _dim_tbl {
	union d_adr_8 dst;
	uint8_t level;
};

enum _pio_mode {
	ON,
	OFF,
	TOGGLE
};

extern struct _sw_tbl sw_tbl[MAX_SWITCHES];
extern struct _sw_tim_tbl timed_tbl[MAX_TIMED_SWITCH];
extern struct _dim_tbl dim_tbl[MAX_DIMMER];

class SwitchHandler
{
	private:
		OwDevices* _devs;
		OneWireBase *ds;
		uint8_t data[10];
		// current latch data to be hanled
		uint8_t cur_latch;
		uint16_t srcData(uint8_t busNr, uint8_t adr1);
		uint8_t dataRead(union pio dst, uint8_t adr[8]);
		uint8_t dimStage(uint8_t dim);
		uint8_t getType(union pio dst);
		uint8_t bitnumber();
		bool timerUpdate(union d_adr_8 dst, uint8_t typ);
		uint8_t dimDown(struct _timer_item* tmr);
		uint8_t dimLevel(union pio dst, uint8_t* id);
		uint8_t dimLevel(union d_adr_8 dst, uint8_t* id);
		bool switchLevelStep(union pio dst, uint8_t level);
		bool setPio(union pio dst, uint8_t adr[8], uint8_t d, enum _pio_mode state);
		bool setLevel(union pio dst, uint8_t adr[8], uint8_t* d, uint8_t id, uint8_t level);
	public:
		uint8_t mode;
		uint8_t light_thr;
		uint8_t dim_on_lvl;
		SwitchHandler();
		SwitchHandler(OwDevices* devs);
		void status();
		bool actorHandle(union d_adr_8 dst, enum _pio_mode state);
		void begin(OneWireBase *ow);
		void loop();
		void initSwTable();
		void saveSwTable();
		bool alarmHandler(uint8_t busNr);
		bool switchHandle(uint8_t busNr, uint8_t adr1);
		bool switchHandle(uint8_t busNr, uint8_t adr1, uint8_t latch);
		bool switchLevel(union pio dst, uint8_t level);
};