#ifndef _MAIN_H
#define _MAIN_H

#if ARDUINO >= 100
#include "Arduino.h"			 // for delayMicroseconds, digitalPinToBitMask, etc
#else
#include "WProgram.h"			// for delayMicroseconds
#include "pins_arduino.h"	// for digitalPinToBitMask, etc
#endif
#include <avr/wdt.h>
#include "version.h"

#define MAX_BUS 4
#define MAX_ADR 13

extern void log_time();
extern void printDst8(union d_adr_8 dst);
extern void printDst(union pio dst);
extern void printSrc(union s_adr src);

extern byte debug;
extern uint8_t min;
extern uint8_t hour;
extern uint8_t sun;
extern uint8_t light;
extern byte light_sensor;
extern unsigned long host_lock;

#endif
