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

#define MAX_BUS 3
#define MAX_ADR 15

extern byte debug;
extern uint8_t min;
extern uint8_t hour;
extern uint8_t sun;
extern uint8_t light;
extern byte light_sensor;

#endif
