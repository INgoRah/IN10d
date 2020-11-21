Arduino 1-Wire Homeautomation Fallback

# Changes

# TodDo
[ ] timer based control (different switch table?)
[ ] Host IF alarm indication debugging with PI
[ ] Host IF for status read
[ ] Host IF for switch table

# Documentation

## Main Tasks
Monitors 1-Wire lines for incoming interrupts (alarms) to be used instead of polling.
Controls a DS2482 (one wire master) via I2C master or optional the software OneWire bus
I2C slave interface for PI plus one  GPIO line for alarm indication
Watchdog feature for PI polling to switch over to fallback Arduino soltuion
Simple light switching matrix based on memory optimized addresses

## Address format and limitations

Slaves based on the OneWireSlave Attiny code uses a format like this:
29 | id | bus | ~id | ~bus | 66 | 77 | CRC
with id: 0 .. F
PIO0/1 are output
Latch2..6 are input

## I2C format
C3 - channel select like in DS2482
F0 - reset 
69 - mode selection: [2] [4]
5A - search first device, wait for search cycle for reading, returns 0 if nothing found or | id | adr [8]
5B - search next device, returns 0 if nothing found or | id | adr [8]
01 - read one event from fifo, status: busy,ok,no_data
02 - add switch entry:  type, bus, sa.adr, sa.latch, da.bus,da.adr,da.pio

status: alarm bus 3 | alarm bus 2 | alarm bus 1 | alarm bus0
bus select
read out alarm : adr | latch
read out status : write adr | read PIO

### current switching

0.8.5 -> 0.1.0
1.11.5 -> 2.3.0 // Eingang -> Oben Flur
1.1.7 -> 1.11.1 // küchentisch -> Eingang
1.11.7 -> 1.1.0 
2.3.4 -> 1.11.1 // Flur -> Eingang
2.1.6 -> 2.3.0
2.2.3 -> 2.3.0
3.7.7 -> 1.1.0
3.7.5 -> 0.1.0

Examples

# write config
    |BTN|PIN|POL|SW 1  2  3  4  5  6  7|CFG 1  2  3  4  5  6  7 |FEA|OFF|MAJ|MIN|TYP

cfg 7 w 38 03 FF FF FF FF FF 01 FF FF FF FF FF 02 FF FF FF FF 02 FE 05 55 0
FF 3 FF FF FF FF FF FF FF FF FF FF FF FF FF FF 1 FF FF FE 6 1 2 0 D9 70
 0  1 2  3 4  5  6  7   8 9  10 11 12 13 14 15 1617 18 19 20212223 