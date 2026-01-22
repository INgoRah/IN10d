# Bugs
Pressing a button to permanently switch on light is not working setting level = 100 turns light off

# Improvements

[ ] Separate latch (input) and switch (output) to send two message in case
of auto, switch table or host controlled switch  
[ ] (ongoing) Timer based on light with configurable threshold
    - update brightness on change to slaves 2.7 and 0.2
[ ] Custom timer time per switch  
[ ] (ongoing) Host IF for status read  
[ ] Light independed timed switch. Currently timed switches only if dark  
[ ] Support 16 bit destination address for all PIOs
[ ] Support timed switch from host
[ ] Timed switches retrigger/switch. Now every latch retriggers timer, but it could
    instead switch off again. This needs to be configurable per switch
[ ] Slave based timer time configurable (needs change in owslave)

# PICO Support

## Adaptations

- OneWire Montor (Pull-Up)
- Ports (Analog, digital, PWM)
- IO changes

## Features

### MQTT Receiver (Port switcher)

### MQTT Sender (Temperature, status)

### Change switch table data struct

- address up to 8 PIOs (3 bits)
- optional address up to 8 busses (3 bits)

### Support non custom Addresses

- DS1820 based sensors (requires polling)
- full 1-wire address (except CRC)

# Switch Table
= Switches =
1.11.5 -> 2.3.0 (2D42 | 86)
2.3.4 -> 1.B.1 (D04 | 57)
1.1.7 -> 1.C.1 (5C2 | 59)
2.1.6 -> 2.3.0 (584 | 86)
2.2.3 -> 2.3.0 (8C4 | 86)
1.11.7 -> 1.1.1 (2DC2 | 43)
1.11.27 -> 1.5.0 (2DE2 | 4A)
1.11.25 -> 1.5.1 (2D62 | 4B)
0.6.7 -> 0.5.0 (19C0 | A)
0.6.4 -> 0.8.0 (1900 | 10)
0.5.5 -> 0.8.0 (1540 | 10)
0.5.7 -> 0.6.0 (15C0 | C)
0.5.3 -> 0.1.0 (14C0 | 2)
1.12.7 -> 1.5.1 (31C2 | 4B)
1.12.5 -> 1.1.1 (3142 | 43)
1.12.15 -> 2.7.0 (33C2 | 8E)
1.12.25 -> 0.9.0 (3162 | 12)
1.12.26 -> 2.7.0 (31A2 | 8E)
1.1.6 -> 1.C.1 (582 | 59)
1.7.2 -> 0.9.0 (1C82 | 12)
0.8.5 -> 0.6.0 (2140 | C)
0.8.7 -> 0.8.0 (21C0 | 10)
1.4.6 -> 1.1.1 (1182 | 43)
1.4.4 -> 1.C.1 (1102 | 59)
1.7.6 -> 1.5.0 (1D82 | 4A)
1.7.4 -> 1.5.1 (1D02 | 4B)
1.1.18 -> 1.1.0 (612 | 42)
1.7.2 -> 2.7.0 (1C82 | 8E)
1.1.16 -> 1.5.1 (592 | 4B)
0.6.5 -> 0.1.0 (1940 | 2)
2.2.23 -> 2.7.0 (8E4 | 8E)
1.7.5 -> 1.1.1 (1D42 | 43)
1.4.14 -> 1.5.1 (1112 | 4B)
1.12.8 -> 1.B.1 (3202 | 57)
1.12.17 -> 1.5.0 (31D2 | 4A)
1.7.3 -> 1.B.1 (1CC2 | 57)
1.7.7 -> 0.1.0 (1DC2 | 2)
1.4.16 -> 1.3.0 (1192 | 46)
Size=114/540
= Timed =
Type 21 0.9.1 -> 0.9.0
Type 21 2.7.2 -> 2.7.0
Type 22 1.11.23 -> 0.9.1
Size=9/40

  0: 2 FF 72 0 42 2D 86 4 D 57 C2 5 59 84 5 86 
 16: C4 8 86 C2 2D 43 E2 2D 4A 62 2D 4B C0 19 A 0 
 32: 19 10 40 15 10 C0 15 C C0 14 2 C2 31 4B 42 31 
 48: C2 33 8E 62 31 12 A2 31 8E 82 5 59 82 1C 12 
 64: 21 C C0 21 10 82 11 43 2 11 59 82 1D 4A 2 
 80: 4B 12 6 42 82 1C 8E 92 5 4B 40 19 2 E4 8 
 96: 42 1D 43 12 11 4B 2 32 57 D2 31 4A C2 1C 57 
112: 1D 2 92 11 46 2 15 8 0 15 40 24 12 15 84 
128: 1C 8E 24 12 15 84 1C 8E 8 D2 2C C2
142:

  0: 3 6 E 2 1 FF 72 0 42 2D 86 4 D 57 C2 5 
 16: 59 84 5 86 C4 8 86 C2 2D 43 E2 2D 4A 62 2D 4B 
C0 19 A 0 19 10 40 15 10 C0 15 C C0 14 2 C2 
31 4B 42 31 43 C2 33 8E 62 31 12 A2 31 8E 82 5 
59 82 1C 12 40 21 C C0 21 10 82 11 43 2 11 59 
82 1D 4A 2 1D 4B 12 6 42 82 1C 8E 92 5 4B 40 
19 2 E4 8 8E 42 1D 43 12 11 4B 2 32 57 D2 31 
4A C2 1C 57 C2 1D 2 92 11 46 2 40 8 0 15 40 
24 12 15 84 1C 8E 1 8E 3 0 15 40 
