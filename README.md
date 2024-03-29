Arduino 1-Wire Homeautomation Base

# Changes
[x] timer based control (different switch table?). Use CLI sw t <bus> <adr> <latch> <bus> <adr> <pio>.
    Default 30 secs at the moment
[x] Host IF for switch table
[x] Host IF for control
[x] Host GPIO signal verified
[x] Switch with same dst does not clear timer if running.
[x] Log latch and switch

# Bugs
    Pressing a button to permanently switch on light is not working
    setting level = 100 turns light off

# TodDo
[ ] (ongoing) Timer based on light with configurable threshold
[ ] Custom timer time per switch
[ ] (ongoing) Host IF for status read
[ ] Light independed timed switch. Currently timed switches only if dark
# Documentation

# Build
Create a post_extra_script.py to copy over to PI:
Import("env", "projenv")
env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "scp ",
        "$BUILD_DIR/${PROGNAME}.hex", "<user>@<ip>:<path>/${PROGNAME}.hex"
    ]), "Copying $BUILD_DIR/${PROGNAME}.hex")
)

Build using PlatformIO: %USERPROFILE%\.platformio\penv\Scripts\platformio.exe run --environment nanoatmega328-deploy

Debugging with "simulation" target

## Main Tasks
Monitors 1-Wire lines for incoming interrupts (alarms) to be used instead of polling.
Controls a DS2482 (one wire master) via I2C master or optional the software OneWire bus
I2C slave interface for PI plus one  GPIO line for alarm indication
Watchdog feature for PI polling to switch over to fallback Arduino soltuion
Simple light switching matrix based on memory optimized addresses

### Alarm handling
Each custom DS2482 generates an alarm signal of > 800 us on the OneWire line. No
other master supports detection of it. But this is a feature of the iButton.
Therefore I need a master to at least monitor the line and signal it to an
other master. The Arduino turned out to be fast and stable to handle basic
light switches in reasonable time compared to a Linux host (threads, updates,
script latency and overhead). So the Arduino controls the light switches.

### Address format and limitations

The adress scheme is aligned with the slave implementation to save space in
the Arduino Nano and speed up the lookup. Only bus and address needs to be known
(and detected in the alarm search).
The latch in the lookup table is only the latch number, no bitmap. Means pressing two
switches at the same time is ignored (first latch taken).

Slaves are based on the OneWireSlave Attiny code uses a format like this:
29 | id | bus | ~id | ~bus | 66 | 77 | CRC
with id: 0 .. F
PIO0/1 are output (usually, but can be configured)
Latch2..7 are input (usually only 4 are used)
A special destination type for other devices (like DS2413) is reserved.
The custom DS2408 implementation also serves a alarm signal of 900 us like
the iButtons do. This avoids frequent polling (would need to be around 200 ms)

### Switching examples ###
A button is pressed. The slave generates an alarm signal on the bus and the Arduino
detects the long signal (> 800 us). It starts polling using the alarm condition
for the (or all) slaves in alarm condition. Reads out the registers and
starts a lookup in the timer table and switch table for commands.

### Measures to overcome limitations
Still the lookup table could be too large and this needs to be mitigated.
- fixed or stable switches in program space: easy to access via different table
  or use custom bootloader for flash storing option
- lookup in EEPROM: cache EEPROM or create new look up table (bus|adr = 4 x 31 lines returns dst by latch index),  max 1 KB

## CLI examples

## I2C format
C3 - channel select like in DS2482
F0 - reset
69 - mode selection: [2] [4]
5A - search first device, wait for search cycle for reading, returns 0 if nothing found or | id | adr [8]
5B - search next device, returns 0 if nothing found or | id | adr [8]
01 - read one event from fifo, status: busy,ok,no_data
02 - add switch entry:  type, bus, adr1, latch, press [0..2], da.bus,da.adr,da.pio
03 - write PIO: type, bus, adr1, pio and release lock, check for an event acknowledging the change
E2 - lock i2c bus and get status, if no events in the queue the lock is released again
78 - Handle ack: pass the sequence number to be acked, this releases the lock


status: alarm bus 3 | alarm bus 2 | alarm bus 1 | alarm bus0
bus select
read out alarm : adr | latch
read out status : write adr | read PIO

### Host Communication

On GPIO low, read status register
Set read pointer:
    Write 0xE1 0xE1
Read one byte (status)
If event data (0x40)
    write 0x1
    status changes to BUSY, wait for OK or NO_DATA
    on ok send read data request
    write 0x96
    read data (on event 9 bytes): type, bus, adr, latch, press, 16 bit data, seq, 0xaa
    seq: 1..0x7f
    read status again and repeat if needed

### Handling events Implementation details:
- check alarm status which resets the gpio, ALARM_STATUS_REGISTER
   iic.i2cWriteSync(iicAdr, 2, Buffer.from([0xE1, 0xA8]));
- read one byte and check for 0x40 otherwise cancel
- request event data, selects status register
  The controller copies the data over from the fifo
  Send CMD_EVT_DATA (= 0x1)
    iic.i2cWriteSync(iicAdr, 1, Buffer.from([0x1]));
    release the bus iic.closeSync();
- read status in a loop till 0
    0 = ok, data is prepared
    1 = busy/waiting to read data
    0x80 no data: return
- select data register, status is READY = 0x4
    iic.i2cWriteSync(iicAdr, 1, Buffer.from([0x96]));
    delay...?
    iic.i2cReadSync(iicAdr, 9, rbuf);
    rbuf[7] contains the sequence number
- send ack
    This will clear the fifo entry and increases the sequence number.
    This also may release the I2C host bus
    iic.i2cWriteSync(iicAdr, 2, Buffer.from([0x78, rbuf[7]]));

## sending commands details
    set read ptr to DS2482_STATUS_REGISTER
    iic.i2cWriteSync(iicAdr, 2, Buffer.from([0xE1, 0xE1]));
    delay(1);
    // wait for ready
    do {
        iic.i2cReadSync(iicAdr, 1, rbuf);
        delay(1);
    } while (rbuf[0] != 0);

    iic.i2cWriteSync(iicAdr, 5, Buffer.from([0x3, bus, id, pio, <level>]));

## Switch configuration

sw : dumps the table
sw r : reads the table from eeprom again
sw s : saves the table into eeprom
sw c : clears the table
sw d <bus> <adr> <latch>: deletes a switch
sw s <bus> <adr> <latch> <dst bus> <dst adr> <dst pio> [type]
	add a switch
sw t <timer type> <bus> <adr> <latch> <dst bus> <dst adr> <dst pio> [type]
	Add a timed switch
    Timed table	timer type: 1 = 20 sec, 2: 30 sec, 3: 1 min, 4: 2 min, 5: 5 mins ...
    Timer on darkness: offset 10
	Timer on darkness with soft off per time if supported (dimmable), time 5 secs: offset 20

    latch 10 .. 17: long press
    latch 0 .. 7: normal press
    latch 20 .. 27: pressing
