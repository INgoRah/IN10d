# Arduino 1-Wire Homeautomation Fallback

## Main Tasks
Monitors a 1-Wire line for incoming interrupts (alarms) instead of polling.
Controls a DS2482 (one wire master) via I2C master
I2C slave interface for PI plus one  GPIO line for alarm indication
Watchdog feature for PI polling to switch over to fallback Arduino soltuion

# I2C format
C3 - channel select like in DS2482
F0 - reset 
69 - mode selection: [2] [4]
5A - search first device, wait for search cycle for reading, returns 0 if nothing found or | id | adr [8]
5B - search next device, returns 0 if nothing found or | id | adr [8]
