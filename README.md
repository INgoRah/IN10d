# Arduino_ds2482_emu
Emulates a DS2482 (one wire master) on an Arduino
- integration into owfs not yet working, basic flow verified on script base
- search handling not yet verified (may not be working)

# Todos
- add new custom register for alarm detection and watchdog polling

# I2C format
C3 - channel select like in DS2482
F0 - reset 
69 - mode selection: [2] [4]
5A - search first device, wait for search cycle for reading, returns 0 if nothing found or | id | adr [8]
5B - search next device, returns 0 if nothing found or | id | adr [8]

void test_detect(OneWireBase *ds, uint8_t t_slot, uint8_t t_w0l, uint8_t t_w1l, uint8_t t_msr, uint8_t t_rc)
{
	byte addr[8], i, j;
	ds->t_slot = t_slot;
	ds->t_w0l = t_w0l;
	ds->t_w1l = t_w1l; 
	ds->t_rc = t_rc;
	ds->t_msr = t_msr;

	Serial.print(t_slot);
	Serial.write(' ');
	Serial.print(t_w0l);
	Serial.write(' ');
	Serial.print(t_w1l);
	Serial.write(' ');
	Serial.print(t_msr);
	Serial.write(' ');
	Serial.println(t_rc);
	/* slot	low	int	rc 	rd
	*/
	ds->reset_search();
	if (ds->reset() == 0) {
			Serial.println("	No devices!");
			return;
	}
	j = 0;
	while (j < 7 && ds->search(addr)) {
		Serial.print("	#");
		Serial.print(j++);
		for (i = 0; i < 8; i++) {
			Serial.print(' ');
			Serial.print(addr[i], HEX);
		}
		Serial.println();
	}
	if (j < 3 || j > 8)
		Serial.print("?");
}

void ow_line_check(OneWireBase *ds) {
	byte slot[] = { 80 };
	byte t_msr[] = { 13, 14, 15 };
	byte t_w0l[] = { 70 };
	byte t_rc[] = { 4, 5, 6, 7 };
	byte t_w1l[] = { 9, 10, 11 };
	byte s, w0l, rc, msr, w1l;

	Serial.println("line and device check");
	for (s = 0; s < sizeof(slot); s++)
		for (w0l = 0; w0l < sizeof(t_w0l); w0l++)
			for (msr = 0; msr < sizeof(t_msr); msr++)
				for (rc = 0; rc < sizeof(t_rc); rc++)
					for (w1l = 0; w1l < sizeof(t_w1l); w1l++) {
						Serial.print(".");
						if (t_w0l[w0l] > slot[s]) {
							//Serial.print(F("w0l > slot "));
							continue;
						}
						// T_SLOT - T_W1L + T_RC
						if (t_w1l[w1l] + t_rc[rc] > slot[s]) {
							continue;
						}
						// T_MSR - T_W1L
						if (t_msr[msr] < t_w1l[w1l]) {
							//Serial.print(F("msr < w1l "));
							continue;
						}
						// T_W1L - T_RC)
						if (t_w1l[w1l] < t_rc[rc]) {
							//Serial.print(F("w1l < rc "));
							continue;
						}
						test_detect (ds, slot[s], t_w0l[w0l], t_w1l[w1l], t_msr[msr], t_rc[rc]);
						delay (10);
					}
}

