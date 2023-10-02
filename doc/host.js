const i2c = require('i2c-bus');
var gpio = require('rpi-gpio');

var iicAdr = 0x2f;
var lastSeq = 0xff;
var alarm = true;

createState('javascript.0.DimStairUG', 0);
createState('javascript.0.DimStairOG', 0);
createState('javascript.0.DimBildEG', 0);
createState('javascript.0.Umlaufpumpe', 0);
createState('javascript.0.log', "started");
createState('javascript.0.verbose', 0);
createState("javascript.0.DimLevel", 15);
createState('javascript.0.SwitchLevel', 220);

createState('javascript.0.StairUGActivity', 0);
createState('javascript.0.StairOGActivity', 0);
createState('javascript.0.devices.brightness', 250);
createState('javascript.0.devices.temp_2_1', 21);
createState('javascript.0.devices.temp_2_4', 19);
createState('javascript.0.devices.hum_2_4', 50);
//createState('javascript.0.devices.temp_1_7', 22);
//createState('javascript.0.devices.temp_1_12', 22);
//createState('javascript.0.devices.temp_0_5', 19);

var obj = {
    type:   'number',
    unit:   '°C',
    read:   true,
    write:  true,
    isFloatComma: true,
    role:   'level.temperature',
    desc:   '1wire sensor'
};
createState('javascript.0.devices.temp_1_7', 22, true,
    {
        name: 'temp_1.7',
        type:   'number',
        unit:   '°C',
        role:   'level.temperature',
        desc:   '1wire sensor'
     },
    {
        id:         "28.0701F8FE667705",
        property:   "temperature"
    },
    null);

createState('javascript.0.devices.temp_1_12', 20, true,
    {
        name: 'temp_1.12',
        type:   'number',
        unit:   '°C',
        role:   'level.temperature',
        desc:   '1wire sensor'
     },
    {
        id:         "28.0C01F3FE6677A5",
        property:   "temperature"
    },
    null);

createState('javascript.0.devices.temp_0_5', 20, true, obj, {
    id:         "28.0500FAFF667705",
    property:   "temperature" }, null );

createState('javascript.0.devices.temp_3_1', 20, true, obj, {
    id:         "28.0103FEFC6677BE",
    property:   "temperature" }, null );

if (getState ('javascript.0.Umlaufpumpe').val == 1)
    setState ('javascript.0.Umlaufpumpe', 0);

gpio.reset();
//gpio.setMode(gpio.MODE_RPI);
// Pin #7 = GPIO6 using git co feature/add-orange-pi-zero on rpi-gpio
gpio.setup(7, gpio.DIR_IN, gpio.EDGE_BOTH, function (err) {
});
alarm = true;
events();
alarm = false;
setTimeout (events, 50);
sendTime();

function sendTime()
{
    const rbuf = Buffer.alloc(1);
    const iic = i2c.openSync(0);
    var d = new Date();
    iic.i2cWriteSync(iicAdr, 4, Buffer.from([0x40, d.getHours() , d.getMinutes(), isAstroDay()]));

    iic.i2cReadSync(iicAdr, 1, rbuf);
    //log ("time sync " + rbuf[0].toString(16));
    iic.closeSync();
}

function delay(ms) {
    var start = new Date().getTime();
    for (var i = 0; i < 1e7; i++) {
      if ((new Date().getTime() - start) > ms){
        break;
      }
    }
  }
  
function powerImp()
{
    var val = getState('javascript.0.power.total').val;
    setState('javascript.0.power.total', val + 1);
}

function poll_sw_event(bus, id, pio) {
    return true;
}

function events() {
    const rbuf = Buffer.alloc(10);
    var iic = i2c.openSync(0);
    var cnt = 50;
    var err = 0;
    var act = 0;
    var verbose = getState('javascript.0.verbose').val;

    do {
        if (err)
            log ("Error case @" + act + "...");
        try {
            //if (alarm == false)
            //    return;
            // check the status first?
            // check alarm status which resets the gpio
            iic = i2c.openSync(0);
            iic.i2cWriteSync(iicAdr, 2, Buffer.from([0xE2, 0xA8]));
            act = 1;
            delay(1);
            iic.i2cReadSync(iicAdr, 1, rbuf);
            // 0x40 = event data
            if ((rbuf[0]) == 0xff)
                continue;
            if ((rbuf[0] & 0x40) == 0x0) {
                // should have released the lock
                iic.closeSync();
                if (alarm)
                    setTimeout (events, 50);
                //log("no more events");
                return;
            }
            act = 2;
            //log ("Alarm register = " + rbuf[0].toString(16));
            /* request event data, selects status register.
               The controller copies the data over from the fifo */
            // command CMD_EVT_DATA (1)
               iic.i2cWriteSync(iicAdr, 1, Buffer.from([0x1]));
            iic.closeSync();
            do {
                delay(5);
                /* read status
                   0 = ok, data is prepared
                   1 = busy/waiting to read data
                   0x02 = processing and should not occur
                   0x40 = event data
                   0x80 no data */
                act = 3;
                iic = i2c.openSync(0);
                iic.i2cReadSync(iicAdr, 1, rbuf);
                iic.closeSync();
                //log ("Status register = " + rbuf[0].toString(16));
                if ((rbuf[0]) == 0xff)
                    continue;
                if (rbuf[0] == 0x80) {
                    log ("no data / handled before? " + rbuf[0].toString(16));
                    return;
                }
            } while (cnt-- > 0 && (rbuf[0] & 0x0f) == 0x1);
            /*
            if ((rbuf[0] & 0x3) != 0x0) {
                log (lastSeq.toString(16) + ": timeout, issue with stat = " + rbuf[0].toString(16));
                // RESET status
                iic.i2cWriteSync(iicAdr, 1, Buffer.from([0xF0]));
                iic.closeSync();
                continue;
            }
            */
            //delay(1);
            act = 4;
            iic = i2c.openSync(0);
            // select data register, status is READY = 0x4
            iic.i2cWriteSync(iicAdr, 1, Buffer.from([0x96]));
            //iic.closeSync();
            delay(1);
            //iic = i2c.openSync(0);
            iic.i2cReadSync(iicAdr, 9, rbuf);
            iic.closeSync();
            // pass the data up
            var t = rbuf[0];
            var b = rbuf[1];
            var a = rbuf[2];
            var latch = rbuf[3];
            var press = rbuf[4];
            var d = rbuf[5] << 8;
            d |= (rbuf[6]);
            var seq = rbuf[7];
            var chk = rbuf[8];
            act = 5;
            if (rbuf[7] > 0x7f || rbuf[8] != 0xaa) {
                log ("ACK received = " + rbuf[7].toString(16) + " / chk = " + rbuf[8].toString(16), 'warn');
                continue;
            }
            //delay(1);
            // acknowledge
            iic = i2c.openSync(0);
            iic.i2cWriteSync(iicAdr, 2, Buffer.from([0x78, rbuf[7]]));
            // This ends the lock
            iic.i2cWriteSync(iicAdr, 2, Buffer.from([0xE1, 0xA8]));
            iic.i2cReadSync(iicAdr, 1, rbuf);
            //rbuf[0] = 0;
            iic.closeSync();
            if (verbose > 0) {
                log ("Type " + t + " " + b + "." + a + "." + latch + " data=" + d.toString(16));
            }
            if (chk != 0xaa) {
                log (lastSeq.toString(16) + ": Invalid status: " + chk.toString(16) + " != 0xAA. Retrying", 'warn');
                // log differently in error case
                err++;
                continue;
                //return;
            }
            if (lastSeq == 0xff)
                lastSeq = seq;
            else {
                var targetSeq = lastSeq + 1;
                if (targetSeq == 0x80)
                    targetSeq = 1;
                if (targetSeq < rbuf[7]) {
                    log ("Sequecense missing prev: " + lastSeq.toString(16) + " now: " + seq.toString(16), 'warn');
                }
                if (lastSeq == seq && err == 0) {
                    log ("got this already: " + lastSeq.toString(16), "warn");
                    log ("Type " + t + " " + b + "." + a + "." + latch + " data=" + d.toString(16), "warn");
                };
                if (err) {
                    log (lastSeq.toString(16) + ": Error recoverd");
                    err = 0;
                } else {
                    //log (rbuf[7].toString(16) + ": ACKed");
                }
                lastSeq = seq;
                //log ("ACKed " + lastSeq.toString(16));
            }
            /* blind read
            iic.i2cReadSync(iicAdr, 1, rbuf);
            if (rbuf[0] == 0x80) {
                log ("we are done");
            }
            */
            updateOwState(t, b, a, press, latch, d);
            //act = 6;
            // check alarm status register, 0x40 = event data
        }
        catch (e) {
            rbuf[0] = 0x40;
            log (e + " ... retrying (" + lastSeq.toString(16) + " / " + act + ")", 'warn');
            //log(e.stack);
            delay(50);
            err++;
        }
    }  while (rbuf[0] & 0x40);
    //log ("Event handling done " + rbuf[0])
    // recall to check for alarm state
    setTimeout (events, 50);
}

function toggleState(d, name)
{
    var pre = getState('owfs.0.wires.' + name).val;
    var state = "";

    // not setting, only updating the value!
    if (d != 0) {
        setState('owfs.0.wires.' + name, false, true);
        if (pre != false)
            log(name + " from " + pre + " to off (ack / " + d + ")");
        else
            log(name + " off (ack)");
        if (name == "Wohn_Heizung") {
            log (lastSeq.toString(16) + ": ack = " + getState('owfs.0.wires.' + name).ack);
        }
        if (getState('owfs.0.wires.' + name).val == false)
            return;
        state = " off";
    }
    else {
        setState('owfs.0.wires.' + name, true, true);
        if (pre != true)
            log(name + " from " + pre + " to on (ack / 0)");
        else
            log(name + " on (ack)");
        if (name == "Wohn_Heizung") {
            log (lastSeq.toString(16) + ": ack = " + getState('owfs.0.wires.' + name).ack);
        }
        if (getState('owfs.0.wires.' + name).val == true)
            return;
        state = " on";
    }
    setState('javascript.0.log', name + state);
    log (lastSeq.toString(16) + ": update state " + d + " of " + name + " to" + state, "info");
}

function updateActivity(id)
{
    var s = getState('javascript.0.' + id).val;
    if (s == 0)
        return;
    s--;
    setState('javascript.0.' + id, s);
    setTimeout(function () {
            updateActivity(id);
        }, 3000);
}

function updateOwState(t, b, a, press, latch, d)
{
    if (b == 0 && a == 9 && latch == 3)
        return;
    if (t == 7) {
        log("Arduino restart", "error");
        setState('javascript.0.log', "Arduino Restart!");
        sendTime();
        sendTo("telegram", { text: 'Arduino restart!' });
        return;
    }
    if (t == 2) {
        //log (lastSeq.toString(16) + ": Temperature " + b + "." + a + " data=" + d / 16);
        if ((d / 16) > 5 && (d / 16) < 30) {
            var temp = (d / 16).toFixed(1);
            try {
                var old = getState('javascript.0.devices.temp_' + b + "_" + a).val;
                setState('javascript.0.devices.temp_' + b + "_" + a, parseFloat(temp));
                if (old != parseFloat(temp))
                    log ("Temperature " + b + "." + a + " = " + temp);
            }
            catch (e) {
                log ("Wrong Temperature data received: " + b + "." + a + " = " + temp);
            }
        }
        return;
    }
    if (t == 3) {
        //log ("Light " + b + "." + a + " data=" + d);
        setState('javascript.0.devices.brightness', d);
        return;
    }
    if (t == 4) {
        log ("dimming " + b + "." + a + "." + latch + " data=" + d.toString(16));
        return;
    }
    if (t == 5) {
        // humidity
        setState('javascript.0.devices.hum_' + b + "_" + a, d);
        return;
    }
    if (t == 6) {
        powerImp();
        return;
    }
    var ignorePresense = false;
    if (b == 0) {
        if (a == 9 && latch == 4) {
            // ignore outside PIR for now ... stale
            return;
        }
        if (a == 5 && latch == 1)
            ignorePresense = true;
        if (a == 5 && latch == 7)
            ignorePresense = true;
    }
    if (b == 1) {
        // ignore heating control
        if ((a == 1 && latch == 2) || (a == 1 && latch == 3))
            ignorePresense = true;
        if ((a == 12 && latch == 2) || (a == 12 && latch == 3))
            ignorePresense = true;
    }
    if (ignorePresense == false) {
        if (getState('javascript.0.Vacation').val == 1) {
            // alarm;
            log ("report alarm state");
            sendTo("telegram", "send", { text: 'Alarm!'  });
        }
        setState('javascript.0.presence', 20);
    }
    var ts = t.toString();
    if (t == 1)
        ts = "switch ";
    if (t == 0)
        ts = "latch ";
    log (lastSeq.toString(16) + ": Type " + ts + b + "." + a + "." + latch + " data=" + d.toString(16));
    if (b == 0) {
        if (a == 1 && latch == 0) {
            toggleState(d & 0x01, 'FlurUnten');
        }
        if (a == 3 && latch == 0)
            toggleState(d & 0x01, 'BadUnten1');
        if (a == 3 && latch == 1)
            toggleState(d & 0x02, 'BadUnten2');
        if (a == 6 && latch == 1 || a == 6 && latch == 0) {
            var c;
            toggleState(d & 0x01, 'BuroTisch');
            if (d & 0x01)
                c = "0"
            else
                c = "7"
            sendTo ("mqtt.0", 'sendMessage2Client', {topic: 'cmnd/tasmota_084490/displayText', message: '[x0y120Bi0Ci0R7:7x0y120Ci' + c + 'u5:5:3]'});
        }
        if (a == 5) {
            if (latch == 0) {
                toggleState(d & 0x01, 'Büro_Schrank');
                if (d & 0x01)
                    c = "0"
                else
                    c = "7"
                sendTo ("mqtt.0", 'sendMessage2Client', {topic: 'cmnd/tasmota_084490/displayText', message: '[x8y120Bi0Ci0R7:7x8y120Ci' + c + 'u5:5:3]'});
            }
            if (latch == 1) {
                toggleState(d & 0x02, 'Büro_Heizung');
            }
            if (latch == 7)
                toggleState(((d & 0x80) == 0), 'Büro_Thermo');
        }
        if (a == 8 && latch == 0) {
            toggleState(d & 0x01, 'Buro');
            if (d & 0x01)
                c = "0"
            else
                c = "7"
            sendTo ("mqtt.0", 'sendMessage2Client', {topic: 'cmnd/tasmota_084490/displayText', message: '[x16y120Bi0Ci0R7:7x16y120Ci' + c + 'u5:5:3]'});
        }
        if (a == 9) {
            if (latch == 0) {
                //log("Dimmer UG val = " + d);
                //setState('javascript.0.DimStairUG', d * (100/15), true);
            }
            if (latch == 1) {
                var now = new Date();
                var st = getState ('javascript.0.StairUGActivity');
                var lc = (now.getTime() - st.lc) / 1000;
                var last = Math.round(lc / 60);
                if (last > 5) {
                    setState('javascript.0.log', "Type " + t + ", Src " + b + "." + a + "." + latch + " data=" + d.toString(16));
                }
                setState('javascript.0.StairUGActivity', 5);
                setTimeout(function () {
                        updateActivity('StairUGActivity');
                    }, 3000);
            }
            if (latch == 4) {
                log("Bewegung draußen");
            }
            if (latch == 2) {
                log("Warnung Pumpe?");
            }
        }
    }
    if (b == 1) {
        if (a == 1) {
            if (latch == 1)
                toggleState (d & 0x02, 'Küche_Tisch');
            if (latch == 0)
                toggleState (d & 0x01, 'Terrase_Ost');
            toggleState(d & 0x04, 'Wohn_Heizung');
            toggleState(d & 0x08, 'Wohn_Thermo');
        }
        if (a == 11) {
            if (latch == 0) {
                setState ('javascript.0.EntranceActivity', -5);
                toggleState (d & 0x01, 'Eingang_PIO_0');
            }
            if (latch == 1)
                toggleState (d & 0x02, 'FlurEG_PIO_1');
            if (latch == 6) {
                toggleState (d & 0x20, 'Flur_Tür');
                if (d & 0x20) {
                    setState('javascript.0.log', "Haus Tür offen");
                    if (!isAstroDay() && getState('owfs.0.wires.Eingang_PIO_0').val == false) {
                        setState ('javascript.0.EntranceActivity', -5);
                        // someone opened the door, give him some light
                        setState('owfs.0.wires.Eingang_PIO_0', true);
                        // and switch off again
                        setTimeout(function () {
                            setState ('javascript.0.EntranceActivity', -5);
                            setState('owfs.0.wires.Eingang_PIO_0', false);
                        }, (120 * 1000));
                    }
                } else {
                    setState('javascript.0.log', "Haus Tür zu");
                }
            }
        }
        if (a == 12) {
            if (latch == 1)
                toggleState (d & 0x02, 'Küche_Tresen');
            toggleState (d & 0x08, 'Kueche_Thermo');
            toggleState (d & 0x04, 'Heizung_Kueche');
        }
        if (a == 5 && latch == 1)
            toggleState(d & 0x02, 'TV_Wand');
        if (a == 5 && latch == 2)
            toggleState(d & 0x01, 'TV_Decke');
    }
    if (b == 2) {
        if (a == 1 && latch == 1)
            toggleState(d & 0x01, 'Bad_PIO_0');
        if (a == 2 && latch == 2)
            toggleState(d & 0x02, 'Bad_PIO_1');

        if (a == 7 && latch == 2) {
            var now = new Date();
            var st = getState ('javascript.0.StairOGActivity');
            var lc = (now.getTime() - st.lc) / 1000;
            var last = Math.round(lc / 60);
            if (last > 5) {
                setState('javascript.0.log', "Type " + t + ", Src " + b + "." + a + "." + latch + " data=" + d.toString(16));
            }
            setState('javascript.0.StairOGActivity', 5);
            setTimeout(function () {
                    updateActivity('StairOGActivity');
                }, 3000);

        }
        if (a == 7 && latch == 0) {
            //log("Dimmer OG val = " + d, "info");
            //setState('javascript.0.DimStairOG', d * (100/15), true);
        }
    }
}

gpio.on('change', function(channel, value) {
    if (value == 0) {
        alarm = true;
        events();
    } else {
        alarm = false;
    }
});

on('javascript.0.Vacation', function (obj)
{
    if (obj.state.ack)
        return;
    const iic = i2c.openSync(0);

    if (obj.state.val) {
        sendTo("telegram", { text: 'Vacation!'  }, function (res) {
            console.log('Sent to ' + res + ' users');
        });
        iic.i2cWriteSync(iicAdr, 2, Buffer.from([0x69, 6]));
    } else {
        iic.i2cWriteSync(iicAdr, 2, Buffer.from([0x69, 14]));
        sendTo("telegram", { text: 'Dahoam!'  });
    }
});


on("javascript.0.Umlaufpumpe", function (obj) {
    if (obj.ack)
        return;

    const iic = i2c.openSync(0);
    iic.i2cWriteSync(iicAdr, 5, Buffer.from([0x3, 0, 9, 3, obj.state.val * 100]));
    iic.closeSync();

    if (obj.state.val == 1) {
     setTimeout(function () {
            setState ('javascript.0.Umlaufpumpe', 0);
        }, 3 * 60 * 1000);
    }
});

on("javascript.0.DimStairUG", function (obj) {
    const iic = i2c.openSync(0);
    iic.i2cWriteSync(iicAdr, 5, Buffer.from([0x3, 0, 9, 0, obj.state.val]));
    iic.closeSync();
});

on("javascript.0.DimStairOG", function (obj) {
    const iic = i2c.openSync(0);
    iic.i2cWriteSync(iicAdr, 5, Buffer.from([0x3, 2, 7, 0, obj.state.val]));
    iic.closeSync();
});

on("javascript.0.DimBildEG", function (obj) {
    const iic = i2c.openSync(0);
    iic.i2cWriteSync(iicAdr, 5, Buffer.from([0x3, 1, 3, 0, obj.state.val]));
    iic.closeSync();
});

function switch_pio(obj)
{
    var s;
    const rbuf = Buffer.alloc(1);
    var s;
    // split each number
    var ar = obj.native.id.split('');
    // one digit for the bus
    var bus = parseInt(ar[6], 16)
    s = ar[3] + "" + ar[4];
    var id = parseInt(s, 16)
    s = obj.native.property;
    var ar = s.split('.');
    var pio = parseInt(ar[1]);
    var cnt = 100;
    var err = 0;
    var retry = 50;

    s = " on ";
    if (obj.state.val == false)
        s = " off ";
    log ("switching " +  obj.name + s + "(ID=" + obj.native.id + ", Bus=" + bus + " Adr=" + id + " PIO=" + pio + ")");

    while (retry--) {
        try {
            const iic = i2c.openSync(0);
            iic.i2cWriteSync(iicAdr, 2, Buffer.from([0xE1, 0xE1]));
            delay(err + 1);
            do {
                iic.i2cReadSync(iicAdr, 1, rbuf);
                delay(err + 1);
            } while (cnt-- > 0 && rbuf[0] != 0);
            if (cnt == 0 && rbuf[0] != 0) {
                iic.closeSync();
                log ("Write issue " + rbuf[0] + ", retrying ...");
                err++;
                continue;
            }

            if (obj.state.val == false) {
                iic.i2cWriteSync(iicAdr, 5, Buffer.from([0x3, bus, id, pio, 0]));
            } else {
                iic.i2cWriteSync(iicAdr, 5, Buffer.from([0x3, bus, id, pio, 100]));
            }
            if (true) {
                iic.closeSync();
                return;
            }
        }
        catch (e) {
            //log (e + " ... retrying");
            delay(err + 50);
            err++;
        }
    } /* while retry */
    log ("Write issues: " + err + ", gave up :-(", 'warn');
}

function retry_switch(obj, retry) {
    if (retry-- == 0) {
        log('failed to swtich ' + obj.name, 'error');
        // not switched, so state reverted
        //setState('owfs.0.wires.' + obj.name, !obj.val, true);
        return;
    }
    var ack = getState('owfs.0.wires.' + obj.name).ack;
    if (ack == false) {
        log ("ack retry " + retry);
        switch_pio(obj);
        setTimeout(retry_switch, 250, obj, retry);
    } else {
        if (obj.name == "Büro_Heizung" || obj.name == "Wohn_Heizung" || obj.name == "Heizung_Kueche") {
            var s = " on ";
            if (obj.state.val == false)
                s = " off ";
            sendTo("telegram", "send", { text: obj.name + ' = ' + s});
            setState('javascript.0.log', obj.name + ' = ' + s);
        }
    }
}

on(/^owfs\.0\.wires\.*/, function (obj) {
    /*
    if (obj.from != 'system.adapter.web.0') {
        log ("wire change from " + obj.from + "ack=" + obj.state.ack + "val=" + obj.state.val);

        return;
    }
    */
    if (obj.state.ack) {
        // remove from ack list?
        return;
    } if (alarm == true) {
        events();
    }
    switch_pio(obj);
    setTimeout(retry_switch, 200, obj, 1);
});

on("javascript.0.SwtichLevel", function (obj) {
    var dimLevel = getState("javascript.0.DimLevel").val;
    log ("SwitchLevel changed to " + obj.state.val)
    const iic = i2c.openSync(0);
    iic.i2cWriteSync(iicAdr, 3, Buffer.from([0x6A, obj.state.val, dimLevel]));
    iic.closeSync();
});

schedule({hour: [13], minute: [45]}, function (obj) {
    setState ('javascript.0.Umlaufpumpe', 0);
});
schedule({hour: [20], minute: [0]}, function (obj) {
    setState ('javascript.0.Umlaufpumpe', 0);
});

schedule({hour: [4,8,10,12,14,16,18,22], minute: [0]}, function () {
    events();
    sendTime();
});

schedule({astro: "sunset", shift: -15}, function () {
    sendTime();
    log("Sunset");
});

schedule({astro: "sunrise", shift: -15}, function () {
    sendTime();
    log("Sunrise");
});
