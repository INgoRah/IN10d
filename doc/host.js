const i2c = require('i2c-bus');
var gpio = require('rpi-gpio');

var iicAdr = 0x2f;
var lastSeq = 0xff;
var alarm = true;
var ev_poll_tmr;

try {
    gpio.reset();
    //gpio.setMode(gpio.MODE_RPI);
    gpio.setup(22, gpio.DIR_IN, gpio.EDGE_FALLING);
    // Pin #7 = GPIO6 using git co feature/add-orange-pi-zero on rpi-gpio
    gpio.setup(7, gpio.DIR_IN, gpio.EDGE_BOTH, function (err) {
        if (err) {
            log("GPIO setup err = " + err);
            log("Run iobroker as root");
            // change user to root in
            // /etc/systemd/system/multi-user.target.wants/iobroker.service
        }
    });
}
catch (e) {
    log (e, 'error');
}

alarm = true;
events();
alarm = false;
setTimeout (events, 50);

function sendTime()
{
    const rbuf = Buffer.alloc(1);
    const iic = i2c.openSync(0);
    var d = new Date();
    iic.i2cWriteSync(iicAdr, 4, Buffer.from([0x40, +d.getHours() , +d.getMinutes(), +isAstroDay()]));
    iic.i2cReadSync(iicAdr, 1, rbuf);
    iic.closeSync();
}


function events() {
    const rbuf = Buffer.alloc(10);
    var cnt = 50;
    var err = 0;
    var act = 0;
    var verbose = getState('javascript.0.verbose').val;

    clearTimeout(ev_poll_tmr);
    ev_poll_tmr = setTimeout(function () {
        log("Forced event poll (missing events)", "warn");
        events();
    }, 5 * 60 * 1000);
    do {
        var iic = i2c.openSync(0);

        if (err)
            log ("Error case @" + act + "...");
        try {
            //if (alarm == false)
            //    return;
            // check the status first?
            // check alarm status which resets the gpio
            // and aquires the SW lock
            iic.i2cWriteSync(iicAdr, 2, Buffer.from([0xE2, 0xA8]));
            act = 1;
            delay(1);
            iic.i2cReadSync(iicAdr, 1, rbuf);
            act = 2;
            // 0x40 = event data
            if ((rbuf[0]) == 0xff) {
                // error, retry
                // should have released the lock
                iic.i2cWriteSync(iicAdr, 1, Buffer.from([0xEF]));
                iic.closeSync();
                delay(10);
                continue;
            }
            if ((rbuf[0] & 0x40) == 0x0) {
                // should have released the lock
                iic.i2cWriteSync(iicAdr, 1, Buffer.from([0xEF]));
                iic.closeSync();
                if (alarm)
                    setTimeout (events, 50);
                return;
            }
            act = 3;
            /* request event data, selects status register.
               The controller copies the data over from the fifo */
            iic.i2cWriteSync(iicAdr, 1, Buffer.from([0x1]));
            do {
                delay(1);
                /* read status
                   0 = ok, data is prepared
                   1 = busy/waiting to read data
                   0x02 = processing and should not occur
                   0x20 = SW lock aquired
                   0x40 = event data
                   0x80 no data */
                act = 4;
                iic.i2cReadSync(iicAdr, 1, rbuf);
                if ((rbuf[0]) == 0xff) {
                    continue;
                }
                if ((rbuf[0] & 0x80) == 0x80) {
                    log ("no data / handled before? " + rbuf[0].toString(16));
                    // release the lock
                    iic.i2cWriteSync(iicAdr, 1, Buffer.from([0xEF]));
                    iic.closeSync();
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
            act = 5;
            // select data register, status is READY = 0x4
            iic.i2cWriteSync(iicAdr, 1, Buffer.from([0x96]));
            //iic.closeSync();
            //delay(1);
            //iic = i2c.openSync(0);
            iic.i2cReadSync(iicAdr, 10, rbuf);
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
            var stat = rbuf[9];
            act = 6;
            if (chk != 0xaa) {
                log ("seq received = " + seq.toString(16) + " / chk = " + rbuf[8].toString(16), 'warn');
                err++;
                continue;
            }
            // acknowledge
            // This ends the lock
            iic.i2cWriteSync(iicAdr, 2, Buffer.from([0x78, seq]));
            //iic.i2cWriteSync(iicAdr, 2, Buffer.from([0xE1, 0xA8]));
            //iic.i2cReadSync(iicAdr, 1, rbuf);
            //rbuf[0] = 0;
            iic.closeSync();
            if (verbose > 0) {
                log ("Type " + t + " " + b + "." + a + "." + latch + " data=" + d.toString(16));
            }
            if (lastSeq == 0xff)
                lastSeq = seq;
            else {
                var targetSeq = lastSeq + 1;
                if (targetSeq == 0x80)
                    targetSeq = 1;
                if (targetSeq < seq) {
                    log ("Sequecense missing prev: " + lastSeq.toString(16) + " now: " + seq.toString(16), 'warn');
                }
                if (lastSeq == seq && err == 0) {
                    log ("got this already: " + lastSeq.toString(16) + ", stat=" + stat.toString(16), "warn");
                    log ("Type " + t + " " + b + "." + a + "." + latch + " data=" + d.toString(16), "warn");
                };
                if (err) {
                    log (lastSeq.toString(16) + ": Error recoverd");
                    err = 0;
                }
                lastSeq = seq;
            }
            updateOwState(t, b, a, press, latch, d);
            if (stat & 0x40) {
                // let it ack for the next loop
                delay(10);
            }
            //act = 6;
        }
        catch (e) {
            iic.closeSync();
            stat = 0x40;
            log (e + " ... retrying (" + lastSeq.toString(16) + " / " + act + ")", 'warn');
            //log(e.stack);
            delay(200);
            err++;
        }
    }  while (stat & 0x40);
    //log ("Event handling done " + stat)
    // recall to check for alarm state
    setTimeout (events, 100);
}

gpio.on('change', function(channel, value) {
    if (channel == 22) {
        if (value == 0) {
            setState("javascript.0.klingel.active", 1);
            return;
        }
    }
    if (value == 0) {
        alarm = true;
        events();
    } else {
        alarm = false;
    }
});

on("javascript.0.DimStairUG", function (obj) {
    const iic = i2c.openSync(0);
    iic.i2cWriteSync(iicAdr, 5, Buffer.from([0x3, 0, 2, 0, obj.state.val]));
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
    var state_name = obj.name.replace(/\./g, '_');
    var ack = getState('owfs.0.wires.' + state_name).ack;
    if (retry-- == 0) {
        log('failed to swtich ' + obj.name + " (ack=" + ack + ")", 'error');
        // not switched, so state reverted
        //setState('owfs.0.wires.' + obj.name, !obj.val, true);
        return;
    }
    if (ack == false) {
        log ("ack retry " + retry);
        switch_pio(obj);
        setTimeout(retry_switch, 250, obj, retry);
    } else {
        if (obj.name == "Büro_Heizung" || obj.name == "Wohn_Heizung" || obj.name == "Heizung_Kueche") {
            var s = " on ";
            if (obj.state.val == false)
                s = " off ";
            //sendTo("telegram", "send", { text: obj.name + ' = ' + s});
            setState('javascript.0.log', obj.name + ' = ' + s);
        }
    }
}

on("javascript.0.SwtichLevel", function (obj) {
    var dimLevel = getState("javascript.0.DimLevel").val;
    log ("SwitchLevel changed to " + obj.state.val)
    const iic = i2c.openSync(0);
    iic.i2cWriteSync(iicAdr, 3, Buffer.from([0x6A, obj.state.val, dimLevel]));
    iic.closeSync();
});
