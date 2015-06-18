# Example 2 of nonblocking magnetometer reads, demonstrating delegation of polling to a scheduler
# Expects an MPU9150 on X side and a 24*2 LCD with Hitachi controller wired as per PINLIST

import pyb
from usched import Sched, wait, Poller
from lcdthread import LCD, PINLIST                          # Library supporting Hitachi LCD module
from mpu9150 import MPU9150

def pollfunc(mpu):
    return 1 if mpu.mag_ready else None

def lcd_thread(mylcd,mpu9150):
    k = mpu9150.mag_correction
    mylcd[1] = "x {:5.3f} y {:5.3f} z {:5.3f}".format(k[0],k[1],k[2])
    while True:
        reason = (yield Poller(pollfunc, (mpu9150,)))       # Scheduler returns when pollfunc
        if reason[1] == 1:                                  # returns something other than None. 1 indicates ready.
            xyz = mpu9150.mag.xyz
            mylcd[0] = "x {:5.1f} y {:5.1f} z {:5.1f}".format(xyz[0], xyz[1], xyz[2])
        elif reason[1] == 2:
            mylcd[0] = "Mag read failure"
        yield from wait(0.5)

objSched = Sched()
lcd0 = LCD(PINLIST, objSched, cols = 24)
mpu9150 = MPU9150('X')
objSched.add_thread(lcd_thread(lcd0, mpu9150))
objSched.run()


