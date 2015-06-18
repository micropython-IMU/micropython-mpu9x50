'''
Demo of MPU9150 nonblocking reads and the reading and use of correction factors.
Shows that call to get_mag() returns fast.
'''

from mpu9150 import MPU9150
import pyb

def testfunc(a):
    start = pyb.micros()
    while not a.mag_ready:
        pass
    dt = pyb.elapsed_micros(start)
    print("Wait time = {:5.2f}mS".format(dt/1000))
    start = pyb.micros()
    xyz = a.mag.xyz
    dt = pyb.elapsed_micros(start)
    print("Time to get = {:5.2f}mS".format(dt/1000))
    print("x = {:5.3f} y = {:5.3f} z = {:5.3f}".format(xyz[0], xyz[1], xyz[2]))
    print("Mag status should be not ready (False): ", a.mag_ready)
    print("Correction factors: x = {:5.3f} y = {:5.3f} z = {:5.3f}".format(
        a.mag_correction[0],
        a.mag_correction[1],
        a.mag_correction[2]))

def test():
    mpu9150 = MPU9150('X')
    testfunc(mpu9150)
    print()
    pyb.delay(250)
    print("Repeating")
    testfunc(mpu9150)

test()

