# imu.py MicroPython driver for the InvenSense inertial measurement units
# This is the base class
# Adapted from Sebastian Plamauer's MPU9150 driver:
# https://github.com/micropython-IMU/micropython-mpu9150.git
# Authors Peter Hinch, Sebastian Plamauer
# V0.1 13th June 2015 Experimental: this code is not yet fully tested

'''
mpu9250 is a micropython module for the InvenSense MPU9250 sensor.
It measures acceleration, turn rate and the magnetic field in three axis.
mpu9150 driver modified for the MPU9250 by Peter Hinch

The MIT License (MIT)
Copyright (c) 2014 Sebastian Plamauer, oeplse@gmail.com, Peter Hinch
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''

# User access is now by properties e.g.
# myimu = MPU9250('X')
# magx = myimu.mag.x
# accelxyz = myimu.accel.xyz
# Error handling: on code used for initialisation, abort with message
# At runtime try to continue returning last good data value. We don't want aircraft
# crashing. However if the I2C has crashed we're probably stuffed.

import pyb
from vector3d import Vector3d

X_AXIS = const(0)
Y_AXIS = const(1)
Z_AXIS = const(2)

class MPUException(Exception):
    pass

def bytes_toint(msb, lsb):
    '''
    Convert two bytes to signed integer (big endian)
    for little endian reverse msb, lsb arguments
    Can be used in an interrupt handler
    '''
    if not msb & 0x80:
        return msb << 8 | lsb # +ve
    return -(((msb ^ 255) << 8)| (lsb ^ 255) +1)

class InvenSenseMPU(object):
    '''
    Module for InvenSense 9DOF IMUs. Base class implements features common to MPU9150 and MPU9250.
    '''
    _I2Cerror = "I2C communication failure"
    def __init__(self, side_str, device_addr, transposition, scaling):
        self._accel = Vector3d(transposition, scaling, self._accel_callback)
        self._gyro = Vector3d(transposition, scaling, self._gyro_callback)
        self.buf1 = bytearray([0]*1)            # Pre-allocated buffers for reads: allows reads to
        self.buf2 = bytearray([0]*2)            # be done in interrupt handlers
        self.buf3 = bytearray([0]*3)
        self.buf6 = bytearray([0]*6)
        self.timeout = 10                       # I2C tieout mS

        tim = pyb.millis()                      # Ensure PSU and device have settled
        if tim < 200:
            pyb.delay(200-tim)

        try:                                    # Initialise I2C
            side = {'X':1, 'Y':2}[side_str.upper()]
        except KeyError:
            raise ValueError('I2C side must be X or Y')
        self._mpu_i2c = pyb.I2C(side, pyb.I2C.MASTER)
        if device_addr is None:
            devices = set(self._mpu_i2c.scan())
            mpus = devices.intersection(set(self._mpu_addr))
            number_of_mpus = len(mpus)
            if number_of_mpus == 0:
                raise MPUException("No MPU's detected")
            elif number_of_mpus == 1:
                self.mpu_addr = mpus.pop()
            else:
                raise ValueError("Two MPU's detected: must specify a device address")
        else:
            if not device_addr in (0,1):
                raise ValueError('Device address must be 0 or 1')
            self.mpu_addr = self._mpu_addr[device_addr]

        self._read(self.buf1, 0x75, self.mpu_addr)
        self.chip_id = int(self.buf1[0])
        if self.chip_id != self._chip_id:
            raise MPUException('MPU communication failure')
        # Can communicate with chip. Set it up.
        self.wake()                             # wake it up
        self.passthrough = True                 # Enable mag access from main I2C bus
        self.accel_range = 0                    # default to highest sensitivity
        self.gyro_range = 0                     # Likewise for gyro

    def _read(self, buf, memaddr, addr):        # addr = I2C device address, memaddr = memory location within the I2C device
        '''
        Read bytes to pre-allocated buffer Caller traps OSError.
        '''
        self._mpu_i2c.mem_read(buf, addr, memaddr, timeout=self.timeout)

    # write to device
    def _write(self, data, memaddr, addr):
        '''
        Perform a memory write. Caller should trap OSError.
        '''
        self._mpu_i2c.mem_write(data, addr, memaddr, timeout=self.timeout)

    # wake
    def wake(self):
        '''
        Wakes the device.
        '''
        try:
            self._write(0x01, 0x6B, self.mpu_addr) # Use best clock source
        except OSError:
            print(self._I2Cerror)
        return 'awake'

    # mode
    def sleep(self):
        '''
        Sets the device to sleep mode.
        '''
        try:
            self._write(0x40, 0x6B, self.mpu_addr)
        except OSError:
            print(self._I2Cerror)
        return 'asleep'

    # passthrough
    @property
    def passthrough(self):
        '''
        Returns passthrough mode True or False
        '''
        try:
            self._read(self.buf1, 0x37, self.mpu_addr)
            return self.buf1[0] & 0x02 > 0
        except OSError:
            print(self._I2Cerror)

    @passthrough.setter
    def passthrough(self, mode):
        if type(mode) is bool:
            val = 2 if mode else 0
            try:
                self._write(val, 0x37, self.mpu_addr) # I think this is right.
                self._write(0x00, 0x6A, self.mpu_addr)
            except OSError:
                print(self._I2Cerror)
        else:
            print('pass either True or False')

    # sample rate. Not sure why you'd ever want to reduce this from the default.
    @property
    def sample_rate(self):
        '''
        Get sample rate as per Register Map document section 4.4
        SAMPLE_RATE= Internal_Sample_Rate / (1 + rate)
        default rate is zero i.e. sample at internal rate.
        '''
        try:
            self._read(self.buf1, 0x19, self.mpu_addr)
            return self.buf1[0]
        except OSError:
            print(self._I2Cerror)

    @sample_rate.setter
    def sample_rate(self, rate):
        '''
        Set sample rate as per Register Map document section 4.4
        '''
        if rate < 0 or rate >255:
            raise ValueError("Rate must be in range 0-255")
        try:
            self._write(rate, 0x19, self.mpu_addr)
        except OSError:
            print(self._I2Cerror)

    # accelerometer range
    @property
    def accel_range(self):
        '''
        Accelerometer range
        Value:              0   1   2   3
        for range +/-:      2   4   8   16  g 
        '''
        try:
            self._read(self.buf1, 0x1C, self.mpu_addr)
            ari = self.buf1[0]//8
            self._ar = ari # if read succeeded
        except OSError:
            print(self._I2Cerror)
            ari = None
        return ari

    @accel_range.setter
    def accel_range(self, accel_range):
        ar = (0x00, 0x08, 0x10, 0x18)
        try:
            self._write(ar[accel_range], 0x1C, self.mpu_addr)
            self._ar = accel_range # if write succeeded
        except IndexError:
            print('accel_range can only be 0, 1, 2 or 3')
        except OSError:
            print(self._I2Cerror)

    # gyroscope range
    @property
    def gyro_range(self):
        '''
        Gyroscope range.
        Pass:               0   1   2    3
        for range +/-:      250 500 1000 2000  degrees/second
        '''
        # set range
        try:
            self._read(self.buf1, 0x1B, self.mpu_addr)
            gri = self.buf1[0]//8
            self._gr = gri # if read succeeded
        except OSError:
            gri = None
            print(self._I2Cerror)
        return gri

    @gyro_range.setter
    def gyro_range(self, gyro_range):
        gr = (0x00, 0x08, 0x10, 0x18)
        try:
            self._write(gr[gyro_range], 0x1B, self.mpu_addr) # Sets fchoice = b11 which enables filter
            self._gr = gyro_range # if write succeeded
        except IndexError:
            print('gyro_range can only be 0, 1, 2 or 3')
        except OSError:
            print(self._I2Cerror)

    # Accelerometer
    @property
    def accel(self):
        return self._accel

    def _accel_callback(self):
        '''
        Update accelerometer Vector3d object
        '''
        try:
            self._read(self.buf6, 0x3B, self.mpu_addr)
        except OSError:
            return self._accel # Last value
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (16384, 8192, 4096, 2048)
        self._accel._set(X_AXIS, self._accel._ivector[0]/scale[self._ar])
        self._accel._set(Y_AXIS, self._accel._ivector[1]/scale[self._ar])
        self._accel._set(Z_AXIS, self._accel._ivector[2]/scale[self._ar])

    def get_accel_irq(self):
        '''
        For use in interrupt handlers. Sets self._accel._ivector[] to signed
        unscaled integer accelerometer values
        '''
        self._read(self.buf6, 0x3B, self.mpu_addr)
        self._accel._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._accel._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._accel._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])


    # Gyro
    @property
    def gyro(self):
        return self._gyro

    def _gyro_callback(self):
        '''
        Update gyroscope Vector3d object
        '''
        try:
            self._read(self.buf6, 0x43, self.mpu_addr)
        except OSError:
            return self._gyro # Last value
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
        scale = (131, 65.5, 32.8, 16.4)
        self._gyro._set(X_AXIS, self._gyro._ivector[0]/scale[self._gr])
        self._gyro._set(Y_AXIS, self._gyro._ivector[1]/scale[self._gr])
        self._gyro._set(Z_AXIS, self._gyro._ivector[2]/scale[self._gr])

    def get_gyro_irq(self):
        '''
        For use in interrupt handlers. Sets self._gyro._ivector[] to signed
        unscaled integer gyro values.
        '''
        self._read(self.buf6, 0x43, self.mpu_addr)
        self._gyro._ivector[0] = bytes_toint(self.buf6[0], self.buf6[1])
        self._gyro._ivector[1] = bytes_toint(self.buf6[2], self.buf6[3])
        self._gyro._ivector[2] = bytes_toint(self.buf6[4], self.buf6[5])
