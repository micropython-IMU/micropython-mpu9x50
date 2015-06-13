# mpu9250.py MicroPython driver for the InvenSense MPU9250 inertial measurement unit
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
import os
from math import sqrt

X_AXIS = const(0)
Y_AXIS = const(1)
Z_AXIS = const(2)

class MPU9250Exception(Exception):
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

def default_wait():
    pyb.delay(50)

class Vector3d(object):
    '''
    Represents a vector in a 3D space using Cartesian coordinates.
    Internally uses sensor relative coordinates.
    Returns vehicle-relative x, y and z values.
    '''
    def __init__(self, transposition, scaling, update_function):
        self._vector = [0,0,0]
        self._ivector = [0,0,0]
        self.cal = (0,0,0)
        self.argcheck(transposition, "Transposition")
        self.argcheck(scaling, "Scaling")
        if (len(transposition) != len(set(transposition))) or min(transposition) < 0 or max(transposition) > 2:
            raise ValueError('Transpose indices must be unique and in range 0-2')
        self._scale = scaling
        self._transpose = transposition
        self.update = update_function

    def argcheck(self, arg, name):
        if len(arg) != 3 or not (type(arg) is list or type(arg) is tuple):
            raise ValueError(name + ' must be a 3 element list or tuple')

    def _set(self, axis, val):                  # Private setter: x,y,z,xyz are read-only
        self._vector[axis] = val

    def calibrate(self, stopfunc, waitfunc = default_wait):
        self.update()
        maxvec = self._vector[:]                # Initialise max and min lists with current values
        minvec =self._vector[:]
        while not stopfunc():
            waitfunc()
            self.update()
            maxvec = list(map(max, maxvec, self._vector))
            minvec = list(map(min, minvec, self._vector))
        self.cal = tuple(map(lambda a, b: (a +b)/2, maxvec, minvec))

    @property
    def _calvector(self):                       # Vector adjusted for calibration offsets
        return list(map(lambda val, offset : val - offset, self._vector, self.cal))

    @property
    def x(self):                                # Corrected, vehicle relative floating point values
        self.update()
        return self._calvector[self._transpose[0]] * self._scale[0]

    @property
    def y(self):
        self.update()
        return self._calvector[self._transpose[1]] * self._scale[1]

    @property
    def z(self):
        self.update()
        return self._calvector[self._transpose[2]] * self._scale[2]

    @property
    def xyz(self):
        self.update()
        return (self._calvector[self._transpose[0]] * self._scale[0],
                self._calvector[self._transpose[1]] * self._scale[1],
                self._calvector[self._transpose[2]] * self._scale[2])
    @property
    def magnitude(self):
        self.update()
        return sqrt(self.x**2 + self.y**2 + self.z**2)

    # Raw uncorrected integer values from sensor
    @property
    def ix(self):
        return self._ivector[0]

    @property
    def iy(self):
        return self._ivector[1]

    @property
    def iz(self):
        return self._ivector[2]

    @property
    def ixyz(self):
        return self._ivector
    @property
    def transpose(self):
        return tuple(self._transpose)
    @property
    def scale(self):
        return tuple(self._scale)

# MPU9250 constructor arguments
# 1.    side_str 'X' or 'Y' depending on the Pyboard I2C interface being used
# 2.    optional device_addr 0, 1 depending on the voltage applied to pin AD0 (Drotek default is 1)
#       if None driver will scan for a device (if one device only is on bus)
# 3, 4. transposition, scaling optional 3-tuples allowing for outputs to be based on vehicle
#       coordinates rather than those of the sensor itself. See readme.

class MPU9250():
    '''
    Module for the MPU9250 9DOF IMU.
    '''
    _mpu_addr = (104, 105)  # addresses of MPU9250 determined by voltage on pin AD0 
    _mag_addr = 12          # Magnetometer address.
    _chip_id = 113
    _I2Cerror = "I2C communication failure"
    def __init__(self, side_str, device_addr = None, transposition = (0,1,2), scaling = (1,1,1)):
        self._mag = Vector3d(transposition, scaling, self._mag_callback)
        self._accel = Vector3d(transposition, scaling, self._accel_callback)
        self._gyro = Vector3d(transposition, scaling, self._gyro_callback)
        self.buf1 = bytearray([0]*1)            # Pre-allocated buffers for reads: allows reads to
        self.buf2 = bytearray([0]*2)            # be done in interrupt handlers
        self.buf3 = bytearray([0]*3)
        self.buf6 = bytearray([0]*6)
        self._mag_stale_count = 0               # MPU9250 count of consecutive reads where old data was returned
        self.timeout = 10                       # I2C tieout mS

        tim = pyb.millis()                      # Ensure PSU and device have settled
        if tim < 200:
            pyb.delay(200-tim)

        try:                                    # Initialise I2C
            side = {'X':1, 'Y':2}[side_str.upper()]
        except KeyError:
            raise MPU9250Exception('I2C side must be X or Y')
        self._mpu_i2c = pyb.I2C(side, pyb.I2C.MASTER)
        if device_addr is None:
            devices = set(self._mpu_i2c.scan())
            mpus = devices.intersection(set(MPU9250._mpu_addr))
            number_of_mpus = len(mpus)
            if number_of_mpus == 0:
                raise MPU9250Exception("No MPU9250's detected")
            elif number_of_mpus == 1:
                self.mpu_addr = mpus.pop()
            else:
                raise MPU9250Exception("Two MPU9250's detected: must specify a device address")
        else:
            if not device_addr in (0,1):
                raise MPU9250Exception('Device address must be 0 or 1')
            self.mpu_addr = self._mpu_addr[device_addr]

        self._read(self.buf1, 0x75, self.mpu_addr)
        self.chip_id = int(self.buf1[0])
        if self.chip_id != MPU9250._chip_id:
            raise MPU9250Exception('MPU9250 communication failure')
        # Can communicate with chip. Set it up.
        self.wake()                             # wake it up
        self.passthrough = True                 # Enable mag access from main I2C bus
        self.mag_correction = self._magsetup()  # 16 bit, 100Hz update.Return correction factors.
        self.accel_range = 0                    # default to highest sensitivity
        self.accel_filter_range = 0             # fast filtered response
        self.gyro_range = 0                     # Likewise for gyro
        self.gyro_filter_range = 0

    @property
    def sensors(self):
        return self._accel, self._gyro, self._mag

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
            print(MPU9250._I2Cerror)
        return 'awake'

    # mode
    def sleep(self):
        '''
        Sets the device to sleep mode.
        '''
        try:
            self._write(0x40, 0x6B, self.mpu_addr)
        except OSError:
            print(MPU9250._I2Cerror)
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
            print(MPU9250._I2Cerror)

    @passthrough.setter
    def passthrough(self, mode):
        if type(mode) is bool:
            val = 2 if mode else 0
            try:
                self._write(val, 0x37, self.mpu_addr) # I think this is right.
                self._write(0x00, 0x6A, self.mpu_addr)
            except OSError:
                print(MPU9250._I2Cerror)
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
            print(MPU9250._I2Cerror)

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
            print(MPU9250._I2Cerror)

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
            print(MPU9250._I2Cerror)
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
            print(MPU9250._I2Cerror)

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
            print(MPU9250._I2Cerror)
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
            print(MPU9250._I2Cerror)

    # Low pass filters
    @property
    def gyro_filter_range(self):
        '''
        Returns the gyro and temperature sensor low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6   7
        Cutoff (Hz):        250 184 92  41  20  10  5   3600
        Sample rate (KHz):  8   1   1   1   1   1   1   8
        '''
        try:
            self._read(self.buf1, 0x1A, self.mpu_addr)
            res = self.buf1[0] & 7
        except OSError:
            print(MPU9250._I2Cerror)
            res = None
        return res

    @gyro_filter_range.setter
    def gyro_filter_range(self, filt):
        # set range
        try:
            if (filt >= 0) and (filt < 8):
                self._write(filt, 0x1A, self.mpu_addr)
            else:
                print('Filter coefficient must be between 0 and 7')
        except OSError:
           print(MPU9250._I2Cerror)

    @property
    def accel_filter_range(self):
        '''
        Returns the accel low pass filter cutoff frequency
        Pass:               0   1   2   3   4   5   6   7 BEWARE 7 doesn't work on device I tried.
        Cutoff (Hz):        460 184 92  41  20  10  5   460
        Sample rate (KHz):  1   1   1   1   1   1   1   1
        '''
        try:
            self._read(self.buf1, 0x1D, self.mpu_addr)
            res = self.buf1[0] & 7
        except OSError:
            print(MPU9250._I2Cerror)
            res = None
        return res

    @accel_filter_range.setter
    def accel_filter_range(self, filt):
        # set range
        try:
            if (filt >= 0) and (filt < 8):      # Sets ACCEL_FCHOICE = 1 enabling fiter
                self._write(filt, 0x1D, self.mpu_addr)
            else:
                print('Filter coefficient must be between 0 and 7')
        except OSError:
           print(MPU9250._I2Cerror)
    # get temperature
    @property
    def temperature(self):
        '''
        Returns the temperature in degree C.
        '''
        try:
            self._read(self.buf2, 0x41, self.mpu_addr)
        except OSError:
            print(MPU9250._I2Cerror)
            return 0
        return bytes_toint(self.buf2[0], self.buf2[1])/333.87 + 21 # I think

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

    # Magnetometer initialisation: use 16 bit continuous mode. Mode 1 is 8Hz mode 2 is 100Hz repetition
    # returns correction values
    def _magsetup(self):                        # mode 2 100Hz continuous reads, 16 bit
        try:
            self._write(0x0F, 0x0A, self._mag_addr)     # fuse ROM access mode
            self._read(self.buf3, 0x10, self._mag_addr) # Correction values
            self._write(0, 0x0A, self._mag_addr)        # Power down mode (AK8963 manual 6.4.6)
            self._write(0x16, 0x0A, self._mag_addr)     # 16 bit (0.15uT/LSB not 0.015), mode 2
        except OSError:
            print(MPU9250._I2Cerror)
        x = (0.5*(self.buf3[0] -128))/128 + 1
        y = (0.5*(self.buf3[1] -128))/128 + 1
        z = (0.5*(self.buf3[2] -128))/128 + 1
        return (x, y, z)

    @property
    def mag(self):
        return self._mag

    def _mag_callback(self):
        '''
        Update magnetometer Vector3d object (if data available)
        '''
        try:                                    # If read fails, returns last valid data and
            self._read(self.buf1, 0x02, self._mag_addr) # increments mag_stale_count
            if self.buf1[0] & 1 == 0:
                return self._mag                # Data not ready: return last value
            self._read(self.buf6, 0x03, self._mag_addr)
            self._read(self.buf1, 0x09, self._mag_addr)
        except OSError:
            self._mag_stale_count += 1
            return self._mag
        if self.buf1[0] & 0x08 > 0:             # An overflow has occurred
            self._mag_stale_count +=1
            return self._mag
        self._mag._ivector[1] = bytes_toint(self.buf6[1], self.buf6[0])  # Note axis twiddling and little endian
        self._mag._ivector[0] = bytes_toint(self.buf6[3], self.buf6[2])
        self._mag._ivector[2] = -bytes_toint(self.buf6[5], self.buf6[4])
        scale = 6.6666                          # scale is (1/0.15uT/LSB)
        self._mag._set(X_AXIS, self._mag._ivector[0]*self.mag_correction[0]/scale)
        self._mag._set(Y_AXIS, self._mag._ivector[1]*self.mag_correction[1]/scale)
        self._mag._set(Z_AXIS, self._mag._ivector[2]*self.mag_correction[2]/scale)
        self._mag_stale_count = 0

    @property
    def mag_stale_count(self):                  # Number of consecutive times where old data was returned
        return self._mag_stale_count

    def get_mag_irq(self): # Uncorrected values because floating point uses heap
        self._read(self.buf1, 0x02, self._mag_addr)
        if self.buf1[0] == 1:                   # Data is ready
            self._read(self.buf6, 0x03, self._mag_addr)
            self._read(self.buf1, 0x09, self._mag_addr)    # Mandatory status2 read
            if self.buf1[0] & 0x08 == 0:        # No overflow has occurred
                self._mag._ivector[1] = bytes_toint(self.buf6[1], self.buf6[0])
                self._mag._ivector[0] = bytes_toint(self.buf6[3], self.buf6[2])
                self._mag._ivector[2] = -bytes_toint(self.buf6[5], self.buf6[4])
