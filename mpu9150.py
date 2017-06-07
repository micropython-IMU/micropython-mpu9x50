'''
mpu9150 is a micropython module for the InvenSense MPU9150 sensor.
It measures acceleration, turn rate and the magnetic field in three axis.

The MIT License (MIT)
Copyright (c) 2014 Sebastian Plamauer oeplse@gmail.com, Peter Hinch
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
# 17th May 2017 utime replaces pyb
# 15th June 2015 Now uses subclass of InvenSenseMPU

from imu import MPU6050, bytes_toint, MPUException
from vector3d import Vector3d
from utime import sleep_ms

def default_mag_wait():
    '''
    delay of 1ms
    '''
    sleep_ms(1)



class MPU9150(MPU6050):
    '''
    Module for the MPU9150 9DOF IMU. Pass X or Y according to on which side the
    sensor is connected. Pass 1 for the first, 2 for the second connected sensor.
    By default interrupts are disabled while reading or writing to the device. This
    prevents occasional bus lockups in the presence of pin interrupts, at the cost
    of disabling interrupts for about 250uS.
    MPU9150 constructor arguments
    1. side_str 'X' or 'Y' depending on the Pyboard I2C interface being used
    2. optional device_addr 0, 1 depending on the voltage applied to pin AD0 (Drotek default is 1)
       if None driver will scan for a device (if one device only is on bus)
    3, 4. transposition, scaling optional 3-tuples allowing for outputs to be based on vehicle
          coordinates rather than those of the sensor itself. See readme.
    '''

    _mag_addr = 12

    def __init__(self, side_str, device_addr=None, transposition=(0, 1, 2), scaling=(1, 1, 1)):

        super().__init__(side_str, device_addr, transposition, scaling)
        self.filter_range = 0           # fast filtered response
        self._mag = None
        self._mag = Vector3d(transposition, scaling, self._mag_callback)
        self._mag_stale_count = 0   # Count of consecutive reads where old data was returned
        self.mag_triggered = False  # Ensure mag is triggered once only until it's read
        self.mag_correction = self._magsetup()  # Returns correction factors.
        self.mag_wait_func = default_mag_wait

    @property
    def sensors(self):
        '''
        returns sensor objects accel, gyro and mag
        '''
        return self._accel, self._gyro, self._mag


    @property
    def mag(self):
        '''
        Triggers mag, waits for it to be ready, then returns the instance
        should be ready in 9mS max
        '''
        while not self.mag_ready:
            self.mag_wait_func()
        return self._mag

    @property
    def mag_nonblocking(self):
        '''
        returns non-blocking magnetometer object
        '''
        return self._mag        # ready or not

    def mag_trigger(self):
        '''
        Initiate a mag reading. Can be called repeatedly.
        '''
        if not self.mag_triggered:
            try:
                self._write(0x01, 0x0A, self._mag_addr)  # single measurement mode
            except OSError:
                raise MPUException(self._I2Cerror)
            self.mag_triggered = True

    @property
    def mag_stale_count(self):
        '''
        Number of consecutive times old data was returned
        '''
        return self._mag_stale_count

    @property
    def mag_ready(self):
        '''
        Initiates a reading if necessary. Returns ready state.
        '''
        self.mag_trigger()
        try:
            self._read(self.buf1, 0x02, self._mag_addr)
        except OSError:
            raise MPUException(self._I2Cerror)
        return bool(self.buf1[0] & 1)

    def _mag_callback(self):
        '''
        Update magnetometer Vector3d object (if data available)
        '''
        try:                                    # If read fails, returns last valid data
            if self.mag_ready:                  # Starts mag if necessary
                self._read(self.buf6, 0x03, self._mag_addr)
                self.mag_triggered = False
            else:
                self._mag_stale_count += 1      # Data not ready: retain last value
                return                          # but increment stale count
            self._read(self.buf6, 0x03, self._mag_addr)  # Mag was ready
            self._read(self.buf1, 0x09, self._mag_addr)  # Read ST2
        except OSError:
            self.mag_triggered = False
            raise MPUException(self._I2Cerror)
        if self.buf1[0] & 0x0C > 0:             # An overflow or data error has occurred
            self._mag_stale_count += 1           # transitory condition? User checks stale count.
            return
        self._mag._ivector[1] = bytes_toint(self.buf6[1], self.buf6[0])  # Note axis twiddling and little endian
        self._mag._ivector[0] = bytes_toint(self.buf6[3], self.buf6[2])
        self._mag._ivector[2] = -bytes_toint(self.buf6[5], self.buf6[4])
        scale = 0.3                             # 0.3uT/LSB
        self._mag._vector[0] = self._mag._ivector[0]*self.mag_correction[0]*scale
        self._mag._vector[1] = self._mag._ivector[1]*self.mag_correction[1]*scale
        self._mag._vector[2] = self._mag._ivector[2]*self.mag_correction[2]*scale
        self._mag_stale_count = 0

    def _magsetup(self):
        '''
        Read magnetometer correction values from ROM. Perform the maths as decribed
        on page 59 of register map and store the results.
        '''
        try:
            self._write(0x0F, 0x0A, self._mag_addr)
            self._read(self.buf3, 0x10, self._mag_addr)
            self._write(0, 0x0A, self._mag_addr)        # Power down mode
        except OSError:
            raise MPUException(self._I2Cerror)
        mag_x = (0.5*(self.buf3[0] - 128))/128 + 1
        mag_y = (0.5*(self.buf3[1] - 128))/128 + 1
        mag_z = (0.5*(self.buf3[2] - 128))/128 + 1
        return (mag_x, mag_y, mag_z)

    def get_mag_irq(self):
        '''
        Uncorrected values because floating point uses heap
        '''
        if not self.mag_triggered:              # Can't do exception handling here
            self._write(1, 0x0A, self._mag_addr)
            self.mag_triggered = True
        self._read(self.buf1, 0x02, self._mag_addr)
        if self.buf1[0] == 1:
            self._read(self.buf6, 0x03, self._mag_addr)  # Note axis twiddling
            self._mag._ivector[1] = bytes_toint(self.buf6[1], self.buf6[0])
            self._mag._ivector[0] = bytes_toint(self.buf6[3], self.buf6[2])
            self._mag._ivector[2] = -bytes_toint(self.buf6[5], self.buf6[4])
            self.mag_triggered = False
