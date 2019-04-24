As of now, this repo holds:
* vector3d - a vector class for IMU devices
* imu - a base class for MPU9x50 devices. Supports the MPU6050.
* mpu9150 - a class for the MPU9150
* mpu9250 - a class for the MPU9250

vector3d will eventually be spun out to serve as common starting point for other
imu-device drivers.  
Documentation for the MPU9150/MPU6050 can be found [here](./README_MPU9150.md).

# Module mpu9250

mpu9250 is a micropython module for the InvenSense MPU9250 sensor.
It measures acceleration, turn rate and the magnetic field in three axes.  
Breakout board: [Drotek](https://store.drotek.com/imu-10dof-mpu9250-ms5611),
[Sparkfun](https://www.sparkfun.com/products/13762)  

The MPU9250 has a number of advantages over the MPU9150 (which has manufacturing status
"not recommended for new designs"). The magnetometer is capable of continuous operation,
updating at 100Hz. It supports separate filters for accelerometer and gyro. And the
breakout board is cheaper. It also supports SPI however this driver assumes an I2C connection.

# Introduction

The MPU9250 is a dual chip module, with the magnetometer provided by an AsahaiKASEI AK8963 chip.
In consequence the coordinate system of the magnetometer is not aligned with that
of the other components. This driver corrects this so that the axes of each instrument
correspond with those of the accelerometer.

Since the driver is likely to be used with the [sensor fusion module](https://github.com/micropython-IMU/micropython-fusion.git)
the orientation of the sensor relative to the vehicle is significant. The Madgwick algorithm assumes
x is orientated towards the front of the vehicle, y is left-right, and z is down. To accommodate
cases where the sensor is mounted orthogonally to this orientation, support is provided for inverting and
transposing axes. The driver returns vehicle-relative coordinates.

### Wiring the sensor to the pyboard

The MPU9250 chip does not support 5V operation however the Drotek board has a voltage regulator so
can be powered from 3.3V or 5V.

| pyboard| mpu9250 |
|:------:|:-------:|
| VIN    | 3V3     |
| GND    | GND     |
| SCL    | SCL     |
| SDA    | SDA     |

### Quickstart

Example assuming an MPU9250 connected to 'X' I2C interface on the Pyboard:
```python
from mpu9250 import MPU9250
imu = MPU9250('X')
print(imu.accel.xyz)
print(imu.gyro.xyz)
print(imu.mag.xyz)
print(imu.temperature)
print(imu.accel.z)
```

# Modules

To employ the driver it is only necessary to import the mpu9250 module and to use the ``MPU9250`` class.

### mpu9250

``MPU9250``  
Class for the MPU9250 sensor.  
``MPUException``  
This will be raised in the event of an I2C error. It is derived from the Python ``OSError``.

### imu

``InvenSenseMPU``  
Base class for InvenSense inertial measurement units.

### vector3d

``Vector3d``  
Class for a 3D vector. This is documented [here](./vector3d.md).

# MPU9250 Class

The class has various properties returning Vector3d instances. The principal properties
of the Vector3d are ``x``, ``y`` and ``z`` returning the vector's components and ``xyz`` which returns
a 3-tuple of the components (x, y, z). It also supports calibration and conversion to vehicle
relative coordinates.

## Methods

``MPU9250()`` The constructor supports the following arguments  
  1. side_str 'X' or 'Y' (mandatory) defines the I2C interface in use. Alternatively an initialised
I2C object may be passed.
  2. device_addr 0 or 1 (optional) Two devices may be used with addresses determined by the voltage
on the AD0 pin. If only one device is used, this argument may be None when the device
will be automatically detected.
  3. transposition (optional) Enables axes to be transposed (see below).
  4. scaling  (optional) Enables axes to be inverted (see below).

The defaults for transposition and scaling will cause the driver to return sensor-relative results.

`` wake()``  
wakes the device  

``sleep()``  
sets the device to sleep mode  

``get_accel_irq()``  
``get_gyro_irq()``  
``get_mag_irq()``  
These methods are somewhat experimental. They are capable of being called from within an
interrupt callback and update the integer properties only of the relevant Vector3D object.
Currently writing nontrivial MicroPython interrupt callbacks is something of a black art
as it can be unclear when the heap is likely to be invoked causing an exception.

## Principal Properties

``sensors``  
Returns three Vector3d objects, accelerometer, gyro and magnetometer.

``accel_range`` integer 0 to 3 read/write  
Returns or sets the current accelerometer range: this determines the accelerometer full scale
range as per the table below. Note that the x, y, z values from the driver are scaled to units
of g regardless of the range selected. Range affects only the full scale capability and resolution.

| Range | Value (+-g) |
|:-----:|:-----------:|
|   0   |    2        |
|   1   |    4        |
|   2   |    8        |
|   3   |   16        |

``gyro_range`` integer 0 to 3 read/write  
Determines the gyro full scale range as per the table below. Note that the x, y, z
values from the driver are scaled to units of degs/s regardless of the range selected.
Range affects only the full scale capability and resolution.

| Range | Value +- degs/sec |
|:-----:|:-----------------:|
|   0   |       250         |
|   1   |       500         |
|   2   |      1000         |
|   3   |      2000         |

``temperature`` float read only  
Returns the chip temperature in degrees celcius

``accel`` Vector3d instance read only  
Returns the ``Vector3d`` holding the current accelerometer data. Units are g.

``gyro`` Vector3d instance read only  
Returns the ``Vector3d`` holding the current gyro data. Units degrees/s.

``mag``  Vector3d instance read only  
Returns the ``Vector3d`` holding the current magnetometer data. As configured by this
driver the MPU9250 reads the magnetometer at 10mS intervals. Units are uT (microtesla).

Accessing a Vector3d instance x, y, z or xyz properties will cause the device to be
read and will return the latest data. In the case of the magnetometer, if it is accessed,
and then accessed again within 10mS and before the unit has had a chance to update,
the old data will be returned and a counter ``MPU9250.mag_stale_count`` will be
incremented. The counter will be cleared the first time fresh data is acquired and read.

In the event that the most recent data read by the magnetometer is in error the Vector3d
instance will return the most recent valid data and increment ``mag_stale_count``.
A high value of ``mag_stale_count`` (or any nonzero value if the read rate is below 100Hz)
suggests an error condition.

``mag_stale_count`` integer read only  
As described above: a count of the number of consecutive times in the curent sequence
of reads that the driver has returned out-of-date values.

``accel_filter_range`` integer 0 to 7 read/write  
The digital low pass filter enables the effect of vibration to be reduced in the
accelerometer readings. The following table gives the approximate bandwidth
and delay for the filter.

| value | bw(Hz) | Delay(mS) |
|:-----:|:------:|:---------:|
|  0    |  460   |    1.94   |
|  1    |  184   |    5.8    |
|  2    |   92   |    7.8    |
|  3    |   41   |   11.8    |
|  4    |   20   |   19.8    |
|  5    |   10   |   35.7    |
|  6    |    5   |   66.96   |
|  7    |  460   |    1.94   |

Note: in my testing option 7 produced garbage.

``gyro_filter_range`` integer 0 to 7 read/write  

The digital low pass filter enables the effect of vibration to be reduced in the
gyro readings. The following table gives the approximate bandwidth and delay for the filter.

| value | bw(Hz) | Delay(mS) |
|:-----:|:------:|:---------:|
|  0    |  250   |    0.97   |
|  1    |  184   |    2.9    |
|  2    |   92   |    3.9    |
|  3    |   41   |    5.9    |
|  4    |   20   |    9.9    |
|  5    |   10   |   17.85   |
|  6    |    5   |   33.48   |
|  7    | 3600   |    0.17   |

See "Other MPU9250 properties" below.

#### Use in interrupt callbacks

MicroPython interrupt callbacks prohibit the use of the heap, which rules out a number of
standard Python techniques including exception handling and the use of floating point.
Currently it is not always evident whether code will use the heap, so any use of these
techniques should be considered experimental. The following MPU9250 methods provide access
to the device where this must be performed in a callback.

``get_accel_irq()``  
``get_gyro_irq()``  
``get_mag_irq()``  
These methods read the device and update the integer values of the Vector3D objects only.
These values hold unscaled values direct from the device so coordinates are device
relative and no calibration, correction or scaling is applied.

Note that the scaling factors of the accelerometer and gyro depend only on the range
selected, hence these can be hard coded in the callback. To get the most accurate readings
from the magnetometer the factory-set corrections should be applied. These are in
the property ``mag_correction`` but being floating point values cannot be used in a callback.
Options (depending on application) are to apply them outside the callback, or convert
them to scaled integers in initialisation code.

See tests/irqtest.py for example code.

## Other MPU9250 properties

``passthrough`` Boolean read/write  
sets passthrough mode. It is activated (True) by default. This needs to be activated
to enable the magnetometer interface to be linked to the SDA, SCL pads.

``sample_rate`` 8 bit integer read/write  
Determines the update rate of the sensor registers. Values should be in range 0 to 255.
The update rate is given by rate = Internal_Sample_Rate / (1 + sample_rate). It is not
clear, given the subset of functionality supported by this driver, why you might want
to change this from the zero default.

``mag_correction`` float 3-tuple  
Holds factory-set magnetometer correction values.

``chip_id``  
The ID of chip (113) is hardcoded on the sensor. Reading this property will test the
communications with the IMU by reading this value which is returned to the user. A
ValueError will be raised if the value is incorrect.

``mpu_addr``  
I2C adress of the accelerometer and the gyroscope.

``mag_addr``  
I2C adress of the magnetometer.

``timeout``  
Timeout for I2C operations.

# Exception handling

Incorrect values such as
```python
imu = MPU9250('Z')
```
will raise a ValueError with a meaningful error message.  
When any communication with the IMU takes place it is possible for the I2C bus to lock up.
This is normally a consequence of hardware failure such as the device being connected incorrectly
or leads being too long. In this instance a custom MPUException will be raised with a
dscriptive message. This is derived from Python's OSError: the user may trap either in the hope
of continuing operation. In my experience this seldom works: if the I2C bus locks up a
power cycle is required to clear it.

# Demo of calibration

```python
>>> from  mpu9250 import MPU9250
>>> a = MPU9250('x')
>>> a.mag.cal
(0, 0, 0)
>>> import pyb
>>> sw = pyb.Switch()
>>> a.mag.calibrate(sw) # User rotates unit about each axis then presses the Pyboard switch
>>> a.mag.cal
(35.30567, 18.92022, -9.428905)
>>>
```

