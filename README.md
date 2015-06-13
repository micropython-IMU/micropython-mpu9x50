# Module mpu9250

mpu9250 is a micropython module for the InvenSense MPU9250 sensor.
It measures acceleration, turn rate and the magnetic field in three axes.  
Breakout board: (Drotek)[http://www.drotek.fr/shop/en/home/421-mpu9250-gyro-accelerometer-magnetometer.html]  

The MPU9250 has a number of advantages over the MPU9150 (which has manufacturing status
"not recommended for new designs"). The magnetometer is capable of continuous operation,
updating at 100Hz. It supports separate filters for accelerometer and gyro. And the
breakout board is cheaper. It also supports SPI however this driver assumes an I2C connection.

# Introduction

The MPU9250 is a dual chip module, with the magnetometer provided by an AsahaiKASEI AK8963 chip.
In consequence the coordinate system of the magnetometer is not aligned with that
of the other components. This driver corrects this so that the axes of each instrument
correspond with those of the accelerometer.

Since the driver is likely to be used with the (sensor fusion module)[https://github.com/micropython-IMU/micropython-fusion.git]
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
```

# Classes

``MPU9250``
Module for the MPU9250 sensor.

``Vector3d``
Object containing a 3D vector

# MPU9250 Class

## Methods

``MPU9250()`` The constructor supports the following arguments  
  1. side_str 'X' or 'Y' (mandatory) defines the I2C interface in use.  
  2. device_addr 0 or 1 (optional) Two devices may be used with addresses determined by the voltage
on the AD0 pin. If only one device is used, this argument may be None when the device
will be automatically detected
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
Returns three Vector3D objects, accelerometer, gyro and magnetometer.

``accel_range`` integer 0 to 3 read/write  
Returns the current accel range and sets it to ``accel_range`` if argument is passed.  
Determines the accelerometer full scale range as per the table below. Note that the
x, y, z values from the driver are scaled to units of g regardless of the range selected.
Range affects only the full scale capability and resolution.

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
accelerometer and gyro readings. The following table gives the approximate bandwidth
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
accelerometer and gyro readings. The following table gives the approximate bandwidth
and delay for the filter.

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

# Vector3D Class

This represents a vector in a 3D space which can be a magnetic field, an acceleration or
a rotation rate. It has read-only x y and z properties which represent the vector's
components relative to the vehicle containing the sensor. It supports cases where the
sensor is mounted orthogonally to the vehicle coordinates: (sensor fusion)[https://github.com/peterhinch/micropython-fusion]
assumes that the x axis corresponds to the front of the vehicle, y is left-right, and
z is vertical. 

The class's internal storage uses Cartesian coordinates relative to the sensor.

## Methods

``Vector3d()`` Constructor has two mandatory arguments
  1. transposition a 3-tuple which must contain three unique integer values from 0 to 2. It relates
  vehicle axes to sensor axes. Hence (0,1,2) effects no change. (1,0,2) interchanges the x and y
  axes for the case where the sensor is rotated through 90 degrees.
  2. scaling A 3-tuple with each element normally being 1 or -1. Returned x, y and z values are
  multiplied by the corresponding value in the tuple. Allows aexes to be inverted.

``calibrate()`` Optional method to enable fixed offsets to be stored and allowed for. Arguments:
  1. Stop function: routine will run until this passed function returns False.
  2. Wait function: Optional. Default is a 50mS delay between readings. User can supply
  an alternative, for example a thread-aware delay in cooperative multitasking environments.

The function takes repeated readings until stopped, when it stores the average vector as a 3-tuple
in the ``cal`` property. This is then used in subsequent x, y, z and xyz readings. Note that the
calibration procedure for the magnetometer involves rotating the sensor around each orthogonal
axis. For the gyro the unit should be held stationary. Calibration is not normally required for
the accelerometer.

## Properties

``x`` ``y`` ``z`` float read only  
These return the scaled vehicle relative coordinates.

``xyz`` float 3-tuple read only  
Returns a 3-tuple (x,y,z) of scaled vehicle relative coordinates.
Note that the above properties cause the device x, y and z values to be read. Use ``xyz`` if
timing is critical or if it is necessary that all three values should originate at the same instant.

``ix`` ``iy`` ``iz`` integer read only  
Return unscaled, sensor relative, integer coordinates as read from the device.

``ixyz`` integer 3-tuple read only  
Returns (x,y,z) of sensor relative, integer coordinates as read from the device.
The integer reads do not provoke a device read, returning the last data to be read by any
method.

``transpose`` integer 3-tuple read only  
Returns transposition 3-tuple as passed to constructor.

``scale`` integer 3-tuple read only  
Returns scaling 3-tuple as passed to constructor.

``magnitude`` float read only  
Convenience property returning the absolute magnitude of the vector.

``cal`` float 3-tuple
Holds the calibration offsets.

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
the property ``mag_correction[]`` but being floating point values cannot be used in a callback.
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
ID of chip (113) )is hardcoded on the sensor.

``mpu_addr``  
I2C adress of the accelerometer and the gyroscope.

``mag_addr``  
I2C adress of the magnetometer.

``timeout``  
Timeout for I2C operations.
