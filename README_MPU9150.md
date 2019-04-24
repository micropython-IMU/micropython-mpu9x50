# Modules imu and mpu9150

mpu9150 is a micropython module for the InvenSense MPU9150 sensor.
It measures acceleration, turn rate and the magnetic field in three axes.

imu supports the MPU6050, which is the same device but without the magnetometer.
It provides the base class for the MPU9150 class.

If you have any questions, open an issue.

# Introduction

The MPU9150 is a dual chip module, with the magnetometer provided by an AsahaiKASEI AK8975 chip.
In consequence the coordinate system of the magnetometer is not aligned with that
of the other components. This driver corrects this so that the axes of each instrument
correspond with those of the accelerometer.

If the driver is used with sensor fusion e.g. [sensor fusion module](https://github.com/micropython-IMU/micropython-fusion.git)
the orientation of the sensor relative to the vehicle is significant. The Madgwick algorithm assumes
x is orientated towards the front of the vehicle, y is left-right, and z is down. To accommodate
cases where the sensor is mounted orthogonally to this orientation, support is provided for inverting and
transposing axes. The driver returns vehicle-relative coordinates.

### Wiring the sensor to the pyboard

| pyboard| mpu9150 |
|:------:|:-------:|
| VIN    | 3V3     |
| GND    | GND     |
| SCL    | SCL     |
| SDA    | SDA     |

### Quickstart

MPU9150 Example:
Example assuming an MPU9150 connected to 'X' I2C interface on the Pyboard:
```python
from mpu9150 import MPU9150
imu = MPU9150('X')
print(imu.accel.xyz)
print(imu.gyro.xyz)
print(imu.mag.xyz)
print(imu.temperature)
print(imu.accel.z)
```

MPU6050 Example:
Example assuming an MPU6050 connected to 'X' I2C interface on the Pyboard:
```python
from imu import MPU6050
imu = MPU6050('X')
print(imu.accel.xyz)
print(imu.gyro.xyz)
print(imu.temperature)
print(imu.accel.z)
```


# Modules

The following modules should be available on the target hardware:

 1. imu.py
 2. mpu9150.py (unless running an MPU6050).
 3. vector3d.py

To employ the driver it is necessary to import the mpu9150 module and to use
the ``MPU9150`` class as per quickstart above. For MPU6050 support issue the
following:

```python
from imu import MPU6050
imu = MPU6050('X')  # Arguments to match hardware
```

### mpu9150

``MPU9150``  
Class for the MPU9150 sensor.

### imu

``MPU6050``  
Base class for InvenSense inertial measurement units.  
``MPUException``  
This will be raised in the event of an I2C error. It is derived from the Python ``OSError``.

### vector3d

``Vector3d``  
Class for a 3D vector. This is documented [here](./vector3d.md).

# MPU9150 and MPU6050 Classes

The MPU9150 class is a superset of MPU6050. The classes are documented as one.
Features missing from the MPU6050 relate to the magnetometer and are detailed
below.

The class has various properties returning Vector3d instances. The principal properties
of the Vector3d are ``x``, ``y`` and ``z`` returning the vector's components and ``xyz`` which returns
a 3-tuple of the components (x, y, z). It also supports calibration and conversion to vehicle
relative coordinates.

## Methods

``MPU9150()`` The constructor supports the following arguments  
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
``get_mag_irq()`` (MPU9150 only)  
These methods are somewhat experimental. They are capable of being called from within an
interrupt callback and update the integer properties only of the relevant Vector3D object.
Currently writing nontrivial MicroPython interrupt callbacks is something of a black art
as it can be unclear when the heap is likely to be invoked causing an exception.

## Principal Properties

``sensors``  
Returns three Vector3d objects, accelerometer, gyro and magnetometer. In the
case of the MPU6050 accelerometer and gyro objects only are returned.

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

``filter_range`` integer read/write.  
Sets or returns the current accel and gyro low pass filter range. Values can range from 0-6
with values out of that range printing a message and being ignore.

The digital low pass filter enables the effect of vibration to be reduced in the accelerometer
and gyro readings. The following table gives the approximate bandwidth and delay for the filter.
Precise values differ slightly between the accelerometer and the gyro and can be obtained
from the device datasheet.

| value | bw(Hz) | Delay(mS) |
|:-----:|:------:|:---------:|
|  0    |  260   |    0.0    |
|  1    |  184   |    2.0    |
|  2    |   94   |    3.0    |
|  3    |   44   |    4.9    |
|  4    |   21   |    8.5    |
|  5    |   10   |   13.8    |
|  6    |    5   |   19.0    |

``temperature`` float read only  
Returns the chip temperature in degrees celcius

``accel`` Vector3d instance read only  
Returns the ``Vector3d`` holding the current accelerometer data. Units are g.

``gyro`` Vector3d instance read only  
Returns the ``Vector3d`` holding the current gyro data. Units degrees/s.

The following are unavailable in the case of the MPU6050:

``mag``  Vector3d instance read only  
This property supports blocking reads and returns the ``Vector3d`` holding the current
magnetometer data. Units are uT (microtesla).  
Access to this property will result in an immediate return if data is ready. Otherwise it
will initiate a read and only return when the reading is complete: this takes up to 10mS.
Consequently you can be sure that the data returned from the ``Vector3d`` instance
was correct at the time it was returned. The first access of the instance x, y, z or xyz properties
will immediately return that data. Subsequent accesses may return more recent data  since any
access initiates another read. Hence if the application requires that all the vector components
relate to the same instant in time the xyz property should be employed.

To illustrate this the following code sequence takes 27mS to execute since each access to
the mag property triggers, and waits for, a read (assuming ``a`` is an MPU9150 instance)
```python
p, q, r = a.mag.x, a.mag.y, a.mag.z
```
If timing is critical, the following prompts a single read and takes 9mS
```python
p, q, r = a.mag.xyz
```
In the event that the most recent data read by the magnetometer is in error the Vector3d
instance will return the most recent valid data and increment ``mag_stale_count``.
When using blocking reads a nonzero value of ``mag_stale_count`` suggests an error condition.

For greater control over timing use the ``mag_nonblocking`` property below.

``mag_nonblocking``  Vector3d instance read only  
This supports nonblocking reads and returns instantly. Returns the ``Vector3d`` holding
the current magnetometer data. Units are uT (microtesla).  
Any read of the ``Vector3d`` x, y, z, or xyz properties will trigger a read, but if data
is not ready it will return the most recent good data and increment ``mag_stale_count``
described below. In practice the nonblocking read should be performed by polling
the ``mag_ready`` property and only reading the ``mag_nonblocking`` Vector3D when
it is ready (see code sample below). If used in this way any nonzero value of
 ``mag_stale_count`` indicates an error has occurred.

``mag_ready`` read only  
This facilitates nonblocking reads. Reading this will trigger a magnetometer read
if one is not in progress. It will return True if ready, False if not ready.  A nonblocking
read at its simplest could be coded as follows
```python
while not imu.mag_ready:
    pass # do something useful
x, y, z = imu.mag.xyz # will return immediately
```

``mag_stale_count`` integer read only  
As described above: a count of the number of consecutive times in the curent sequence
of reads that the driver has returned out-of-date values.

``mag_wait_func``  
When doing a blocking read the driver repeatedly calls this function. The default function provides a 1mS
delay but the user can override this, typically to provide a thread-aware delay in cooperative
multitasking environments. Longer delays will have no effect other than increasing the latency
of the read.

See "Other properties" below.

#### Use in interrupt callbacks

MicroPython interrupt callbacks prohibit the use of the heap, which rules out a number of
standard Python techniques including exception handling and the use of floating point.
Currently it is not always evident whether code will use the heap, so any use of these
techniques should be considered experimental. The following MPU9150 methods provide access
to the device where this must be performed in a callback.

``get_accel_irq()``  
``get_gyro_irq()``  
``get_mag_irq()`` (MPU9150 only)  
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

## Other properties

``passthrough`` Boolean read/write  
sets passthrough mode. It is activated (True) by default. This needs to be activated
to enable the magnetometer interface to be linked to the SDA, SCL pads.

``sample_rate`` 8 bit integer read/write  
Determines the update rate of the sensor registers. Values should be in range 0 to 255.
The update rate is given by rate = Internal_Sample_Rate / (1 + sample_rate). It is not
clear, given the subset of functionality supported by this driver, why you might want
to change this from the zero default.

``chip_id``  
The ID of chip (104) is hardcoded on the sensor. Reading this property will test the
communications with the IMU by reading this value which is returned to the user. A
ValueError will be raised if the value is incorrect.

``_mpu_addr``  
I2C adress of the accelerometer and the gyroscope.

For MPU9150 only:

``mag_correction`` float 3-tuple  
Holds factory-set magnetometer correction values.

``_mag_addr``  
I2C adress of the magnetometer.

# Exception handling

Incorrect values such as  
```python
imu = MPU9150('Z')
```
will raise a ValueError with a meaningful error message.  
When any communication with the IMU takes place it is possible for the I2C bus to lock up.
This is normally a consequence of hardware failure such as the device being connected incorrectly
or leads being too long. In this instance a custom MPUException will be raised with a
descriptive message. This is derived from Python's OSError: the user may trap either in the hope
of continuing operation. In my experience this seldom works: if the I2C bus locks up a
power cycle is required to clear it.

# Transposition and Scaling

These cater for the case where the sensor is mounted orthogonally to the vehicle coordinates,
and where the user wishes to return vehicle coordinates (usually for sensor fusion). Vehicle
coordinates are conventionally:  
X forward direction of motion (positive forwards)  
Y Left-right  
Z Vertical (positive towards ground)  

Both parameters are passed to the IMU constructor as 3-tuples of integers. The transposition
tuple should contain the numbers 0, 1 and 2. The first number indicates the IMU coordinate
which should be regarded as X, the second Y and the final Z. Hence (1, 0, 2) indicates a
sensor mounted in the horizontal plane but rotated through 90 degrees so that X and Y are
interchanged. The default (0, 1, 2) is a conventionally mounted IMU.

The scaling tuple normally contains the numbers 1 and -1 only. Each value returned by the driver
is multiplied by the corresponding scaling value. Hence (1, 1, -1) corresponds to a sensor
mounted upside down. The default (1, 1, 1) is a conventionally mounted IMU. Scaling is applied
after transposition. In other words it is applied to vehicle coordinates rather than sensor
coordinates.

# Demo of magnetometer calibration

```python
>>> from  mpu9150 import MPU9150
>>> a = MPU9150('X')
>>> a.mag.cal
(0, 0, 0)
>>> import pyb
>>> sw = pyb.Switch()
>>> a.mag.calibrate(sw) # User rotates unit about each axis then presses the Pyboard switch
>>> a.mag.cal
(35.30567, 18.92022, -9.428905)
>>>
```
