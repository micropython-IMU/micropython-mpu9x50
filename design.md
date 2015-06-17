# IMU driver design guide

This document provides an overview of the design of the IMU drivers as suggestions for drivers
for new IMU devices. The aim is to provide guidance on the use of the ``Vector3d`` class along with
general hints aimed at achieving a common interface and design approach between drivers. The design
provides for hardware which offers continuous readings, and also devices where a read must be triggered
and a result awaited. In the latter case blocking and nonblocking I/O are supported, with the former
being the default for simplicity of application.

Each IMU instance has three Vector3d members which are conventionally private (names begin with
underscore). User access is via the following IMU properties: ``accel``, ``gyro`` and ``mag``. For 
devices with continuous readings these simply return the underlying _accel, _gyro and _mag members.

For devices which require triggering, access to the relevant member triggers a read and repeatedly
calls a wait function until the device is ready. It then returns the underlying Vector3d for the user
to access. The wait function should be capable of being overidden to provide for the case where the
user needs a thread-aware function.

This means that if a user executes
```python
mx = myimu.mag.x
```
a blocking read is automatically performed.

# Update mechanism

The Vector3d constructor requires an update function which is a method of the IMU driver class.
When the user accesses any property of the Vector3d which needs sensor data this function will
be called: the driver should read the hardware and update the corresponding member vector.

This involves reading integer data from the sensor and storing it in the vector's ``_ivector`` list,
then performing any scaling or correction on each ``_ivector`` value before storing the resultant
float values in the vector's ``_vector`` list. In the event of a potentially recoverable failure
such as an out of range value the update function currently returns without modifying the vector.
This means that the user will get the last good value. I2C errors will raise an MPUException
(a class derived from OSError).

The theory behind this is that IMU's may be used in drones or balancing robots where consequences of
failure could be dire. If there is any chance of recovery from transient errors sourcing old data
seems the best option. The user has the option to trap the MPUException and attempt to continue,
although the chances of success are (in my experience) slim.

The purpose of the vector's ``_ivector`` values is to provide access to the raw sensor data and also to
support optional IMU methods which can update the vector from within an interrupt service routine:
this precludes any use of floating point.

# Calibration

The Vector3d class provides for the removal of fixed offsets from the returned values. The use of
this is described in vector3d.md but in operation it repeatedly calls the update function. By default
there is a 50mS delay between calls but this can be overridden by the user. If the hardware has any
requirement for a minimum delay the user should be made aware of this.

# Nonblocking reads

Where a device performs continuous readings the user can expect a fast response to calling (say)
``imu.mag.x``. However, as explained above, if the device requires triggering, the call will block
until data is available. For such devices only, to cater for cases where this is unacceptable the
following property should be supported (example is for a magnetometer).  
``mag_ready`` This should return True if ready, False if not.  
Typically this determines if the device has been triggered (by means of a boolean instance
variable). If not it triggers it, sets the variable and returns False. If it has been triggered
it reads the hardware status and returns True or False as appropriate.

For such devices the update callback should behave sensibly if the user incorrectly calls it before
the device is ready. To achieve this it should check ``mag_ready`` and only read the device if it
is ready, when it should also clear down the triggered variable.  
If the device is not ready it should simply return - although you may want to keep a tally
of such instances as per ``mag_stale_count`` in the MPU9150 driver. This behaviour means that
the user gets the last good data from the Vector3d instance.  
In the event of an error in communicating with the device the triggered status should be cleared,
before raising an MPUException.  
If the device has been successfully read, the Vector3d should be updated as described above. Example
code is in mpu9150.py ``_mag_callback()``.

# Interrupt-capable updates

If these are to be supported these should be minimal methods to ensure that MicroPython doesn't
access the heap - this results in an immediate runtime error. It is also good practice to return
immediately. Typical application will be in a timer callback with the aim being to continuously
provide updated information. Note that error trapping is impossible as this (currently) uses the
heap. The device read function should use pre-allocated buffers for the same reason.

For devices performing continuous reads, the method simply reads the sensor and updates the Vector3d's
``_ivector`` values.

For blocking devices it should check the ``triggered`` instance variable: if it's not set it should
trigger the device, set the ``triggered`` flag and return. Otherwise it should read the device status.
If it's not ready it should return, otherwise it should read the sensor and update the Vector3d's
``_ivector`` values.

# Sensor read methods

To be compatible with interrupt-capable updates it is suggested that these use pre-allocated buffers.
This may be impractical if the device requires reads of a wide variety of sizes.

Interrupt-capable updates cannot use the struct module to convert from bytes to integers as this
uses the heap. Consequently it should be done in Python code. For consistency this function might be
used throughout the driver.

# General Python approach

In my view print() statements to indicate errors is not good practice in production code. This is
because in applications of any size it can be hard to locate their source. It's more Pythonic to raise a
``ValueError`` with a descriptive message.

Trapping hardware errors needs to be done with caution. If there is a proven method of recovery
it's fine, otherwise there is a risk of the code locking up. If recovery is unlikely (as in I2C errors)
it's better (in my view) to let the user decide what to do by handling the error. I adopt the approach
of trapping the error, doing anything which might aid the user in recovering, then raising an
``MPUException`` derived from ``OSError``.
