# Module vector3d

# Vector3d Class

This represents a vector in a 3D space which can be a magnetic field, an acceleration or
a rotation rate. It has read-only x y and z properties which represent the vector's
components relative to the vehicle containing the sensor. It supports cases where the
sensor is mounted orthogonally to the vehicle coordinates: [sensor fusion](https://github.com/micropython-IMU/micropython-fusion.git)
assumes that the x axis corresponds to the front of the vehicle, y is left-right, and
z is vertical. 

The class's internal storage uses Cartesian coordinates relative to the sensor.

## Methods

Users of the IMU classes need not be concerned with the constructor as this is used only by
the IMU classes.

``Vector3d()`` Constructor has three mandatory arguments
  1. transposition a 3-tuple which must contain three unique integer values from 0 to 2. It relates
  vehicle axes to sensor axes. Hence (0,1,2) effects no change. (1,0,2) interchanges the x and y
  axes for the case where the sensor is rotated through 90 degrees in the horizontal plane.
  2. scaling A 3-tuple with each element normally being 1 or -1. Returned x, y and z values are
  multiplied by the corresponding value in the tuple. Allows axes to be inverted.
  3. update_function Normally a method of the class of which the ``Vector3d()`` is a member.
  It is called when the user accesses ``Vector3d()`` properties which require the sensor to be read.
  It will update the ``Vector3d()`` internal ``_vector[]`` and ``_ivector[]`` lists.

``calibrate()`` Optional method to enable fixed offsets to be stored and allowed for. Arguments:
  1. Stop function: routine will run until this passed function returns ``False``.
  2. Wait function: Optional. Default is a 50mS delay between readings. User can supply
  an alternative, for example a thread-aware delay in cooperative multitasking environments.

The function takes repeated readings until stopped, when it stores the average vector as a 3-tuple
in the ``cal`` property. This is then used in subsequent x, y, z and xyz readings. Note that the
calibration procedure for the magnetometer involves rotating the sensor around each orthogonal
axis. For the gyro the unit should be held stationary. Calibration is not normally required for
accelerometer but I would expect it to be performed with device rotation as per the magnetometer.

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
Returns the absolute magnitude of the vector.

``azimuth``  float read only  
Returns the angle (in degrees) between y and x vector components.

``elevation`` float read only  
Returns the angle (in degrees) of the vector relative to the xy plane such that a horizontal
vector will return zero degrees.

``inclination`` float read only  
Returns the angle (in degrees)  of the vector relative to the xy plane such that a horizontal
vector will return 90 degrees. This is typically used with the accelerometer where a horizontal sensor
or vehicle will have vector components of approximately (0.0, 0.0, 1.0) and an inclination of zero.

``cal`` float 3-tuple  
Holds the calibration offsets.
