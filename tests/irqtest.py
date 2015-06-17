# Test program for IRQ based access to MPU9250
# Note there will be small differences between the lines because
# of drift or movement occurring between the readings
import pyb
from mpu9250 import MPU9250
import micropython
micropython.alloc_emergency_exception_buf(100)

# Note: with a magnetometer read in the callback, a frequency of 1KHz hogged the CPU
tim = pyb.Timer(4, freq=20)            # freq in Hz

imu = MPU9250('X')

def cb(timer):                          # Callback: populate array members
    imu.get_gyro_irq()
    imu.get_accel_irq()
    imu.get_mag_irq()
#    print(imu.accel.ixyz)

tim.callback(cb)
#print("waiting one second")
print("You should see slightly different values on each pair of readings.")
print("            Accelerometer                               Gyro                                Magnetometer")
#pyb.delay(1)
for count in range(10):
    pyb.delay(400)                      # Ensure an interrupt occurs to re-populate integer values
    scale = 6.6666                      # Correction factors involve floating point
    mag = list(map(lambda x, y : x*y/scale, imu.mag.ixyz, imu.mag_correction))
    print("Interrupt:", [x/16384 for x in imu.accel.ixyz], [x/131 for x in imu.gyro.ixyz], mag)
    pyb.delay(100)
    print("Normal:   ", imu.accel.xyz, imu.gyro.xyz, imu.mag.xyz)
    print()

tim.callback(None)
