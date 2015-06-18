# Test program for IRQ based access to MPU9150
# Note there will be small differences between the lines because
# of drift or movement occurring between the readings
import pyb
from mpu9150 import MPU9150
import micropython
micropython.alloc_emergency_exception_buf(100)

# Note: with a magnetometer read in the callback, a frequency of 1KHz hogged the CPU
tim = pyb.Timer(4, freq=20)            # freq in Hz

imu = MPU9150('X')

def cb(timer):                          # Callback: populate array members
    imu.get_gyro_irq()
    imu.get_accel_irq()
    imu.get_mag_irq()

tim.callback(cb)
print("You should see slightly different values on each pair of readings.")
print("            Accelerometer                               Gyro                                Magnetometer")
for count in range(10):
    pyb.delay(400)
    scale = 3.33198                     # Correction factors involve floating point
    mag = list(map(lambda x, y : x*y/scale, imu.mag.ixyz, imu.mag_correction))
    print("Interrupt:", [x/16384 for x in imu.accel.ixyz], [x/131 for x in imu.gyro.ixyz], mag)
    pyb.delay(100)
    print("Normal:   ", imu.accel.xyz, imu.gyro.xyz, imu.mag.xyz)
    print()

tim.callback(None)

def timing():                           # Check magnetometer call timings
    imu.mag_triggered = False           # May have been left True by above code
    start = pyb.micros()
    imu.get_mag_irq()
    t1 = pyb.elapsed_micros(start)
    start = pyb.micros()
    imu.get_mag_irq()
    t2 = pyb.elapsed_micros(start)
    pyb.delay(200)
    start = pyb.micros()
    imu.get_mag_irq()
    t3 = pyb.elapsed_micros(start)

    # 1st call initialises hardware
    # 2nd call tests it (not ready)
    # 3rd call tests ready (it will be after 200mS) and reads data
    print(t1, t2, t3) # 1st call takes 265uS second takes 175uS. 3rd takes 509uS
