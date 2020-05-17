# Example 2 of nonblocking magnetometer reads, demonstrating use with uasyncio

# Author: Peter Hinch
# Copyright Peter Hinch 2020 Released under the MIT license

# Expects an MPU9150 on X side and a 24*2 LCD with Hitachi controller wired as per PINLIST
# Requires uasyncio V3 and as_drivers directory (plus contents) from
# https://github.com/peterhinch/micropython-async/tree/master/v3

import uasyncio as asyncio
from machine import I2C
from as_drivers.hd44780.alcd import LCD, PINLIST  # Library supporting Hitachi LCD module
from mpu9150 import MPU9150


async def lcd_task(mylcd, imu):
    print('Running...')
    k = imu.mag_correction
    mylcd[1] = "x {:5.3f} y {:5.3f} z {:5.3f}".format(k[0],k[1],k[2])
    while True:
        try:
            while not imu.mag_ready:
                await asyncio.sleep(0)
        except OSError:
            mylcd[0] = "Mag read failure"
            continue
        xyz = imu.mag.xyz
        mylcd[0] = "x {:5.1f} y {:5.1f} z {:5.1f}".format(xyz[0], xyz[1], xyz[2])
        await asyncio.sleep_ms(500)

lcd0 = LCD(PINLIST, cols = 24)
imu = MPU9150(I2C(1))

try:
    asyncio.run(lcd_task(lcd0, imu))
except KeyboardInterrupt:
    print('Interrupted')
finally:
    asyncio.new_event_loop()

