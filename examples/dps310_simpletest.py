import time
import board
import busio
import adafruit_dps310
from adafruit_debug_i2c import DebugI2C

i2c = DebugI2C(busio.I2C(board.SCL, board.SDA))

#i2c = busio.I2C(board.SCL, board.SDA)
print("DPS310 Test")
print("******************************************************************")

dps310 = adafruit_dps310.DPS310(i2c)

# for cnt in range(2):
while True:
    # > Pressure = 1023.07 hPa

    # print("prssure(bin): ", format(dps310.pressure, '#010b'))
    print("Press:", dps310.pressure)
    time.sleep(0.1)
