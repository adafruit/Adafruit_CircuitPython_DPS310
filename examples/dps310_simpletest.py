import time
import board
import busio
import adafruit_dps310
from kattni_debug_i2c import DebugI2C

i2c = DebugI2C(busio.I2C(board.SCL, board.SDA))

# i2c = busio.I2C(board.SCL, board.SDA)
print("DPS310 Test")
print("******************************************************************")

dps310 = adafruit_dps310.DPS310(i2c)
# print("Press:", dps310.pressure)

while True:
    print("temp scale: ", dps310._temp_scale)
    print("Temp: ", dps310.temperature)
    time.sleep(0.4)
