# The MIT License (MIT)
#
# Copyright (c) 2020 Bryan Siepert for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_dps310`
================================================================================

Library for the DPS310 Precision Barometric Pressure Sensor


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

.. todo:: Update the PID for the below and add links to any specific hardware product page(s), or category page(s)
* Adafruit's DPS310 Breakout: https://adafruit.com/product/44XX

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_DPS310.git"

# Common imports; remove if unused or pylint will complain
from time import sleep
import adafruit_bus_device.i2c_device as i2c_device
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct, Struct
from adafruit_register.i2c_struct_array import StructArray
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits

_DPS310_DEFAULT_ADDRESS = 0x77 # DPS310 default i2c address
_DPS310_DEVICE_ID = 0x10 # DPS310 device identifier

_DPS310_PRSB2 = 0x00       # Highest byte of pressure data
_DPS310_TMPB2 = 0x03       # Highest byte of temperature data
_DPS310_PRSCFG = 0x06      # Pressure configuration
_DPS310_TMPCFG = 0x07      # Temperature configuration
_DPS310_MEASCFG = 0x08     # Sensor configuration
_DPS310_CFGREG = 0x09      # Interrupt/FIFO configuration
_DPS310_RESET = 0x0C       # Soft reset
_DPS310_PRODREVID = 0x0D   # Register that contains the part ID
_DPS310_TMPCOEFSRCE = 0x28 # Temperature calibration src




#   bool _init(void);
#   void _readCalibration(void);
#   void _read();



#   void reset(void);
#   void setMode(dps310_mode_t mode);

#   void configurePressure(dps310_rate_t rate, dps310_oversample_t os);
#   void configureTemperature(dps310_rate_t rate, dps310_oversample_t os);

#   bool pressureAvailable(void);
#   bool temperatureAvailable(void);


#pylint: enable=bad-whitespace
class CV:
    """struct helper"""

    @classmethod
    def add_values(cls, value_tuples):
        """Add CV values to the class"""
        cls.string = {}
        cls.lsb = {}

        for value_tuple in value_tuples:
            name, value, string, lsb = value_tuple
            setattr(cls, name, value)
            cls.string[value] = string
            cls.lsb[value] = lsb

    @classmethod
    def is_valid(cls, value):
        """Validate that a given value is a member"""
        return value in cls.string

class Mode(CV):
    """Options for ``mode``"""
    pass #pylint: disable=unnecessary-pass

# attribute name, value, string, lsb value
#pylint: disable=no-member
Mode.add_values((
    ('IDLE', 0, "Idle", None),
    ('ONE_PRESSURE', 1, "One-Shot Pressure", None),
    ('ONE_TEMPERATURE', 2, "One-Shot Temperature", None),
    ('CONT_PRESSURE', 3, "Continuous Pressure", None),
    ('CONT_TEMP', 4, "Continuous Temperature", None),
    ('CONT_PRESTEMP', 5, "Continuous Pressure & Temperature", None),
))

class Rate(CV):
    """Options for data_rate"""
    pass

Rate.add_values((
    ('RATE_1_HZ', 0, 1, None),
    ('RATE_2_HZ', 1, 2, None),
    ('RATE_4_HZ', 2, 4, None),
    ('RATE_8_HZ', 3, 8, None),
    ('RATE_16_HZ', 4, 16, None),
    ('RATE_32_HZ', 5, 32, None),
    ('RATE_64_HZ', 6, 64, None),
    ('RATE_128_HZ', 7, 128, None)
))

class Samples(CV):
    """Options for oversample_count"""
    pass

Samples.add_values((
    ('COUNT_1', 0, 1, None),
    ('COUNT_2', 1, 2, None),
    ('COUNT_4', 2, 4, None),
    ('COUNT_8', 3, 8, None),
    ('COUNT_16', 4, 16, None),
    ('COUNT_32', 5, 32, None),
    ('COUNT_64', 6, 64, None),
    ('COUNT_128', 7, 128, None),
))

class DPS310:
    """Library for the DPS310 Precision Barometric Pressure Sensor.

        :param ~busio.I2C i2c_bus: The I2C bus the DPS310 is connected to.
        :param address: The I2C slave address of the sensor

    """


    # Register definitions
    _device_id = ROUnaryStruct(_DPS310_PRODREVID, ">B")
    _reset = UnaryStruct(_DPS310_RESET, ">B")
    _mode_bits = RWBits(3, _DPS310_MEASCFG, 0)
    #   Adafruit_BusIO_Register MEAS_CFG = Adafruit_BusIO_Register(
    #       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_MEASCFG, 1);
    #   Adafruit_BusIO_RegisterBits modebits =
    #       Adafruit_BusIO_RegisterBits(&MEAS_CFG, 3, 0);

    #     Adafruit_BusIO_Register PRS_CFG = Adafruit_BusIO_Register(
    #   i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_PRSCFG, 1);

    _ratebits = RWBits(3, _DPS310_PRSCFG, 4)
    _osbits = RWBits(4, _DPS310_PRSCFG, 0)

    #   Adafruit_BusIO_RegisterBits ratebits =
    #       Adafruit_BusIO_RegisterBits(&PRS_CFG, 3, 4);

    #   Adafruit_BusIO_RegisterBits osbits =
    #       Adafruit_BusIO_RegisterBits(&PRS_CFG, 4, 0);

    _shiftbit = RWBit(_DPS310_CFGREG, 2)
    #   Adafruit_BusIO_Register CFG_REG = Adafruit_BusIO_Register(
    #       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_CFGREG, 1);
    #   Adafruit_BusIO_RegisterBits shiftbit =
    #       Adafruit_BusIO_RegisterBits(&CFG_REG, 1, 2);

    _sensor_ready = RWBit(_DPS310_MEASCFG, 6)
    #     Adafruit_BusIO_Register MEAS_CFG = Adafruit_BusIO_Register(
    #       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_MEASCFG, 1);
    #   Adafruit_BusIO_RegisterBits SENSOR_RDY =
    #       Adafruit_BusIO_RegisterBits(&MEAS_CFG, 1, 6);

    _raw_pressure = ROUnaryStruct(_DPS310_PRSB2, ">b")
    #  Adafruit_BusIO_Register PRS_B2 = Adafruit_BusIO_Register(
    #   i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_PRSB2, 3, MSBFIRST);

    _raw_temperature = ROUnaryStruct(_DPS310_TMPB2, ">b")

    #       Adafruit_BusIO_Register TMP_B2 = Adafruit_BusIO_Register(
    #   i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_TMPB2, 3, MSBFIRST);


    _oversample_scalefactor = (524288, 1572864, 3670016, 7864320, 253952,
        516096, 1040384, 2088960)

    def __init__(self, i2c_bus, address=_DPS310_DEFAULT_ADDRESS):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        if self._device_id != _DPS310_DEVICE_ID:
            raise RuntimeError("Failed to find DPS310 - check your wiring!")

        self.initialize()

    def initialize(self):
        """Reset the sensor to the default state"""
        print("called initialize")
        self.reset()
        # wait for hardware reset to finish
        print("reset finished")
        self._read_calibration()
        # // default to high precision
        # self.pressure_configuration(Rate.RATE_64_HZ, Samples.COUNT_64)
        # configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
        # self.temperature_configuration(Rate.RATE_64_HZ, Samples.COUNT_64)
        # setMode(DPS310_CONT_PRESTEMP);
        self.mode = Mode.CONT_PRESSURE
        # // wait until we have at least one good measurement
        # while (!temperatureAvailable() || !pressureAvailable()) {
        #     delay(10);
        # }
        # return true;
        sleep(1)
        print("mode:", self.mode)

    def reset(self):
        """Perform a soft-reset on the sensor"""
        self._reset = 0x89
        while not self._sensor_ready:
            sleep(0.001)

    @property
    def pressure(self):
        """Returns the current pressure reading in kPA"""

        # raw_pressure = self._raw_pressure
        # raw_pressure = twosComplement(PRS_B2.read(), 24);

        #print("Raw prs: " , raw_pressure)
        # _pressure = (float)raw_pressure / pressure_scale
        # print("Scaled prs:", _pressure)
        return self._raw_pressure

    @property
    def temperature(self):
        """The current temperature reading in degrees C"""
        return self._raw_temperature
        # _scaled_rawtemp = (float)raw_temperature / temp_scale;
        # _temperature = _scaled_rawtemp * _c1 + _c0 / 2.0;

    @property
    def sensor_ready(self):
        """Identifies the sensorhas measurements ready"""
        return self._sensor_ready

    @property
    def mode(self):
        """An example"""
        return self._mode_bits

    @mode.setter
    def mode(self, value):
        if not Mode.is_valid(value):
            raise AttributeError("mode must be an `Mode`")

        self._mode_bits = value

    def pressure_configuration(self, rate, oversample):
        self._ratebits = rate
        self._osbits = oversample


        if oversample > Samples.COUNT_8:
            self._shiftbit = 1
        else:
            self._shiftbit = 0

        self._pressure_scale = self._oversample_scalefactor[oversample]

    @staticmethod
    def twosComplement(val, bits):
        if (val & (1 << (bits - 1))):
            val -= (1 << bits)

        return val


    def _read_calibration(self):
        pass

        # void Adafruit_DPS310::_readCalibration(void) {
        # // Wait till we're eady to read calibration
        # Adafruit_BusIO_Register MEAS_CFG = Adafruit_BusIO_Register(
        #     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_MEASCFG, 1);
        # Adafruit_BusIO_RegisterBits CALIB_RDY =
        #     Adafruit_BusIO_RegisterBits(&MEAS_CFG, 1, 7);
        # while (!CALIB_RDY.read()) {
        #     delay(1);
        # }

        # uint8_t coeffs[18];
        # for (uint8_t addr = 0; addr < 18; addr++) {
        #     Adafruit_BusIO_Register coeff = Adafruit_BusIO_Register(
        #         i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, 0x10 + addr, 1);
        #     coeffs[addr] = coeff.read();
        # }
        # _c0 = ((uint16_t)coeffs[0] << 4) | (((uint16_t)coeffs[1] >> 4) & 0x0F);
        # _c0 = twosComplement(_c0, 12);

        # _c1 = twosComplement((((uint16_t)coeffs[1] & 0x0F) << 8) | coeffs[2], 12);

        # _c00 = ((uint32_t)coeffs[3] << 12) | ((uint32_t)coeffs[4] << 4) |
        #         (((uint32_t)coeffs[5] >> 4) & 0x0F);
        # _c00 = twosComplement(_c00, 20);

        # _c10 = (((uint32_t)coeffs[5] & 0x0F) << 16) | ((uint32_t)coeffs[6] << 8) |
        #         (uint32_t)coeffs[7];
        # _c10 = twosComplement(_c10, 20);

        # _c01 = twosComplement(((uint16_t)coeffs[8] << 8) | (uint16_t)coeffs[9], 16);
        # _c11 = twosComplement(((uint16_t)coeffs[10] << 8) | (uint16_t)coeffs[11], 16);
        # _c20 = twosComplement(((uint16_t)coeffs[12] << 8) | (uint16_t)coeffs[13], 16);
        # _c21 = twosComplement(((uint16_t)coeffs[14] << 8) | (uint16_t)coeffs[15], 16);
        # _c30 = twosComplement(((uint16_t)coeffs[16] << 8) | (uint16_t)coeffs[17], 16);