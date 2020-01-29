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
from adafruit_register.i2c_bit import RWBit, ROBit
from adafruit_register.i2c_bits import RWBits, ROBits

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

    _pressure_ratebits = RWBits(3, _DPS310_PRSCFG, 4)
    _pressure_osbits = RWBits(4, _DPS310_PRSCFG, 0)


    _temp_ratebits = RWBits(3, _DPS310_TMPCFG, 4)
    _temp_osbits = RWBits(4, _DPS310_TMPCFG, 0)

    _shiftbit = RWBit(_DPS310_CFGREG, 2)
    #   Adafruit_BusIO_Register CFG_REG = Adafruit_BusIO_Register(
    #       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_CFGREG, 1);
    #   Adafruit_BusIO_RegisterBits shiftbit =
    #       Adafruit_BusIO_RegisterBits(&CFG_REG, 1, 2);
    #COEF_RDY SENSOR_RDY TMP_RDY PRS_RDY
    _coefficients_ready = RWBit(_DPS310_MEASCFG, 7)
    _sensor_ready = RWBit(_DPS310_MEASCFG, 6)
    _temp_ready = RWBit(_DPS310_MEASCFG, 5)
    _pressure_ready = RWBit(_DPS310_MEASCFG, 4)
    #     Adafruit_BusIO_Register MEAS_CFG = Adafruit_BusIO_Register(
    #       i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_MEASCFG, 1);
    #   Adafruit_BusIO_RegisterBits SENSOR_RDY =
    #       Adafruit_BusIO_RegisterBits(&MEAS_CFG, 1, 6);

    _raw_pressure = ROBits(24, _DPS310_PRSB2, 0, 3)
    _raw_pressure2 = ROUnaryStruct(_DPS310_PRSB2, ">b")
    _raw_pressure3 = Struct(_DPS310_PRSB2, ">b")
    #  Adafruit_BusIO_Register PRS_B2 = Adafruit_BusIO_Register(
    #   i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_PRSB2, 3, MSBFIRST);

    _raw_temperature = ROBits(24, _DPS310_TMPB2, 0, 3)

    #       Adafruit_BusIO_Register TMP_B2 = Adafruit_BusIO_Register(
    #   i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_TMPB2, 3, MSBFIRST);

    _coeff_temp_src_bit = ROBit(_DPS310_TMPCOEFSRCE, 7)
    # Adafruit_BusIO_Register TMP_COEFF = Adafruit_BusIO_Register(
    #     i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, DPS310_TMPCOEFSRCE, 1);
    # Adafruit_BusIO_RegisterBits srcbit =
    #     Adafruit_BusIO_RegisterBits(&TMP_COEFF, 1, 7);

    _temp_src_bit = RWBit(_DPS310_TMPCFG, 7)
    # // the bit is in another register
    # Adafruit_BusIO_RegisterBits extbit =
    #     Adafruit_BusIO_RegisterBits(&TMP_CFG, 1, 7);

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
        sleep(0.010)
        print("reset finished")
        self._read_calibration()
        self._pressure_scale = None
        self.pressure_configuration(Rate.RATE_64_HZ, Samples.COUNT_64)
        self.temperature_configuration(Rate.RATE_64_HZ, Samples.COUNT_64)

        self.mode = Mode.CONT_PRESTEMP

        # wait until we have at least one good measurement
        while (self._temp_ready is False) and (self._pressure_ready is False):
            sleep(0.001)

        print("mode:", self.mode)
        self._pressure_scale = 1
    def reset(self):
        """Perform a soft-reset on the sensor"""
        self._reset = 0x89
        while not self._sensor_ready:
            sleep(0.001)

    @property
    def pressure(self):
        """Returns the current pressure reading in kPA"""

        # buf = bytearray(buffer_list)

    


        # with obj.i2c_device as i2c:
        #     i2c.write_then_readinto(self.buffer, self.buffer, out_end=1, in_start=1)

        # with self.i2c_device as i2c:
        #     i2c.readinto(buf)


        raw_temperature = self.twosComplement(self._raw_temperature, 24)
        raw_pressure = self.twosComplement(self._raw_pressure, 24)

        _scaled_rawtemp = raw_temperature / self.temp_scale
        _temperature = _scaled_rawtemp * _c1 + _c0 / 2.0

        p_red = raw_pressure / self._pressure_scale
        pres_calc = self._c00 + p_red * (self._c10 + p_red * (self._c20 + p_red * self._c30)) +     _scaled_rawtemp * (self._c01 + p_red * (self._c11 + p_red * self._c21))

        final_pressure = pres_calc / 100
        return final_pressure

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
        """Configure the pressure rate and oversample count"""
        self._pressure_ratebits = rate
        self._pressure_osbits = oversample

        self._shiftbit = (oversample > Samples.COUNT_8)

        self._pressure_scale = self._oversample_scalefactor[oversample]

    def temperature_configuration(self, rate, oversample):
        """Configure the temperature rate and oversample count"""

        self._temp_ratebits = rate
        self._temp_osbits = oversample
        self._shiftbit = (oversample > Samples.COUNT_8)

        self._tmp_src_bit = self._coeff_temp_src_bit

    def twosComplement(self, val, bits):
        if (val & (1 << (bits - 1))):
            val -= (1 << bits)

        return val


    def _read_calibration(self):

        while not self._coefficients_ready:
            sleep(0.001)

        buffer = bytearray(19) # addr + reads
        coeffs = []
        for addr in range(18):
            buffer = bytearray(2) # addr + reads
            buffer[0] = 0x10 + addr

            with self.i2c_device as i2c:
                i2c.write_then_readinto(buffer, buffer, out_end=1, in_start=1)
                coeffs[addr] = buffer[1]

        self._c0 = (coeffs[0] << 4) | ((coeffs[1] >> 4) & 0x0F)
        self._c0 = twosComplement(_c0, 12)

        self._c1 = twosComplement(((coeffs[1] & 0x0F) << 8) | coeffs[2], 12)

        self._c00 = (coeffs[3] << 12) | (coeffs[4] << 4) | ((coeffs[5] >> 4) & 0x0F)
        self._c00 = twosComplement(_c00, 20)

        self._c10 = ((coeffs[5] & 0x0F) << 16) | (coeffs[6] << 8) |coeffs[7]
        self._c10 = twosComplement(_c10, 20)

        self._c01 = twosComplement((coeffs[8] << 8) | coeffs[9], 16)
        self._c11 = twosComplement((coeffs[10] << 8) | coeffs[11], 16)
        self._c20 = twosComplement((coeffs[12] << 8) | coeffs[13], 16)
        self._c21 = twosComplement((coeffs[14] << 8) | coeffs[15], 16)
        self._c30 = twosComplement((coeffs[16] << 8) | coeffs[17], 16)

        print("c0 = ", _c0)
        print("c1 = ", _c1)
        print("c00 = ", _c00)
        print("c10 = ", _c10)
        print("c01 = ", _c01)
        print("c11 = ", _c11)
        print("c20 = ", _c20)
        print("c21 = ", _c21)
        print("c30 = ", _c30)