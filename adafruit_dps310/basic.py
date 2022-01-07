# SPDX-FileCopyrightText: 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_dps310.basic`
================================================================================

Library for the DPS310 Precision Barometric Pressure Sensor

* Author(s): Bryan Siepert, Jose David M.

Implementation Notes
--------------------

**Hardware:**

* `Adafruit DPS310 Precision Barometric Pressure / Altitude Sensor
  <https://www.adafruit.com/product/4494>`_ (Product ID: 4494)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library:
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

* Adafruit's Register library:
  https://github.com/adafruit/Adafruit_CircuitPython_Register

"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_DPS310.git"

# Common imports; remove if unused or pylint will complain
import math
from time import sleep
from micropython import const
from adafruit_bus_device import i2c_device
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
from adafruit_register.i2c_bit import RWBit, ROBit
from adafruit_register.i2c_bits import RWBits, ROBits

_DPS310_DEFAULT_ADDRESS = const(0x77)  # DPS310 default i2c address
_DPS310_DEVICE_ID = const(0x10)  # DPS310 device identifier

_DPS310_PRSB2 = const(0x00)  # Highest byte of pressure data
_DPS310_TMPB2 = const(0x03)  # Highest byte of temperature data
_DPS310_PRSCFG = const(0x06)  # Pressure configuration
_DPS310_TMPCFG = const(0x07)  # Temperature configuration
_DPS310_MEASCFG = const(0x08)  # Sensor configuration
_DPS310_CFGREG = const(0x09)  # Interrupt/FIFO configuration
_DPS310_RESET = const(0x0C)  # Soft reset
_DPS310_PRODREVID = const(0x0D)  # Register that contains the part ID
_DPS310_TMPCOEFSRCE = const(0x28)  # Temperature calibration src


class DPS310:
    # pylint: disable=too-many-instance-attributes
    """Library for the DPS310 Precision Barometric Pressure Sensor.
    Depending on your board memory availability you could use DPS310_Advanced.

    :param ~busio.I2C i2c_bus: The I2C bus the DPS310 is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x77`

    **Quickstart: Importing and using the DPS310**

        Here is an example of using the :class:`DPS310` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from adafruit_dps310.basic import DPS310

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()   # uses board.SCL and board.SDA
            dps310 = DPS310(i2c)

        Now you have access to the :attr:`temperature` and :attr:`pressure` attributes.

        .. code-block:: python

            temperature = dps310.temperature
            pressure = dps310.pressure

    """
    # Register definitions
    _device_id = ROUnaryStruct(_DPS310_PRODREVID, ">B")
    _reset_register = UnaryStruct(_DPS310_RESET, ">B")
    _mode_bits = RWBits(3, _DPS310_MEASCFG, 0)  #

    _pressure_osbits = RWBits(4, _DPS310_PRSCFG, 0)

    _temp_osbits = RWBits(4, _DPS310_TMPCFG, 0)

    _temp_measurement_src_bit = RWBit(_DPS310_TMPCFG, 7)

    _pressure_shiftbit = RWBit(_DPS310_CFGREG, 2)
    _temp_shiftbit = RWBit(_DPS310_CFGREG, 3)

    _coefficients_ready = RWBit(_DPS310_MEASCFG, 7)
    _sensor_ready = RWBit(_DPS310_MEASCFG, 6)
    _temp_ready = RWBit(_DPS310_MEASCFG, 5)
    _pressure_ready = RWBit(_DPS310_MEASCFG, 4)

    _raw_pressure = ROBits(24, _DPS310_PRSB2, 0, 3, lsb_first=False)
    _raw_temperature = ROBits(24, _DPS310_TMPB2, 0, 3, lsb_first=False)

    _calib_coeff_temp_src_bit = ROBit(_DPS310_TMPCOEFSRCE, 7)

    _reg0e = RWBits(8, 0x0E, 0)
    _reg0f = RWBits(8, 0x0F, 0)
    _reg62 = RWBits(8, 0x62, 0)

    def __init__(self, i2c_bus, address=_DPS310_DEFAULT_ADDRESS):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        if self._device_id != _DPS310_DEVICE_ID:
            raise RuntimeError("Failed to find DPS310 - check your wiring!")
        self._pressure_scale = None
        self._temp_scale = None
        self._c0 = None
        self._c1 = None
        self._c00 = None
        self._c00 = None
        self._c10 = None
        self._c10 = None
        self._c01 = None
        self._c11 = None
        self._c20 = None
        self._c21 = None
        self._c30 = None
        self._oversample_scalefactor = (
            524288,
            1572864,
            3670016,
            7864320,
            253952,
            516096,
            1040384,
            2088960,
        )
        self._sea_level_pressure = 1013.25

        self.initialize()

    def initialize(self):
        """Initialize the sensor to continuous measurement"""

        self.reset()

        self._pressure_osbits = 6
        self._pressure_shiftbit = True
        self._pressure_scale = self._oversample_scalefactor[6]

        self._temp_osbits = 6
        self._temp_scale = self._oversample_scalefactor[6]
        self._temp_shiftbit = True

        self._mode_bits = 7

        # wait until we have at least one good measurement
        self.wait_temperature_ready()
        self.wait_pressure_ready()

    # (https://github.com/Infineon/DPS310-Pressure-Sensor#temperature-measurement-issue)
    # similar to DpsClass::correctTemp(void) from infineon's c++ library
    def _correct_temp(self):
        """Correct temperature readings on ICs with a fuse bit problem"""
        self._reg0e = 0xA5
        self._reg0f = 0x96
        self._reg62 = 0x02
        self._reg0e = 0
        self._reg0f = 0

        # perform a temperature measurement
        # the most recent temperature will be saved internally
        # and used for compensation when calculating pressure
        _unused = self._raw_temperature

    def reset(self):
        """Reset the sensor"""
        self._reset_register = 0x89
        # wait for hardware reset to finish
        sleep(0.010)
        while not self._sensor_ready:
            sleep(0.001)
        self._correct_temp()
        self._read_calibration()
        # make sure we're using the temperature source used for calibration
        self._temp_measurement_src_bit = self._calib_coeff_temp_src_bit

    @property
    def pressure(self):
        """Returns the current pressure reading in kPA"""

        temp_reading = self._raw_temperature
        raw_temperature = self._twos_complement(temp_reading, 24)
        pressure_reading = self._raw_pressure
        raw_pressure = self._twos_complement(pressure_reading, 24)
        _scaled_rawtemp = raw_temperature / self._temp_scale

        _temperature = _scaled_rawtemp * self._c1 + self._c0 / 2.0

        p_red = raw_pressure / self._pressure_scale

        pres_calc = (
            self._c00
            + p_red * (self._c10 + p_red * (self._c20 + p_red * self._c30))
            + _scaled_rawtemp * (self._c01 + p_red * (self._c11 + p_red * self._c21))
        )

        final_pressure = pres_calc / 100
        return final_pressure

    @property
    def altitude(self):
        """The altitude based on the sea level pressure (:attr:`sea_level_pressure`) -
        which you must enter ahead of time)"""
        return 44330 * (
            1.0 - math.pow(self.pressure / self._sea_level_pressure, 0.1903)
        )

    @property
    def temperature(self):
        """The current temperature reading in degrees Celsius"""
        _scaled_rawtemp = self._raw_temperature / self._temp_scale
        _temperature = _scaled_rawtemp * self._c1 + self._c0 / 2.0
        return _temperature

    @property
    def sea_level_pressure(self):
        """The local sea level pressure in hectoPascals (aka millibars). This is used
        for calculation of :attr:`altitude`. Values are typically in the range
        980 - 1030."""
        return self._sea_level_pressure

    @sea_level_pressure.setter
    def sea_level_pressure(self, value):
        self._sea_level_pressure = value

    def wait_temperature_ready(self):
        """Wait until a temperature measurement is available."""

        while self._temp_ready is False:
            sleep(0.001)

    def wait_pressure_ready(self):
        """Wait until a pressure measurement is available"""

        while self._pressure_ready is False:
            sleep(0.001)

    @staticmethod
    def _twos_complement(val, bits):
        if val & (1 << (bits - 1)):
            val -= 1 << bits

        return val

    def _read_calibration(self):

        while not self._coefficients_ready:
            sleep(0.001)

        buffer = bytearray(19)
        coeffs = [None] * 18
        for offset in range(18):
            buffer = bytearray(2)
            buffer[0] = 0x10 + offset

            with self.i2c_device as i2c:

                i2c.write_then_readinto(buffer, buffer, out_end=1, in_start=1)

                coeffs[offset] = buffer[1]

        self._c0 = (coeffs[0] << 4) | ((coeffs[1] >> 4) & 0x0F)
        self._c0 = self._twos_complement(self._c0, 12)

        self._c1 = self._twos_complement(((coeffs[1] & 0x0F) << 8) | coeffs[2], 12)

        self._c00 = (coeffs[3] << 12) | (coeffs[4] << 4) | ((coeffs[5] >> 4) & 0x0F)
        self._c00 = self._twos_complement(self._c00, 20)

        self._c10 = ((coeffs[5] & 0x0F) << 16) | (coeffs[6] << 8) | coeffs[7]
        self._c10 = self._twos_complement(self._c10, 20)

        self._c01 = self._twos_complement((coeffs[8] << 8) | coeffs[9], 16)
        self._c11 = self._twos_complement((coeffs[10] << 8) | coeffs[11], 16)
        self._c20 = self._twos_complement((coeffs[12] << 8) | coeffs[13], 16)
        self._c21 = self._twos_complement((coeffs[14] << 8) | coeffs[15], 16)
        self._c30 = self._twos_complement((coeffs[16] << 8) | coeffs[17], 16)
