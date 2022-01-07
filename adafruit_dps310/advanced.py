# SPDX-FileCopyrightText: 2020 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_dps310.advanced`
================================================================================

Library for the DPS310 Precision Barometric Pressure Sensor. This is the advanced
version. Includes some features not present in `adafruit_dps310.basic`

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

from time import sleep
from micropython import const
from adafruit_register.i2c_bits import RWBits
from adafruit_dps310.basic import DPS310

# pylint: disable=no-member,unnecessary-pass


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
    """Options for ``mode``

    +--------------------------+------------------------------------------------------------------+
    | Mode                     | Description                                                      |
    +--------------------------+------------------------------------------------------------------+
    | ``Mode.IDLE``            | Puts the sensor into a shutdown state                            |
    +--------------------------+------------------------------------------------------------------+
    | ``Mode.ONE_PRESSURE``    | Setting `mode` to ``Mode.ONE_PRESSURE`` takes a single pressure  |
    |                          | measurement then switches to ``Mode.IDLE``                       |
    +--------------------------+------------------------------------------------------------------+
    | ``Mode.ONE_TEMPERATURE`` | Setting `mode` to ``Mode.ONE_TEMPERATURE`` takes a single        |
    |                          | temperature measurement then switches to ``Mode.IDLE``           |
    +--------------------------+------------------------------------------------------------------+
    | ``Mode.CONT_PRESSURE``   | Take pressure measurements at the current `pressure_rate`.       |
    |                          | `temperature` will not be updated                                |
    +--------------------------+------------------------------------------------------------------+
    | ``Mode.CONT_TEMP``       | Take temperature measurements at the current `temperature_rate`. |
    |                          | `pressure` will not be updated                                   |
    +--------------------------+------------------------------------------------------------------+
    | ``Mode.CONT_PRESTEMP``   | Take temperature and pressure measurements at the current        |
    |                          | `pressure_rate` and `temperature_rate`                           |
    +--------------------------+------------------------------------------------------------------+

    """

    pass  # pylint: disable=unnecessary-pass


Mode.add_values(
    (
        ("IDLE", 0, "Idle", None),
        ("ONE_PRESSURE", 1, "One-Shot Pressure", None),
        ("ONE_TEMPERATURE", 2, "One-Shot Temperature", None),
        ("CONT_PRESSURE", 5, "Continuous Pressure", None),
        ("CONT_TEMP", 6, "Continuous Temperature", None),
        ("CONT_PRESTEMP", 7, "Continuous Pressure & Temperature", None),
    )
)


class Rate(CV):
    """Options for :attr:`pressure_rate` and :attr:`temperature_rate`"""

    pass


Rate.add_values(
    (
        ("RATE_1_HZ", 0, 1, None),
        ("RATE_2_HZ", 1, 2, None),
        ("RATE_4_HZ", 2, 4, None),
        ("RATE_8_HZ", 3, 8, None),
        ("RATE_16_HZ", 4, 16, None),
        ("RATE_32_HZ", 5, 32, None),
        ("RATE_64_HZ", 6, 64, None),
        ("RATE_128_HZ", 7, 128, None),
    )
)


class SampleCount(CV):
    """Options for :attr:`temperature_oversample_count` and :attr:`pressure_oversample_count`"""

    pass


SampleCount.add_values(
    (
        ("COUNT_1", 0, 1, None),
        ("COUNT_2", 1, 2, None),
        ("COUNT_4", 2, 4, None),
        ("COUNT_8", 3, 8, None),
        ("COUNT_16", 4, 16, None),
        ("COUNT_32", 5, 32, None),
        ("COUNT_64", 6, 64, None),
        ("COUNT_128", 7, 128, None),
    )
)
# pylint: enable=unnecessary-pass

_DPS310_DEFAULT_ADDRESS = const(0x77)  # DPS310 default i2c address
# _DPS310_DEVICE_ID = const(0x10)  # DPS310 device identifier

# _DPS310_PRSB2 = const(0x00)  # Highest byte of pressure data
# _DPS310_TMPB2 = const(0x03)  # Highest byte of temperature data
_DPS310_PRSCFG = const(0x06)  # Pressure configuration
_DPS310_TMPCFG = const(0x07)  # Temperature configuration
# _DPS310_MEASCFG = const(0x08)  # Sensor configuration
# _DPS310_CFGREG = const(0x09)  # Interrupt/FIFO configuration
# _DPS310_RESET = const(0x0C)  # Soft reset
# _DPS310_PRODREVID = const(0x0D)  # Register that contains the part ID
# _DPS310_TMPCOEFSRCE = const(0x28)  # Temperature calibration src


class DPS310_Advanced(DPS310):
    # pylint: disable=too-many-instance-attributes
    """Library for the DPS310 Precision Barometric Pressure Sensor.
    This class contains some of other configurable features

    :param ~busio.I2C i2c_bus: The I2C bus the DPS310 is connected to.
    :param int address: The I2C device address. Defaults to :const:`0x77`

    **Quickstart: Importing and using the DPS310**

        Here is an example of using the :class:`DPS310_Advanced` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            from adafruit_dps310.dps310_advanced import DPS310_Advanced as DPS310

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
    _pressure_ratebits = RWBits(3, _DPS310_PRSCFG, 4)
    _temp_ratebits = RWBits(3, _DPS310_TMPCFG, 4)

    def __init__(self, i2c_bus, address=_DPS310_DEFAULT_ADDRESS):
        super().__init__(i2c_bus, _DPS310_DEFAULT_ADDRESS)

    def initialize(self):
        """Initialize the sensor to continuous measurement"""

        self.reset()

        self.pressure_rate = Rate.RATE_64_HZ
        self.pressure_oversample_count = SampleCount.COUNT_64
        self.temperature_rate = Rate.RATE_64_HZ
        self.temperature_oversample_count = SampleCount.COUNT_64
        self.mode = Mode.CONT_PRESTEMP

        # wait until we have at least one good measurement
        self.wait_temperature_ready()
        self.wait_pressure_ready()

    @property
    def temperature_ready(self):
        """Returns true if there is a temperature reading ready"""
        return self._temp_ready

    def wait_temperature_ready(self):
        """Wait until a temperature measurement is available.

        To avoid waiting indefinitely this function raises an
        error if the sensor isn't configured for temperate measurements,
        ie. ``Mode.ONE_TEMPERATURE``, ``Mode.CONT_TEMP`` or ``Mode.CONT_PRESTEMP``.
        See the `Mode` documentation for details.
        """
        if self._mode_bits in (Mode.IDLE, Mode.ONE_PRESSURE, Mode.CONT_PRESSURE):
            raise RuntimeError(
                "Sensor mode is set to idle or pressure measurement,\
                    can't wait for a temperature measurement"
            )
        while self._temp_ready is False:
            sleep(0.001)

    @property
    def pressure_ready(self):
        """Returns true if pressure readings are ready"""
        return self._pressure_ready

    def wait_pressure_ready(self):
        """Wait until a pressure measurement is available

        To avoid waiting indefinitely this function raises an
        error if the sensor isn't configured for pressure measurements,
        ie.  ``Mode.ONE_PRESSURE``, ``Mode.CONT_PRESSURE`` or ``Mode.CONT_PRESTEMP``
        See the `Mode` documentation for details.
        """
        if self._mode_bits in (Mode.IDLE, Mode.ONE_TEMPERATURE, Mode.CONT_TEMP):
            raise RuntimeError(
                "Sensor mode is set to idle or temperature measurement,\
                    can't wait for a pressure measurement"
            )
        while self._pressure_ready is False:
            sleep(0.001)

    @property
    def mode(self):
        """The measurement mode. Must be a `Mode`. See the `Mode` documentation for details"""
        return self._mode_bits

    @mode.setter
    def mode(self, value):
        if not Mode.is_valid(value):
            raise AttributeError("mode must be an `Mode`")

        self._mode_bits = value

    @property
    def pressure_rate(self):
        """Configure the pressure measurement rate. Must be a `Rate`"""
        return self._pressure_ratebits

    @pressure_rate.setter
    def pressure_rate(self, value):
        if not Rate.is_valid(value):
            raise AttributeError("pressure_rate must be a Rate")
        self._pressure_ratebits = value

    @property
    def pressure_oversample_count(self):
        """The number of samples taken per pressure measurement. Must be a ``SampleCount``"""
        return self._pressure_osbits

    @pressure_oversample_count.setter
    def pressure_oversample_count(self, value):
        if not SampleCount.is_valid(value):
            raise AttributeError("pressure_oversample_count must be a SampleCount")

        self._pressure_osbits = value
        self._pressure_shiftbit = value > SampleCount.COUNT_8
        self._pressure_scale = self._oversample_scalefactor[value]

    @property
    def temperature_rate(self):
        """Configure the temperature measurement rate. Must be a `Rate`"""
        return self._temp_ratebits

    @temperature_rate.setter
    def temperature_rate(self, value):
        if not Rate.is_valid(value):
            raise AttributeError("temperature_rate must be a Rate")
        self._temp_ratebits = value

    @property
    def temperature_oversample_count(self):
        """The number of samples taken per temperature measurement. Must be a ``SampleCount``"""
        return self._temp_osbits

    @temperature_oversample_count.setter
    def temperature_oversample_count(self, value):
        if not SampleCount.is_valid(value):
            raise AttributeError("temperature_oversample_count must be a SampleCount")

        self._temp_osbits = value
        self._temp_scale = self._oversample_scalefactor[value]
        self._temp_shiftbit = value > SampleCount.COUNT_8
