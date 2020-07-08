Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-dsp310/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/dps310/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_DPS310/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_DPS310/actions
    :alt: Build Status

Library for the DPS310 Precision Barometric Pressure Sensor


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_.

Installing from PyPI
=====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-dps310/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-dps310

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-dps310

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-dps310

Usage Example
=============

.. code-block:: python3

    import time
    import board
    import busio
    import adafruit_dps310

    i2c = busio.I2C(board.SCL, board.SDA)

    dps310 = adafruit_dps310.DPS310(i2c)

    while True:
        print("Temperature = %.2f *C"%dps310.temperature)
        print("Pressure = %.2f hPa"%dps310.pressure)
        print("")
        time.sleep(1.0)

Caveat: by default the library initializes the IC with constant temperature and pressure measurements at 64Hz with 64 samples. It is not possible to change the IC's mode, temperature_oversample_count or pressure_oversample_count on-the-fly so resetting the IC and operation parameteres is required. For instance, to set the mode to continuous pressure measurement at 1Hz with 2 samples:

.. code-block:: python3

    dps310.reset()
    dps310.pressure_oversample_count = adafruit_dps310.SampleCount.COUNT_2
    dps310.pressure_rate = adafruit_dps310.Rate.RATE_1_HZ
    dps310.mode = adafruit_dps310.Mode.CONT_PRESSURE
    dps310.wait_pressure_ready()


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_DPS310/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
