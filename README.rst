Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-dsp310/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/dps310/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_DPS310/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_DPS310/actions
    :alt: Build Status

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

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


Installing to a connected CircuitPython Device
==============================================
Some devices, eg. the QT-PY, are very limited in memory. The DPS310 library contains
two variants - basic and advanced - which give different levels of functionality.

Installing the DPS310 library could have the following outcomes:

    * It installs successfully and your code runs successfully. Woo-hoo! Continue with
      your amazing project.
    * It installs successfully and your code fails to run with a memory allocation
      error. Try one of the following:

        * If your ``code.py`` is large, especially if it has lots of comments, you
          can shrink it into a ``.mpy`` file instead. See the Adafruit
          `Learn Guide <https://learn.adafruit.com/Memory-saving-tips-for-CircuitPython/non-volatile-not-enough-disk-space>`_
          on shrinking your code.
        * Only use the basic DPS310 implementation, and remove the following file:
          ``<CIRCUITPY>/lib/adafruit_dps310/advanced.mpy`` where <CIRCUITPY> is the
          mounted location of your device. Make sure that your code only uses the basic
          implementation.


Usage Example
=============

.. code-block:: python3

    import time
    import board
    from adafruit_dps310.basic import DPS310

    i2c = board.I2C()   # uses board.SCL and board.SDA

    dps310 = DPS310(i2c)

    while True:
        print("Temperature = %.2f *C"%dps310.temperature)
        print("Pressure = %.2f hPa"%dps310.pressure)
        print("")
        time.sleep(1.0)


Known Issues
============
Library extensive features might not be compatible with memory limited boards. For these kind of
boards you need to use the ``adafruit_dps310/basic.mpy``, the file needs to be in the ``mpy``
format in order to fit in memory.
For boards with more memory available you could use the code present
in ``adafruit_dps310/advanced.py``. For usage refer to ``dps310_simpletest_advanced.py``


Documentation
=============

API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/dps310/en/latest/>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_DPS310/blob/main/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
