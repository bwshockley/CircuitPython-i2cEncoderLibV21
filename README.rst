Introduction
============

.. image:: https://readthedocs.org/projects/circuitpython-i2cencoderlibv21/badge/?version=latest
    :target: https://circuitpython-i2cencoderlibv21.readthedocs.io/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://travis-ci.com/bwshockley/CircuitPython-i2cEncoderLibV21.svg?branch=master
    :target: https://travis-ci.com/bwshockley/CircuitPython-i2cEncoderLibV21
    :alt: Build Status

This is a CircuitPython helper library for the i2c Encoder made by Simon Caron: http://www.duppa.net/i2c-encoder-v2-1/


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_.

Installing from PyPI
=====================
.. note:: This library is not available on PyPI yet. Install documentation is included
   as a standard element. Stay tuned for PyPI availability!

.. todo:: Remove the above note if PyPI version is/will be available at time of release.
   If the library is not planned for PyPI, remove the entire 'Installing from PyPI' section.

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-i2cencoderlibv21/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-i2cencoderlibv21

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-i2cencoderlibv21

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-i2cencoderlibv21

Usage Example
=============

.. code-block:: shell

    # imports
    import time
    import struct
    import busio
    import board
    import digitalio
    import i2cencoderlibv21

    # Setup the Inturrpt Pin from the encoder.
    INT = digitalio.DigitalInOut(board.A3)
    INT.direction = digitalio.Direction.INPUT
    INT.pull = digitalio.Pull.UP

    # Initialize the device.
    i2c = busio.I2C(board.SCL, board.SDA)
    encoder = i2cencoderlibv21.I2CEncoderLibV21(i2c, 0x21)

    def EncoderChange():
        encoder.write_rgb_code(0x00FF00)
        valBytes = struct.unpack('>i', encoder.readCounter32())
        print ('Changed: {}'.format(valBytes[0]))

    def EncoderIncrement():
        print ('Incrementing...')

    def EncoderPush():
        encoder.write_rgb_code(0x0000FF)
        print ('Encoder Pushed!')

    def EncoderRelease():
        encoder.write_rgb_code(0x00FFFF)
        print ('Encoder Released!')

    def EncoderDoublePush():
        encoder.write_rgb_code(0xFF00FF)
        print ('Encoder Double Push!')

    def EncoderMax():
        encoder.write_rgb_code(0xFF0000)
        print ('Encoder max!')

    def EncoderMin():
        encoder.write_rgb_code(0xFF0000)
        print ('Encoder min!')

    def EncoderFade():
        encoder.write_rgb_code(0x000000)

    def Encoder_INT():
        encoder.update_status()

    # Start by resetting the encoder. Reset takes 400us , so let us give it time to settle.
    encoder.reset()
    time.sleep(.1)

    # When the board was initialized, the default config was loaded.
    # Here we can override that config if we want.
    encconfig = (i2cencoderlibv21.INT_DATA | i2cencoderlibv21.WRAP_DISABLE
                 | i2cencoderlibv21.DIRE_RIGHT | i2cencoderlibv21.IPUP_DISABLE
                 | i2cencoderlibv21.RMOD_X1 | i2cencoderlibv21.RGB_ENCODER)
    encoder.begin(encconfig)

    # Setup other varibles
    encoder.write_counter(0)
    encoder.write_max(10)
    encoder.write_min(-10)
    encoder.write_step_size(1)
    encoder.write_antibounce_period(25)
    encoder.write_double_push_period(50)
    encoder.write_fade_rgb(5)

    # Declare callbacks
    encoder.onChange = EncoderChange
    encoder.onIncrement = EncoderIncrement
    encoder.onButtonRelease = EncoderRelease
    encoder.onButtonPush = EncoderPush
    encoder.onButtonDoublePush = EncoderDoublePush
    encoder.onMax = EncoderMax
    encoder.onMin = EncoderMin
    encoder.onFadeProcess = EncoderFade

    # Autoconfigure the interrupt register according to the callbacks declared.
    # Must be called after declaring callbacks.
    encoder.autoconfig_interrupt()

    while True:
        if not INT.value:       #If INT pin goes LOW - we know the encoder status changed.
            Encoder_INT()

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/bwshockley/CircuitPython_i2cEncoderLibV21/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.
