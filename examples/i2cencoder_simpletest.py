# Simple test for an RGB encoder.

# Author: Ben Shockley

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
    encoder.writeRGBCode(0x00FF00)
    valBytes = struct.unpack('>i', encoder.readCounter32())
    print ('Changed: {}'.format(valBytes[0]))

def EncoderPush():
    encoder.writeRGBCode(0x0000FF)
    print ('Encoder Pushed!')

def EncoderRelease():
    encoder.writeRGBCode(0x00FFFF)
    print ('Encoder Released!')

def EncoderDoublePush():
    encoder.writeRGBCode(0xFF00FF)
    print ('Encoder Double Push!')

def EncoderMax():
    encoder.writeRGBCode(0xFF0000)
    print ('Encoder max!')

def EncoderMin():
    encoder.writeRGBCode(0xFF0000)
    print ('Encoder min!')

def EncoderFade():
    encoder.writeRGBCode(0x000000)

def Encoder_INT():
    encoder.update_status()

# Start by resetting the encoder. Reset takes 400us , so let us give it time to settle.
encoder.reset()
time.sleep(.1)

# When the board was initialized, the default config was loaded.
# Here we can override that config if we want.
encconfig = (i2cencoderlibv21.INT_DATA | i2cencoderlibv21.WRAP_DISABLE,
             | i2cencoderlibv21.DIRE_RIGHT | i2cencoderlibv21.IPUP_ENABLE,
             | i2cencoderlibv21.RMOD_X1 | i2cencoderlibv21.RGB_ENCODER)
encoder.begin(encconfig)

# Setup other varibles
encoder.writeCounter(0)
encoder.writeMax(10)
encoder.writeMin(-10)
encoder.writeStep(1)
encoder.writeAntiBouncePeriod(25)
encoder.writeDoublePushPeriod(50)
encoder.writeFadeRGB(2)

# Declare callbacks
encoder.onChange = EncoderChange
encoder.onButtonRelease = EncoderRelease
encoder.onButtonPush = EncoderPush
encoder.onButtonDoublePush = EncoderDoublePush
encoder.onMax = EncoderMax
encoder.onMin = EncoderMin
encoder.onFadeProcess = EncoderFade

# Autoconfigure the interrupt register according to the callbacks declared.
encoder.autoconfigInterrupt()

while True:
    if not INT.value:       #If INT pin goes LOW - we know the encoder status changed.
        Encoder_INT(encoder)
