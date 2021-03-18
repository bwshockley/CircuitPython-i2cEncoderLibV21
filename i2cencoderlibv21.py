# The MIT License (MIT)
#
# Copyright (c) 2018 ladyada for adafruit industries
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
`i2cencoderv21`
====================================================
A CircuitPython library for the I2CEncoder v2.1 board from Simon Caron.

* Author(s): Ben Shockley & Simon Caron

Implementation Notes
--------------------
**Hardware:**
**Software and Dependencies:**
* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""
# imports
import struct
from adafruit_bus_device.i2c_device import I2CDevice


__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/bwshockley/CircuitPython-i2cEncoderLibV21.git"


# pylint: disable=bad-whitespace

# Encoder register addresses.
REG_GCONF    = 0x00
REG_GP1CONF  = 0x01
REG_GP2CONF  = 0x02
REG_GP3CONF  = 0x03
REG_INTCONF  = 0x04
REG_ESTATUS  = 0x05
REG_I2STATUS = 0x06
REG_FSTATUS  = 0x07
REG_CVALB4   = 0x08
REG_CVALB3   = 0x09
REG_CVALB2   = 0x0A
REG_CVALB1   = 0x0B
REG_CMAXB4   = 0x0C
REG_CMAXB3   = 0x0D
REG_CMAXB2   = 0x0E
REG_CMAXB1   = 0x0F
REG_CMINB4   = 0x10
REG_CMINB3   = 0x11
REG_CMINB2   = 0x12
REG_CMINB1   = 0x13
REG_ISTEPB4  = 0x14
REG_ISTEPB3  = 0x15
REG_ISTEPB2  = 0x16
REG_ISTEPB1  = 0x17
REG_RLED     = 0x18
REG_GLED     = 0x19
REG_BLED     = 0x1A
REG_GP1REG   = 0x1B
REG_GP2REG   = 0x1C
REG_GP3REG   = 0x1D
REG_ANTBOUNC = 0x1E
REG_DPPERIOD = 0x1F
REG_FADERGB  = 0x20
REG_FADEGP   = 0x21
REG_GAMRLED  = 0x27
REG_GAMGLED  = 0x28
REG_GAMBLED  = 0x29
REG_GAMMAGP1 = 0x2A
REG_GAMMAGP2 = 0x2B
REG_GAMMAGP3 = 0x2C
REG_GCONF2   = 0x30
REG_IDCODE   = 0x70
REG_VERSION  = 0x71
REG_EEPROMS  = 0x80

# Encoder configuration bit. Use with GCONF #
FLOAT_DATA = 0x0001
INT_DATA = 0x0000
WRAP_ENABLE = 0x0002
WRAP_DISABLE = 0x0000
DIRE_LEFT = 0x0004
DIRE_RIGHT = 0x0000
IPUP_DISABLE = 0x0008
IPUP_ENABLE = 0x0000
RMOD_X2 = 0x0010
RMOD_X1 = 0x0000
RGB_ENCODER = 0x0020
STD_ENCODER = 0x0000
EEPROM_BANK1 = 0x0040
EEPROM_BANK2 = 0x0000
RESET = 0x0080
CLK_STRECH_ENABLE = 0x0100
CLK_STRECH_DISABLE = 0x0000
REL_MODE_ENABLE = 0x0200
REL_MODE_DISABLE = 0x0000

CONFIG_DEFAULT = (INT_DATA | WRAP_DISABLE | DIRE_RIGHT
                  | IPUP_ENABLE | RMOD_X1 | RGB_ENCODER)

# Encoder status bits and setting.
# Use with: INTCONF for set and with ESTATUS for read the bits  #
PUSHR = 0x01
PUSHP = 0x02
PUSHD = 0x04
RINC = 0x08
RDEC = 0x10
RMAX = 0x20
RMIN = 0x40
INT_2 = 0x80

# Encoder Int2 bits. Use to read the bits of I2STATUS  #
GP1_POS = 0x01
GP1_NEG = 0x02
GP2_POS = 0x04
GP2_NEG = 0x08
GP3_POS = 0x10
GP3_NEG = 0x20
FADE_INT = 0x40

# Encoder Fade status bits. Use to read the bits of FSTATUS  #
FADE_R = 0x01
FADE_G = 0x02
FADE_B = 0x04
FADE_GP1 = 0x08
FADE_GP2 = 0x10
FADE_GP3 = 0x20

# GPIO Configuration. USe with GP1CONF,GP2CONF,GP3CONF #
GP_PWM = 0x00
GP_OUT = 0x01
GP_AN = 0x02
GP_IN = 0x03

GP_PULL_EN = 0x04
GP_PULL_DI = 0x00

GP_INT_DI = 0x00
GP_INT_PE = 0x08
GP_INT_NE = 0x10
GP_INT_BE = 0x18

# Gamma configuration #

GAMMA_OFF = 0
GAMMA_1 = 1
GAMMA_1_8 = 2
GAMMA_2 = 3
GAMMA_2_2 = 4
GAMMA_2_4 = 5
GAMMA_2_6 = 6
GAMMA_2_8 = 7

class I2CEncoderLibV21:
    """Helper library for the i2c Encoder from Simon Caron
         http://www.duppa.net/i2c-encoder-v2-1/

        :param ~busio.I2C i2c_bus: The I2C bus the encoder is connected to.
        :param byte address: The I2C slave address of the encoder
        :param byte config: Initial setup config.  Default is CONFIG_DEFAULT
    """
    onButtonRelease = None
    onButtonPush = None
    onButtonDoublePush = None
    onIncrement = None
    onDecrement = None
    onChange = None
    onMax = None
    onMin = None
    onMinMax = None
    onGP1Rise = None
    onGP1Fall = None
    onGP2Rise = None
    onGP2Fall = None
    onGP3Rise = None
    onGP3Fall = None
    onFadeProcess = None

    stat = 0
    stat2 = 0
    gconf = 0

    def __init__(self, i2c_bus, address):
        """Initialization of the encoder."""
        self.i2c_device = I2CDevice(i2c_bus, address)

    def begin(self, config=CONFIG_DEFAULT):
        """
        Set the encoder's general configuration.

        :param 1 or 2-byte config: Configuration to be set.
        """
        self._write_reg(REG_GCONF, config)
        self._write_reg(REG_GCONF2, config >> 8)
        self.gconf = config

    def reset(self):
        """Reset the encoder."""
        self._write_reg(REG_GCONF, RESET)

    def _event_caller(self, event) :
        """ Call che attached callaback if it is defined."""
        self._read_reg(REG_GCONF)
        if event:
            event()

    # Return true if the status of the encoder changed, otherwise return false #
    def update_status(self) :
        """Runs the callback for the encoder status."""
        self.stat = self._read_reg(REG_ESTATUS)[0]

        if self.stat == 0:
            self.stat2 = 0
            return False

        if (self.stat & PUSHR) != 0 :
            self._event_caller (self.onButtonRelease)

        if (self.stat & PUSHP) != 0 :
            self._event_caller (self.onButtonPush)

        if (self.stat & PUSHD) != 0 :
            self._event_caller (self.onButtonDoublePush)

        if (self.stat & RINC) != 0 :
            self._event_caller (self.onIncrement)
            self._event_caller (self.onChange)

        if (self.stat & RDEC) != 0 :
            self._event_caller (self.onDecrement)
            self._event_caller (self.onChange)

        if (self.stat & RMAX) != 0 :
            self._event_caller (self.onMax)
            self._event_caller (self.onMinMax)

        if (self.stat & RMIN) != 0 :
            self._event_caller (self.onMin)
            self._event_caller (self.onMinMax)

        if (self.stat & INT_2) != 0 :
            self.stat2 = self._read_reg(REG_I2STATUS)[0]

            if self.stat2 == 0 :
                return True

            if (self.stat2 & GP1_POS) != 0 :
                self._event_caller (self.onGP1Rise)

            if (self.stat2 & GP1_NEG) != 0 :
                self._event_caller (self.onGP1Fall)

            if (self.stat2 & GP2_POS) != 0 :
                self._event_caller (self.onGP2Rise)

            if (self.stat2 & GP2_NEG) != 0 :
                self._event_caller (self.onGP2Fall)

            if (self.stat2 & GP3_POS) != 0 :
                self._event_caller (self.onGP3Rise)

            if (self.stat2 & GP3_NEG) != 0 :
                self._event_caller (self.onGP3Fall)

            if (self.stat2 & FADE_INT) != 0 :
                self._event_caller (self.onFadeProcess)
        return True

    # Read functions #

    # Return the GP1 Configuration#
    def readGP1conf(self) :
        return self._read_reg(REG_GP1CONF)

    # Return the GP1 Configuration#
    def readGP2conf(self) :
        return self._read_reg(REG_GP2CONF)

    # Return the GP1 Configuration#
    def readGP3conf(self) :
        return self._read_reg(REG_GP3CONF)

    # Return the INT pin configuration#
    def readInterruptConfig(self) :
        return self._read_reg(REG_INTCONF)

    # Check if a particular status match.  Before require updateStatus() #
    def readStatus(self, status) :
        if (self.stat & status) != 0 :
            return True
        return False

    # Return the status of the encoder #
    def readStatusRaw(self) :
        return self.stat

    # Check if a particular status of the Int2 match. Before require updateStatus() #
    def readInt2(self, status) :
        return bool((self.stat2 & status) != 0 )

    # Return the Int2 status of the encoder. Before require updateStatus()  #
    def readInt2Raw(self):
        return self.stat2

    # Return Fade process status  #
    def readFadeStatusRaw(self):
        return self._read_reg(REG_FSTATUS)

    # Check if a particular status of the Fade process match. #
    def readFadeStatus(self, status):
        if (self._read_reg(REG_FSTATUS) & status) == 1 :
            return True
        return False

    # Return the PWM LED R value  #
    def readLEDR(self) :
        return self._read_reg(REG_RLED)

    # Return the PWM LED G value  #
    def readLEDG(self) :
        return self._read_reg(REG_GLED)

    # Return the PWM LED B value  #
    def readLEDB(self) :
        return self._read_reg(REG_BLED)

    # Return the 32 bit value of the encoder counter  #
    def readCounterFloat(self) :
        return self._read_reg_float(REG_CVALB4)

    # Return the 32 bit value of the encoder counter  #
    def readCounter32(self) :
        return self._read_reg32(REG_CVALB4)

    # Return the 16 bit value of the encoder counter  #
    def readCounter16(self) :
        return self._read_reg16(REG_CVALB2)

    # Return the 8 bit value of the encoder counter  #
    def readCounter8(self) :
        return self._read_reg(REG_CVALB1)

    # Return the Maximum threshold of the counter #
    def readMax(self) :
        return self._read_reg32(REG_CMAXB4)

    # Return the Minimum threshold of the counter #
    def readMin(self) :
        return self._read_reg32(REG_CMINB4)

    # Return the Maximum threshold of the counter #
    def readMaxFloat(self) :
        return self._read_reg_float(REG_CMAXB4)

    # Return the Minimum threshold of the counter #
    def readMinFloat(self) :
        return self._read_reg_float(REG_CMINB4)

    # Return the Steps increment #
    def readStep(self) :
        return self._read_reg16(REG_ISTEPB4)

    # Return the Steps increment, in float variable #
    def readStepFloat(self) :
        return self._read_reg_float(REG_ISTEPB4)

    # Read GP1 register value #
    def readGP1(self) :
        return self._read_reg(REG_GP1REG)

    # Read GP2 register value #
    def readGP2(self) :
        return self._read_reg(REG_GP2REG)

    # Read GP3 register value #
    def readGP3(self) :
        return self._read_reg(REG_GP3REG)

    # Read Anti-bouncing period register #
    def readAntibouncingPeriod(self) :
        return self._read_reg(REG_ANTBOUNC)

    # Read Double push period register #
    def readDoublePushPeriod(self) :
        return self._read_reg(REG_DPPERIOD)

    # Read the fade period of the RGB LED#
    def readFadeRGB(self) :
        return self._read_reg(REG_FADERGB)

    # Read the fade period of the GP LED#
    def readFadeGP(self):
        return self._read_reg(REG_FADEGP)

    # Read the ID code #
    def readIDCode(self):
        return self._read_reg(REG_IDCODE)

    # Read the Version code #
    def readVersion(self):
        return self._read_reg(REG_VERSION)

    # Read the EEPROM memory#
    def read_eeprom(self, add):
        """
        Read the data in the EEPROM

        :param byte add: registry address from which to read.
        """
        if add <= 0x7f:
            if (self.gconf & EEPROM_BANK1) != 0:
                self.gconf = self.gconf & 0xBF
                self._write_reg(REG_GCONF, self.gconf)

            data = self._read_reg((REG_EEPROMS + add))
        else:
            if (self.gconf & EEPROM_BANK1) == 0:
                self.gconf = self.gconf | 0x40
                self._write_reg(REG_GCONF, self.gconf)

            data = self._read_reg(add)
        return data

    def autoconfig_interrupt(self) :
        """
        Automatically configures the interrupt register according to
        the callback(s) declared.

        """
        reg = 0

        if self.onButtonRelease != None:
            reg = reg | PUSHR

        if self.onButtonPush != None:
            reg = reg | PUSHP

        if self.onButtonDoublePush != None:
            reg = reg | PUSHD

        if self.onIncrement != None:
            reg = reg | RINC

        if self.onDecrement != None:
            reg = reg | RDEC

        if self.onChange != None:
            reg = reg | RINC
            reg = reg | RDEC

        if self.onMax != None:
            reg = reg | RMAX

        if self.onMin != None:
            reg = reg | RMIN

        if self.onMinMax != None:
            reg = reg | RMAX
            reg = reg | RMIN

        if self.onGP1Rise != None:
            reg = reg | INT_2

        if self.onGP1Fall != None:
            reg = reg | INT_2

        if self.onGP2Rise != None:
            reg = reg | INT_2

        if self.onGP2Fall != None:
            reg = reg | INT_2

        if self.onGP3Rise != None:
            reg = reg | INT_2

        if self.onGP3Fall != None:
            reg = reg | INT_2

        if self.onFadeProcess != None:
            reg = reg | INT_2

        self._write_reg(REG_INTCONF, reg)


    # Write the counter value #
    def write_counter(self, value) :
        self._write_reg32(REG_CVALB4, value)

    # Write the counter value #
    def write_counter_float(self, value) :
        self._write_reg_float(REG_CVALB4, value)

    # Write the maximum threshold value #
    def write_max(self, max_val) :
        self._write_reg32(REG_CMAXB4, max_val)

    # Write the maximum threshold value #
    def write_max_float(self, max_val) :
        self._write_reg_float(REG_CMAXB4, max_val)

    # Write the minimum threshold value #
    def write_min(self, min_val) :
        self._write_reg32(REG_CMINB4, min_val)

    # Write the minimum threshold value #
    def write_min_float(self, min_val) :
        self._write_reg_float(REG_CMINB4, min_val)

    # Write the Step increment value #
    def write_step_size(self, step):
        self._write_reg32(REG_ISTEPB4, step)

    # Write the Step increment value #
    def write_step_size_float(self, step):
        self._write_reg_float(REG_ISTEPB4, step)

    # Write the PWM value of the RGB LED red #
    def write_ledr(self, rled):
        self._write_reg(REG_RLED, rled)

    # Write the PWM value of the RGB LED green #
    def write_ledg(self, gled):
        self._write_reg(REG_GLED, gled)

    # Write the PWM value of the RGB LED blue #
    def write_ledb(self, bled):
        self._write_reg(REG_BLED, bled)

    # Write 24bit color code #
    def write_rgb_code(self, rgb):
        self._write_reg24(REG_RLED, rgb)

    # Write GP1 register, used when GP1 is set to output or PWM #
    def writeGP1(self, gp1):
        self._write_reg(REG_GP1REG, gp1)

    # Write GP2 register, used when GP2 is set to output or PWM #
    def writeGP2(self, gp2):
        self._write_reg(REG_GP2REG, gp2)

    # Write GP3 register, used when GP3 is set to output or PWM #
    def writeGP3(self, gp3):
        self._write_reg(REG_GP3REG, gp3)

    def write_antibounce_period(self, bounce):
        self._write_reg(REG_ANTBOUNC, bounce)

    def write_double_push_period(self, dperiod):
        self._write_reg(REG_DPPERIOD, dperiod)

    # Write Fade timing in ms #
    def write_fade_rgb(self, fade):
        self._write_reg(REG_FADERGB, fade)

    # Write Fade timing in ms #
    def write_fade_gp(self, fade):
        self._write_reg(REG_FADEGP, fade)

    # Write the Gamma value on RLED #
    def writeGammaRLED(self, gamma):
        self._write_reg(REG_GAMRLED, gamma)

    # Write the Gamma value on GLED #
    def writeGammaGLED(self, gamma):
        self._write_reg(REG_GAMGLED, gamma)

    # Write the Gamma value on BLED #
    def writeGammaBLED(self, gamma):
        self._write_reg(REG_GAMBLED, gamma)

    # Write the Gamma value on GP1 #
    def writeGammaGP1(self, gamma):
        self._write_reg(REG_GAMMAGP1, gamma)

    # Write the Gamma value on GP2 #
    def writeGammaGP2(self, gamma):
        self._write_reg(REG_GAMMAGP2, gamma)

    # Write the Gamma value on GP3 #
    def writeGammaGP3(self, gamma):
        self._write_reg(REG_GAMMAGP3, gamma)

    # Write the EEPROM memory#
    def write_eeprom(self, add, data):
        """
        write the data to the EEPROM

        :param byte add: registry address where to write.
        :param byte data: 1-Byte of data to write.
        """
        if add <= 0x7f: # Prevent writing to protected registry.
            if (self.gconf & EEPROM_BANK1) != 0:
                self.gconf = self.gconf & 0xBF
                self._write_reg(REG_GCONF, self.gconf)

            self._write_reg((REG_EEPROMS + add), data)
        else:
            if (self.gconf & EEPROM_BANK1) == 0:
                self.gconf = self.gconf | 0x40
                self._write_reg(REG_GCONF, self.gconf)

            self._write_reg(add, data)


    def setInterrupts(self, interrupts):
        """
        Set the Interrupt Configuration Manually.

        :param byte interrupts: byte contining interrupt configuration.
        """
        self._write_reg(REG_INTCONF, interrupts)


    ### Generic Read / Write Functions ###
    def _write_reg(self, reg, value):
        """
        Write a single byte to the registry.

        :param reg: The registry address where to write the byte.
        :param value: the byte data to write.
        """
        buffer = bytearray(2)
        buffer[0] = reg
        buffer[1] = value
        with self.i2c_device as i2c:
            i2c.write(buffer)

    def _write_reg24(self, reg, value):
        """
        Write three bytes to the registry.

        :param reg: The registry address where to write the bytes.
        :param value: the 3-byte data (24-bit) to write.
        """
        buffer = bytearray(3)
        buffer[0] = reg
        packed_struct = struct.pack('>i', value)
        buffer[1:4] = packed_struct[1:4]
        with self.i2c_device as i2c:
            i2c.write(buffer)

    def _write_reg32(self, reg, value):
        """
        Write four bytes to the registry.

        :param reg: The registry address where to write the bytes.
        :param value: the 4-byte data (32-bit) to write.
        """
        buffer = bytearray(4)
        buffer[0] = reg
        packed_struct = struct.pack('>i', value)
        buffer[1:4] = packed_struct[0:4]
        with self.i2c_device as i2c:
            i2c.write(buffer)

    def _write_reg_float(self, reg, value):
        """
        Write four bytes to the registry, as a float.

        :param reg: The registry address where to write the bytes.
        :param value: the 4-byte data (float) to write.
        """
        buffer = bytearray(4)
        buffer[0] = reg
        packed_struct = struct.pack('>f', value)
        buffer[1:4] = packed_struct[0:4]
        with self.i2c_device as i2c:
            i2c.write(buffer)

    def _read_reg(self, reg):
        """
        Read a single byte from the registry.
        Returns a bytearray of size 1.

        :param reg: The registry address from which to read.
        """
        buffer_in = bytearray(1)
        buffer_out = bytearray(1)
        buffer_in[0] = reg
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buffer_in, buffer_out, out_end=1, in_end=1)
        return buffer_out

    def _read_reg16(self, reg):
        """
        Read two bytes from the registry.
        Returns a bytearray of size 2.

        :param reg: The registry address from which to read.
        """
        buffer_in = bytearray(1)
        buffer_out = bytearray(2)
        buffer_in[0] = reg
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buffer_in, buffer_out, out_end=1, in_end=2)
        return buffer_out

    def _read_reg24(self, reg):
        """
        Read tree bytes from the registry.
        Returns a bytearray of size 3.

        :param reg: The registry address from which to read.
        """
        buffer_in = bytearray(1)
        buffer_out = bytearray(3)
        buffer_in[0] = reg
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buffer_in, buffer_out, out_end=1, in_end=3)
        return buffer_out

    def _read_reg32(self, reg):
        """
        Read four bytes from the registry.
        Returns a bytearray of size 4.

        :param reg: The registry address from which to read.
        """
        buffer_in = bytearray(1)
        buffer_out = bytearray(4)
        buffer_in[0] = reg
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buffer_in, buffer_out, out_end=1, in_end=4)
        return buffer_out

    def _read_reg_float(self, reg):
        """
        Read four bytes from the registry.
        Returns a bytearray of size 4 in float format.

        :param reg: The registry address from which to read.
        """
        buffer_in = bytearray(1)
        buffer_out = bytearray(4)
        buffer_in[0] = reg
        with self.i2c_device as i2c:
            i2c.write_then_readinto(buffer_in, buffer_out, out_end=1, in_end=None)
        value = struct.unpack(">f", buffer_out)
        return value
