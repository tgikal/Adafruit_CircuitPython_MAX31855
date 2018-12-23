# The MIT License (MIT)
#
# Copyright (c) 2017 Radomir Dopieralski for Adafruit Industries.
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
``adafruit_max31855``
===========================

This is a CircuitPython driver for the Maxim Integrated MAX31855 thermocouple
amplifier module.

* Author(s): Radomir Dopieralski

Implementation Notes
--------------------

**Hardware:**

* Adafruit `MAX31855 Thermocouple Amplifier Breakout
  <https://www.adafruit.com/product/269>`_ (Product ID: 269)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the ESP8622 and M0-based boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""
try:
    import struct
except ImportError:
    import ustruct as struct

import math

from adafruit_bus_device.spi_device import SPIDevice

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MAX31855.git"

class MAX31855:
    """
    Driver for the MAX31855 thermocouple amplifier.
    """

    def __init__(self, spi, cs):
        self.spi_device = SPIDevice(spi, cs)
        self.data = bytearray(4)

    def _read(self, internal=False):
        with self.spi_device as spi:
            spi.readinto(self.data)  #pylint: disable=no-member
        if self.data[3] & 0x01:
            raise RuntimeError("thermocouple not connected")
        if self.data[3] & 0x02:
            raise RuntimeError("short circuit to ground")
        if self.data[3] & 0x04:
            raise RuntimeError("short circuit to power")
        if self.data[1] & 0x01:
            raise RuntimeError("faulty reading")
        temp, refer = struct.unpack('>hh', self.data)
        refer >>= 4
        temp >>= 2
        if internal:
            return refer
        return temp

    @property
    def temperature(self):
        """Thermocouple temperature in degrees Celsius."""
        return self._read() / 4

    @property
    def reference_temperature(self):
        """Internal reference temperature in degrees Celsius."""
        return self._read(True) * 0.625

    @property
    def linearized_temperature(self):
        """Return the NIST-linearized thermocouple temperature value in degrees Celsius."""
        # Read temperatures
        probeTempC = self.temperature()
        internalTempC = self.reference_temperature()
        
        # MAX31855 thermocouple voltage reading in mV
        thermocoupleVoltage = (probeTempC - internalTempC) * 0.041276
        
        # MAX31855 cold junction voltage reading in mV
        coldJunctionVoltage = (-0.176004136860E-01 +
            0.389212049750E-01  * internalTempC +
            0.185587700320E-04  * math.pow(internalTempC, 2.0) +
            -0.994575928740E-07 * math.pow(internalTempC, 3.0) +
            0.318409457190E-09  * math.pow(internalTempC, 4.0) +
            -0.560728448890E-12 * math.pow(internalTempC, 5.0) +
            0.560750590590E-15  * math.pow(internalTempC, 6.0) +
            -0.320207200030E-18 * math.pow(internalTempC, 7.0) +
            0.971511471520E-22  * math.pow(internalTempC, 8.0) +
            -0.121047212750E-25 * math.pow(internalTempC, 9.0) +
            0.118597600000E+00  * math.exp(-0.118343200000E-03 *
            math.pow((internalTempC-0.126968600000E+03), 2.0)))

        # cold junction voltage + thermocouple voltage
        voltageSum = thermocoupleVoltage + coldJunctionVoltage

        # calculate corrected temperature reading based on coefficients for 3 different ranges
        # float b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10;
        if voltageSum < 0:
            b0 = 0.0000000E+00
            b1 = 2.5173462E+01
            b2 = -1.1662878E+00
            b3 = -1.0833638E+00
            b4 = -8.9773540E-01
            b5 = -3.7342377E-01
            b6 = -8.6632643E-02
            b7 = -1.0450598E-02
            b8 = -5.1920577E-04
            b9 = 0.0000000E+00
        elif voltageSum < 20.644:
            b0 = 0.000000E+00
            b1 = 2.508355E+01
            b2 = 7.860106E-02
            b3 = -2.503131E-01
            b4 = 8.315270E-02
            b5 = -1.228034E-02
            b6 = 9.804036E-04
            b7 = -4.413030E-05
            b8 = 1.057734E-06
            b9 = -1.052755E-08
        elif voltageSum < 54.886:
            b0 = -1.318058E+02
            b1 = 4.830222E+01
            b2 = -1.646031E+00
            b3 = 5.464731E-02
            b4 = -9.650715E-04
            b5 = 8.802193E-06
            b6 = -3.110810E-08
            b7 = 0.000000E+00
            b8 = 0.000000E+00
            b9 = 0.000000E+00
        else:
            raise RuntimeError("temperature out of linearized range")
        
        return (b0 + b1 * voltageSum +
            b2 * pow(voltageSum, 2.0) +
            b3 * pow(voltageSum, 3.0) +
            b4 * pow(voltageSum, 4.0) +
            b5 * pow(voltageSum, 5.0) +
            b6 * pow(voltageSum, 6.0) +
            b7 * pow(voltageSum, 7.0) +
            b8 * pow(voltageSum, 8.0) +
            b9 * pow(voltageSum, 9.0))
