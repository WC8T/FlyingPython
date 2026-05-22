#0123456789012345678901234567890123456789012345678901234567890123456789012345678
# SEN0501.py
# functions for operating the SEN0501 v2 from DFRobot
# https://wiki.dfrobot.com/SKU_SEN0501_Gravity_Multifunctional_Environmental_Sensor#FAQ
# Copyright 2026 Paul Taylor WC8T 
#
# This software is based on the https://github.com/cdjq/DFRobot_EnvironmentalSensor.git
# by:
# Copyright 2010 DFRobot Co.Ltd
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software andassociated documentation files (the "Software"), to deal in the
# Software without restriction, including without limitation the rights to use, copy,
# modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so, subject to the
# following conditions:
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
# CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
# OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

from machine import I2C, Pin
import builtins
import ustruct
# I2C1
# GPIO Pin 19 = Pico pin 25  (I2C1, SCL)
# GPIO Pin 18 = Pico pin 24  (I2C1, SDA)
# SEN0501 channel 1, we are on 15 and 14 with address 0x22
# BMP280 channel 1, we are on 15 and 14 with address 0x77

_I2C_Channel = 1
_I2C_GPIO_SCL_Pin = 15
_I2C_GPIO_SDA_Pin = 14
_I2C_Address = 0x22

_REGISTER_PRESSURE = 0X18
_REGISTER_HUMIDITY = 0x16
_REGISTER_TEMP = 0x14
_REGISTER_LUMINOSITY = 0x12
_REGISTER_UV = 0x10
_REGISTER_ADDRESS = 0x04

def SEN0501_Init():
    Error = 0
    i2c = I2C(_I2C_Channel, scl=Pin(_I2C_GPIO_SCL_Pin), sda=Pin(_I2C_GPIO_SDA_Pin), freq=100000)
    # See if the sensor is connected.
    #
    if(False == SEN0501_TestForSEN0501(i2c)):
        # Sensor not found
        Error = 1
        
    return Error, i2c

def SEN0501_read16(i2c, register):
    data = i2c.readfrom_mem(_I2C_Address, register, 2)
    return data[0] << 8 | data[1]

def SEN0501_ReadUV(i2c):
    # read ultraviolet with the LTR390-UV-01 sensor (V2 of the board) and return in mw/cm^2
    uvdata = SEN0501_read16(i2c, _REGISTER_UV)
        
    outputVoltage = (3.0 * uvdata)/1024
    # changed this to check the output voltage to match the DFRobot C code with the range check.
    if outputVoltage <= 0.99:
        outputVoltage = 0.99
    elif outputVoltage >= 2.99:
        outputVoltage = 2.99
    # map function in cpp equivalent is (outputVoltage - fromLow) * (toHigh-toLow) / (fromHigh-fromLow) + toLow    
    uv = (outputVoltage - 0.99) * (15.0 - 0.0) / (2.9 - 0.99) + 0.0
    # Max output at 2.99 volts is 15.71 and Min output at 0.99 volts is 0.00
    return round(uv, 2)

def SEN0501_ReadLuminosity(i2c):
    # read luminosity VEML7700 and return in lx
    luminositydata = SEN0501_read16(i2c, _REGISTER_LUMINOSITY)
    luminosity = luminositydata * (1.0023 + luminositydata * (8.1488e-5 + luminositydata * (-9.3924e-9 + luminositydata * 6.0135e-13)))
    return round(luminosity, 4)

def SEN0501_ReadTemp(i2c):
    # read the temperature SHT-C3 and return in C
    tempdata = SEN0501_read16(i2c, _REGISTER_TEMP)
    temp = (-45) +((tempdata * 175.00) / 1024.00 / 64.00)
    return round(temp, 2)

def SEN0501_ReadHumidity(i2c):
    # read humidity (SHT-C3) and return in %
    humiditydata = SEN0501_read16(i2c, _REGISTER_HUMIDITY)
    humidity = (humiditydata / 1024) * 100 / 64
    return round(humidity,2)

def SEN0501_ReadAtmosphericPres(i2c):
    # read atmospheric pressure BMP280 and return in mBar
    atmopressdata = SEN0501_read16(i2c, _REGISTER_PRESSURE)
    return round(atmopressdata, 2)

def SEN0501_ReadAddress(i2c):
    # reads the i2c address from the SEN0501.  Not really usefull because you have
    # to know the address to read it but here it is.
    addread = SEN0501_read16(i2c, _REGISTER_ADDRESS)
    return addread    
    
def SEN0501_AtmPressToAlt(atmospress):
    # convert atmospheric pressure in mBar to altitude in m
    elevation = 44307.69 * (1.0 - pow(atmospress / 1013.25, 0.190284))
    return round(elevation,1)

def SEN0501_m2ft(meters):
    return round((meters * 3.28084),1)

def SEN0501_C2F(inC):
    return round((inC * 9./5. + 32.), 1)

def SEN0501_TestForSEN0501(i2c):
    addr = _I2C_Address
    devlist = i2c.scan()
    for item in devlist:
        if item == addr: return True
    return False

def SEN0501_ScanForI2CAddresses(i2c):
    devices = i2c.scan()
    if len(devices) == 0:
        print("No i2c device !")
    else:
        print('i2c devices found:',len(devices))
    for device in devices:
         print("At address: ",hex(device))