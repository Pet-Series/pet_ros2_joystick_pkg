#!/usr/bin/env python3
# coding = utf-8
 
#################################################################################################
###
### Target: Simple test script for KY-053 Analog Digital Converter ads1115(16-bit x 4 channels)
### Host: Raspberry Pi (Ubuntu) connected to 
###
### $ sudo apt install i2c-tools
### $ sudo apt install python3-pip
### $ sudo pip3 install smbus2
### $ sudo i2cdetect -y 1                                <- Expected address is 0x48
### $ sudo chmod a+rw /dev/i2c-1
### $ sudo pip3 install adafruit-circuitpython-ads1x15   <-Is this necessary?
### $ sudo pip3 install board
### $ sudo pip3 install --force-reinstall adafruit-blinka  <- Due to 20.04 libraries...
### $ sudo python3 ADS1115.py 
###
### Launch sequence:
### 1) $ sudo phyton3 ADS1115.py 

import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Create the I2C bus
i2c = busio.I2C (board.SCL, board.SDA)

# Create the ADC object using the I2C bus
ads = ADS.ADS1115 (i2c)

# Create single-ended input on channels
chan0 = AnalogIn (ads, ADS.P0)
chan1 = AnalogIn (ads, ADS.P1)
chan2 = AnalogIn (ads, ADS.P2)
chan3 = AnalogIn (ads, ADS.P3)


while True:
    print ("channel 0[ n/a  ]:", "{:> 5} \ t {:> 5.3f}". format (chan0.value, chan0.voltage))
    print ("channel 1[Y-axis]:", "{:> 5} \ t {:> 5.3f}". format (chan1.value, chan1.voltage))
    print ("channel 2[Twist ]:", "{:> 5} \ t {:> 5.3f}". format (chan2.value, chan2.voltage))
    print ("channel 3[X-axis]:", "{:> 5} \ t {:> 5.3f}". format (chan3.value, chan3.voltage))
    print ("----------------------------------------------- ---- ")
    time.sleep (1)