#!/usr/bin/env python3
### $ sudo apt install i2c-tools 
### $ sudo apt install python3-pip 
### $ sudo pip3 install smbus2 
### $ sudo i2cdetect -y 1 
### $ sudo pip3 install adafruit-circuitpython-ads1x15 
### $ sudo pip3 install board 
### $ sudo pip3 install --force-reinstall adafruit-blinka 
import board
import digitalio
import busio

print("Hello blinka!")

# Try to great a Digital input
pin = digitalio.DigitalInOut(board.D4)
print("Digital IO ok!")

# Try to create an I2C device
i2c = busio.I2C(board.SCL, board.SDA)
print("I2C ok!")

# Try to create an SPI device
spi = busio.SPI(board.SCLK, board.MOSI, board.MISO)
print("SPI ok!")

print("done!")