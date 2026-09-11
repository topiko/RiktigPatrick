#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Raspberry Pi to Arduino I2C Communication
#i2cdetect -y 1

#library
import sys
from smbus2 import SMBus #,smbus2
import time

# Slave Addresses
I2C_SLAVE_ADDRESS = 11 #0x0b ou 11

# This function converts a string to an array of bytes.
def ConvertStringsToBytes(src):
  converted = []
  for b in src:
    converted.append(ord(b))
  return converted

def main():
    # Create the I2C bus
    with SMBus(1) as I2Cbus:
        I2Cbus.pec=1
        while True:
            for i in range(2**8-1):
                #print("Sending ", str(i))
                #I2Cbus.write_i2c_block_data(I2C_SLAVE_ADDRESS, 0x00, ConvertStringsToBytes(str(i)))
                response=False
                while not response:
                    try:
                        #data=I2Cbus.read_i2c_block_data(I2C_SLAVE_ADDRESS,0x00,6)
                        data=I2Cbus.read_i2c_block_data(I2C_SLAVE_ADDRESS,0x00,8)
                        print("recieve from slave:")
                        print(data)
                        response=True
                    except:
                        print("remote i/o error")
                        time.sleep(0.5)
    return 0

if __name__ == '__main__':
     try:
        main()
     except KeyboardInterrupt:
        print("program was stopped manually")
