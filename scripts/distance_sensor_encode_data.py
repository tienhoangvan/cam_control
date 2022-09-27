#!/usr/bin/env python3

from locale import D_T_FMT
import serial
import struct

encoding = 'utf-8'

def read_distance():
    global ser

    data=[None for i in range (3)]
    ser = serial.Serial('/dev/ttyUSB0', baudrate=19200, bytesize=8, parity='N', stopbits=1)

    result = float()

    count = int(0)
    while count < 3:
  
        for i in range(3):
            # Check the type of data received from serial
            d = ser.read()
            print("\n Type data:", type(d))

            # Convert to decimal
            dt = struct.unpack('B', d)
            print("\n Type data[i]:", type(dt))

            data[i] = dt[0]
            
            print("\n Data", i, ":", data[i])
        
        print("\n Raw Data:", data)

        result = data[0] + data[1]*0.01 

        print("\n Result:", result)
        count = count + 1


if __name__ == '__main__':
    global ser
    read_distance()
