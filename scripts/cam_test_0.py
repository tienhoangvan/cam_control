#!/usr/bin/env python3

import time
import serial
import struct

def zoom_write():
    global zoom_in, zoom_out, zoom_stop

    # Initiating serial connection
    ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, bytesize=8, parity='N', stopbits=1)

    #print("\n Trying to send zoom command to Camera over serial port...")

    if ser.isOpen():
        try:
            stop_command = struct.pack('BBBBBBB', 255, 1, 0, 0, 0, 0, 1)

            # Zoom-In or Zoom-Out
            if zoom_in:
                data = struct.pack('BBBBBBB', 255, 1, 0, 32, 0, 0, 33)      # ZOOM-IN  
            if zoom_out:
                data= struct.pack('BBBBBBB', 255, 1, 0, 64, 0, 0, 65)       # ZOOM-OUT
            
            # Write the command
            ser.write(data)
            
            # Delay
            time.sleep(2)

            # Stop Zoom
            ser.write(stop_command)
            print('\n Zoom successfully!')

            # Close the connection
            ser.close()

        except Exception as e_w:
            logMsg ='\n Communication error...:' + str(e_w)
            print(logMsg)

def zoom_read():
    # Initiating serial connection
    ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, bytesize=8, parity='N', stopbits=1)

    if ser.isOpen():
        # Generating buffers
        n = 7
        buffer = [None for i in range (n)]         # For storing data read from serial
        try:
            # Send a request to receive a zoom feedback
            data_req = struct.pack('BBBBBBB', 255, 1, 0, 167, 0, 0, 168)
            
            # Write the command
            ser.write(data_req)
            
            # Read the zoom feedback
            # Get n bytes from buffer
            for i in range(0,n):
                # Read one byte from buffer
                b = ser.read()

                # Convert to decimal
                b_val = struct.unpack('B', b)
                buffer[i] = b_val[0]
                #print("\n Data", i, ":", buffer[i])

            # Close the connection
            ser.close()
            print("\n Raw Data:", buffer)

            # Decode: X30 <--> 64*256; 360 <-->65535
            # High byte
            high_byte = buffer[4]
            
            #Low byte
            low_byte = buffer[5]
            
            raw_value = high_byte*256 + low_byte
            
            #result = raw_value*360/65535
            result = raw_value*30/(64*256)
            result = round(result)

            print('\n Zoom Level: ', result)
            
        except Exception as e_r:
            logMsg ='\n Communication error...:' + str(e_r)
            print(logMsg)

if __name__ == '__main__':
    global zoom_in, zoom_out, zoom_stop

    zoom_in = bool(False)
    zoom_out = bool(False)
    zoom_stop = bool(False)

    zoom_in = True
    zoom_write()

    time.sleep(5)
    zoom_read()
