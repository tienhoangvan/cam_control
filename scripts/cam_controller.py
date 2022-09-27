#!/usr/bin/env python3
import rospy
import serial
import struct
from std_msgs.msg import Float64


def read_distance():
    global distance

    # Generating buffers
    n = 40
    buffer = [None for i in range (n)]         # For storing data read from serial
    #print("\n Initial Buffer:", buffer)

    samples = []        # For storing measurement values
    #print("\n Initial Samples:", samples)

    # Initiating serial connection
    ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, bytesize=8, parity='N', stopbits=1)

    # Variables
    val = float()       # measurement value

    # Get n bytes from buffer
    for i in range(0,n):
        # Read one byte from buffer
        b = ser.read()

        # Check the type of data received from serial
        #print("\n Type data:", type(b))

        # Convert to decimal
        b_val = struct.unpack('B', b)
        #print("\n Type data[i]:", type(b_val))

        buffer[i] = b_val[0]
        #print("\n Data", i, ":", buffer[i])
    #print("\n Raw Data:", buffer)
    
    # Get result as the average value from 10 values
    j = int(n-1)
    k = int(0)
    sum = float(0)  
    while j > 0:
        if buffer[j] == 255:
            val = buffer[j-2] + buffer[j-1]*0.01
            samples.append(val)
            k = k + 1
            sum = sum + val
            # Get only 10 samples
            if k == 10:
                break

            # Jump to new j
            j = j - 3
        else:
            j = j - 1
    #print("\n Samples:", samples)

    # Calculate average values from samples
    distance = round(sum/k, 2)
    #print("\n Sample Number:", k)
    print("\n Measurement Distance:", distance)


if __name__ == '__main__':
    global distance
    distance = Float64()
    try:
        rospy.init_node('distance_measurement')
        pub = rospy.Publisher('distance', Float64, queue_size=10)
        idle = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            try:
                read_distance()
                pub.publish(distance)
                idle = rospy.Rate(50)
            except:
                pass
    except rospy.ROSInterruptException:
        pass
