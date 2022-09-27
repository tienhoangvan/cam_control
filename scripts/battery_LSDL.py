#!/usr/bin/env python
import rospy
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.client.sync import ModbusSerialClient
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Point
from std_msgs.msg import String

global client
client = ModbusSerialClient(method = 'rtu', baudrate= 9600, port='/dev/ttyS1', stopbits= 1, paraity= 'N', bytesize=8)

#Connect to the serial modbus server
connection = client.connect()
print (connection) #kiem tra ket noi dc hay chua 

#result = client.read_holding_registers(0,1,unit=0x01)
#print(result.registers[0])
  
def read_BAT():
	global client
	rospy.init_node("modbus", anonymous=True)
	pub = rospy.Publisher('BAT',Point,queue_size=10)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		try:
			result = client.read_holding_registers(18,3,unit=0x01) #doc tu 18, so luong 3, ID = 1
			voltage = result.registers[0]*0.01
			charging = result.registers[1]*0.01
			capacity = result.registers[2]*0.025		 
			pub.publish(Point(voltage,charging,capacity))
			"""
			if charging < 32767: # charging
				print "voltage(V): ", voltage*0.01, "charging (A): ",charging*0.01, "% BAT :", capacity*0.025  	# 100*capacity/6553.5	
			else:
				charging = (65535 - charging)*0.01
				print "voltage(V): ", voltage*0.01, "discharging (A): ", charging, "% BAT :", capacity*0.025	# 100*capacity/6553.5	
			"""				
		except:
			client = ModbusSerialClient(method = 'rtu', baudrate= 9600, port='/dev/ttyS1', stopbits= 1, paraity= 'N', bytesize=8)
			connection = client.connect()
		rate.sleep()        

if __name__ == '__main__':
    try:
        read_BAT()
    except rospy.ROSInterruptException:
        pass
