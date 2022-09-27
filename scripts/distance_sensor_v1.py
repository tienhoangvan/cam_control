#!/usr/bin/env python3

from pymodbus.client.sync import ModbusSerialClient


def read_distance():
    global client

    client = ModbusSerialClient(method='rtu', baudrate=19200,
                                port='/dev/ttyUSB0', stopbits=1, parity='N', bytesize=8)
    # Connect to the serial modbus server
    connection = client.connect()
    print("\n Connected to Sensor:", connection, "\n")  # kiem tra ket noi dc hay chua

    # Test reading values
    #result = client.read_holding_registers(0,3,unit=0x01)
    #print("\n Result:", result, "\n")


if __name__ == '__main__':
    try:
        while True:
            read_distance()
    except:
        pass
