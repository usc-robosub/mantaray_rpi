#!/usr/bin/env python3

# Sets up publishers for pressure sensor and a service for kill switch

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, SetBoolResponse
#import smbus
import struct
# from mantaray_rpi.msg import FloatStamped

dataTypes = ["depth", "pressure", "temperature", "altititude"] # Publishers
numDataTypes = len(dataTypes)
dataTypeCommandNums = [5, 9, 13, 17]

pubs = [None] * numDataTypes

#init i2c
#bus = smbus.SMBus(0) # 0 as in /dev/i2c-0
i2c_address = 0x2d

def writeNumber(value):
    bus.write_byte(i2c_address, value)
    return -1

def readString(register):
    data = bus.read_i2c_block_data(i2c_address, register, 16) # read a block of up to 16 bytes
    result = ''.join(chr(i) for i in data)
    return result

def readData(_data):
    data = bus.read_byte_data(i2c_address, _data) # read block of up to 16 bytes
    return data

def receive_float(command, size=4):
    #data = bus.read_i2c_block_data(i2c_address, command, size)
    #data = struct.unpack('f', bytearray(data))[0]
    data = 100
    return data

def handle_kill_sw(req):
    print(f"Changing Ks state to {req.data}")
    # Write to teensy using i2c the killswitch state
    #writeNumber(req.data)
    return SetBoolResponse(True, "test")

def kill_sw_server():
    rospy.init_node('kill_switch')
    s = rospy.Service('change_kill_switch', SetBool, handle_kill_sw)
    print("Kill switch ready.")

if __name__ == "__main__":
    #rospy.init_node("teensyData", anonymous=False)
    #for i in range(numDataTypes):
    #    pubs[i] = rospy.Publisher("/mantaray/teensyData/"+dataTypes[i], Float64, queue_size=1)
    kill_sw_server()
    
    #r = rospy.Rate(1) #1hz
    #while not rospy.is_shutdown():
        #num = int(input("Enter a number: "))
        #print("testing...")
        #for i in range(numDataTypes):
        #    pubs[i].publish(receive_float(dataTypeCommandNums[i]))
        #r.sleep()
    rospy.spin()
