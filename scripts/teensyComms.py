
#!/usr/bin/env python3

# Sets up publishers for pressure sensor and a service for kill switch

import rospy
from std_msgs.msg import Float64
from std_msgs.srv import SetBool, SetBoolResponse
# from mantaray_rpi.msg import FloatStamped

dataTypes = ["depth", "pressure", "temperature", "altititude"] # Publishers
numDataTypes = len(dataTypes)

pubs = [None] * numDataTypes

def handle_kill_sw(req):
    print(f"Changing Ks state to {req.data}")
    return SetBoolResponse(req.a + req.b)

def kill_sw_server():
    rospy.init_node('kill_switch')
    s = rospy.Service('change_kill_switch', SetBool, handle_kill_sw)
    print("Kill switch ready.")

if __name__ == "__main__":
    rospy.init_node("teensyData", anonymous=False)
    for i in range(numDataTypes):
        pubs[i] = rospy.Publisher("/mantaray/teensyData/"+dataTypes[i], Float64, queue_size=1)
    kill_sw_server()
    rospy.spin()
