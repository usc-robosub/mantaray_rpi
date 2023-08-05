#!/usr/bin/env python3   
'''
Subscribes to a control effot publisher and publishes to the corresponding thrusters.
For the PID
'''
import rospy
from std_msgs.msg import Float64
NUM_YAW_THRUSTERS = 4

def updateThrusters(data):
    global pubs
    pubs[0].publish(-data.data)
    pubs[1].publish(data.data)
    # pubs[2].publish(data.data)
    # pubs[3].publish(data.data)

if __name__ == "__main__":
    rospy.init_node("yaw_controller", anonymous=True, log_level=rospy.DEBUG)
    for i in range(NUM_YAW_THRUSTERS):
        pubs = rospy.Publisher("/mantaray/thruster/"+ str(i) + "/input", Float64, queue_size=5)
    rospy.Subscriber("/yaw/control_effort", Float64, callback=updateThrusters)
    rospy.spin()