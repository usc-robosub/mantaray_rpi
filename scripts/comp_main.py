#!/usr/bin/env python3

import copy
import rospy
import time
from fsm import fsm
from fsm_basic import fsm_basic
from fsm_pcontrol import fsm_pcontrol
from mantaray_rpi.msg import FloatStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header, Float64
from tf.transformations import quaternion_from_euler, euler_from_matrix
import math

running = True

SUB_NAME = "mantaray"
THRUSTER_COUNT = 8

targetGoal = 0
cvCorrectionP = 0.1

def cv_data_callback(data):
    # Assumes data is a Float64
    # This is added to <targetGoal> to make the sub face towards the goal post
    targetGoal += cvCorrectionP * data.data

def thruster_publisher(name, fsm):
    global targetGoal
    rospy.init_node('mantaray_control', anonymous=True) 
    count = 0
    while (rospy.get_param("targetUpdated", default=False) or count < 100):
        targetGoal = rospy.get_param("targetRadian", default=2)
        time.sleep(0.1)

    # Debugging stuff
    if (rospy.get_param("targetUpdated", default=False)):
        print("targetUpdated!")
    else:
        print("Target not updated!")

    fsm.set_state(1)

    pub = []

    fsm.current_state.set_rot_target(quaternion_from_euler(math.radians(0),math.radians(0),math.radians(45)))

    for i in range(THRUSTER_COUNT):
        pub.append(rospy.Publisher('/' + name + '/thruster/'+str(i)+'/input', Float64, queue_size=10))
    
    rospy.Subscriber("/mantaray/cvCorrectAngle", Float64, cv_data_callback) #TODO:WILL Implement publishers

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        fsm.compRun(100, targetGoal) 
        for i in range(THRUSTER_COUNT):
            fs = Float64()
            fs.data = fsm.get_state().get_thrust_list()[i]
            pub[i].publish(fs)
        rate.sleep()
    
    #Stops the motors when ros shuts down
    stop_motors()

def stop_motors(pub):
    for i in range(THRUSTER_COUNT):
        fs = Float64()
        fs.data = 0
        pub[i].publish(fs)

def stop(pub):
    print("stop")
    for i in range(THRUSTER_COUNT):
        fs = Float64()
        fs.data = 0
        pub[i].publish(fs)
    running = False
    rospy.shutdown()

if __name__ == "__main__":
    sub_control_state = fsm()
    # print("initializing thrusters...")
    # time.sleep(12)
    # print("thrusters initialized")
    thruster_publisher(SUB_NAME, sub_control_state)
