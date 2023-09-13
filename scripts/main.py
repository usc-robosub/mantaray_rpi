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



def thruster_publisher(name, fsm):
    
    rospy.init_node('mantaray_control', anonymous=True) 
    fsm.set_state(1)

    pub = []

    fsm.current_state.set_rot_target(quaternion_from_euler(math.radians(0),math.radians(0),math.radians(45)))

    for i in range(THRUSTER_COUNT):
        pub.append(rospy.Publisher('/' + name + '/thruster/'+str(i)+'/input', Float64, queue_size=10))
    
    
    while not rospy.is_shutdown():
        fsm.run(100)
        for i in range(THRUSTER_COUNT):
            fs = Float64()
            fs.data = fsm.get_state().get_thrust_list()[i]
            pub[i].publish(fs)

        rate = rospy.Rate(10) # 10hz
        rate.sleep()


def stop():
    print("stop")
    running = False
    rospy.shutdown()



if __name__ == "__main__":
    sub_control_state = fsm()
    print("initializing thrusters...")
    time.sleep(12)
    print("thrusters initialized")
    thruster_publisher(SUB_NAME, sub_control_state)
