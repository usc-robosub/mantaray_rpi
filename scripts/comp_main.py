#!/usr/bin/env python3

import rospy
# from fsm import fsm
from std_msgs.msg import Float64
from tf.transformations import quaternion_from_euler
import math

running = True

SUB_NAME = "mantaray"
THRUSTER_COUNT = 8

targetGoal = 0
cvCorrectionP = 0.1

def cv_data_callback(data):
    # Assumes data is a Float64
    # This is added to <targetGoal> to make the sub face towards the goal post
    global targetGoal
    targetGoal += cvCorrectionP * data.data

def thruster_publisher(name, fsm):
    global targetGoal
    # fsm.set_state(2)

    pub = []

    # fsm.current_state.set_rot_target(quaternion_from_euler(math.radians(0),math.radians(0),math.radians(45)))

    for i in range(THRUSTER_COUNT):
        pub.append(rospy.Publisher('/' + name + '/thruster/'+str(i)+'/input', Float64, queue_size=10))
    
    # rospy.Subscriber("/mantaray/cvCorrectAngle", Float64, cv_data_callback) #TODO:WILL Implement publishers

    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        # rospy.logdebug("targetGoal: "+str(targetGoal))
        fsm.compRun(100, targetGoal) 
        for i in range(THRUSTER_COUNT):
            fs = Float64()
            fs.data = fsm.get_state().get_thrust_list()[i]
            pub[i].publish(fs)
            # rospy.logdebug("Current thrust[" + str(i) + "]: " + str(fs.data))
        rate.sleep()
    
    #Stops the motors when ros shuts down
    stop_motors(pub)

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
    rospy.init_node('mantaray_control', anonymous=True, log_level=rospy.DEBUG)
    count = 0

    # Debugging stuff
    if (rospy.get_param("targetUpdated", default=False)):
        rospy.logdebug("targetUpdated!")
        rospy.logdebug("New target: " + str(targetGoal))
    else:
        rospy.logdebug("Target not updated!")

    sub_control_state = fsm()
    rospy.logdebug("initializing thrusters in comp_main...")
    targetGoal = rospy.get_param("targetRadian", 0)
    thruster_publisher(SUB_NAME, sub_control_state)
