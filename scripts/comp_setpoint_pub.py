#!/usr/bin/env python3   
import rospy
from std_msgs.msg import Float64

'''
Creates a new setpoint every second
'''

def cv_data_callback(data):
    # Assumes data is a Float64
    # This is added to <targetGoal> to make the sub face towards the goal post
    global cvGoal
    cvGoal += cvCorrectionP * data.data

if __name__ == "__main__":
    rospy.init_node("yaw_setpoint_pub", anonymous=False, log_level=rospy.DEBUG)
    cvGoal = 0
    cvCorrectionP = 0.1
    pub = rospy.Publisher("/yaw/setpoint", Float64, queue_size=2)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown(): 
        newTarget = Float64()
        newTarget.data = rospy.get_param("targetRadian", default=1) + cvGoal * cvCorrectionP
        pub.publish(newTarget)
        rate.sleep()