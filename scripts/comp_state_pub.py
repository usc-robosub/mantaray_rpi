#!/usr/bin/env python3   
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

'''
Converts the imu Z rotation into a state every second
'''

def updateYawData(data):
    global z_rot
    euler_rot = euler_from_quaternion([data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w])
    z_rot.data = euler_rot[2]

if __name__ == "__main__":
    z_rot = Float64
    z_rot.data = 1
    rospy.init_node("yaw_state_pub", anonymous=False, log_level=rospy.DEBUG)
    pub = rospy.Publisher("/yaw/state", Float64, queue_size=2)
    rospy.Subscriber("/mantaray/imu", Imu, updateYawData)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(z_rot)
        rate.sleep()