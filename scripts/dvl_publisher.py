#!/usr/bin/env python3

from wldvl import WlDVL
import rospy
from mantaray_rpi.msg import Dvl

dvl = WlDVL("/dev/ttyUSB0", 115200)


def dvl_publisher_callback():
    rospy.init_node('dvl_publisher', anonymous=False)
    rospy.loginfo("dvl_publisher node started")
    pub = rospy.Publisher('/mantaray/dvl', String, queue_size=10)
    
    while not rospy.is_shutdown():
        data = dvl.read()
        
        msg = Dvl()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'world'
        msg.velocity.x = data['vx']
        msg.velocity.y = data['vy']
        msg.velocity.z = data['vz']
        msg.fom = data['fom']
        msg.altitude = data['altitude']
        msg.valid = data['valid']
        
        pub.publish(msg)
        
        rate = rospy.Rate(10) # 10hz
        rate.sleep()


if "__name__" == "__main__":
    dvl_publisher_callback()
