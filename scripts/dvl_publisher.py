#!/usr/bin/env python3

from wldvl import WlDVL
import rospy
from mantaray_rpi.msg import Dvl

dvl = WlDVL("/dev/ttyUSB0", 115200)


def dvl_publisher_callback():
    rospy.init_node('dvl_publisher', anonymous=False, log_level=rospy.DEBUG)
    rospy.loginfo("dvl_publisher node started")
    pub = rospy.Publisher('/mantaray/dvl', Dvl, queue_size=10)
    
    while not rospy.is_shutdown():
        data = dvl.read()
        try:
            msg = Dvl()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'world'
            msg.velocity.x = data['vx']
            rospy.logdebug(msg.velocity.x)    
            msg.velocity.y = data['vy']
            rospy.logdebug(msg.velocity.y)    
            msg.velocity.z = data['vz']
            rospy.logdebug(msg.velocity.z)    
            msg.fom = data['fom']
            rospy.logdebug(msg.fom)    
            msg.altitude = data['altitude']
            rospy.logdebug(msg.altitude)    
            msg.valid = data['valid']

            rospy.logdebug("Got a " + str(msg.valid) + " msg")    
            pub.publish(msg)
        except Exception as e:
            rospy.logdebug(e)
        
        rate = rospy.Rate(20) # 10hz
        rate.sleep()


if __name__ == "__main__":
    print("entering dvl publisher`")
    dvl_publisher_callback()
    print("leaving dvl publisher")
