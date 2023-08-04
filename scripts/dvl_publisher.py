#!/usr/bin/env python3

from wldvl import WlDVL
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
# from mantaray_rpi.msg import Dvl
# Based on Code from Duke Uni's robosub team (https://github.com/DukeRobotics/robosub-ros/blob/pool-test-apr-22/onboard/catkin_ws/src/data_pub/scripts/dvl_to_odom.py)
DVL_ODOM_TOPIC = 'sensors/dvl/odom'

dvl = WlDVL("/dev/ttyUSB0", 115200)

def dvl_publisher_callback():
    rospy.init_node('dvl_publisher', anonymous=False, log_level=rospy.DEBUG)
    rospy.loginfo("dvl_publisher node started")
    odom_pub = rospy.Publisher(DVL_ODOM_TOPIC, Odometry, queue_size=5)
    
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
        


if __name__ == "__main__":
    print("entering dvl publisher`")
    dvl_publisher_callback()
    print("leaving dvl publisher")
