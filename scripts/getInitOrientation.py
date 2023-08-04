#!/usr/bin/env python3   

import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu

if __name__ == "__main__":
    rospy.init_node("getInitOrientation", anonymous=False, log_level=rospy.DEBUG)
    rospy.logdebug("Getting imu data")
    try:
        currentData = rospy.wait_for_message("/mantaray/imu", Imu, timeout=10)
    except Exception as e:
        rospy.logwarn("Failed to get initial orientation!")
        rospy.set_param("targetUpdated", False)
        exit()
    currentOrien = [currentData.orientation.x, currentData.orientation.y, currentData.orientation.z, currentData.orientation.w]
    currentZ = euler_from_quaternion(currentOrien)[2]
    rospy.logdebug("currentZ: " + str(currentZ))
    rospy.set_param("targetRadian", currentZ)
    rospy.set_param("targetUpdated", True)