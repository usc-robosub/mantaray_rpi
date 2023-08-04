import rospy
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu

if __name__ == "__main__":
    currentData = rospy.wait_for_message("/manataray/imu", Imu, timeout=10)
    currentZ = euler_from_quaternion(currentData)[2]
    rospy.set_param("targetRadian", currentZ)