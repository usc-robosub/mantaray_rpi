# Simply returns the orientation of the sub to help with finding the goalTarget
import rospy
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import euler_from_quaternion

def imu_callback(data):
    imu_quat = np.zeros(4)

    imu_quat[0] = data.orientation.x
    imu_quat[1] = data.orientation.y
    imu_quat[2] = data.orientation.z
    imu_quat[3] = data.orientation.w

    rospy.loginfo(imu_quat[3])

if __name__ == "__main__":
    rospy.init_node('imu_listener', anonymous=True, log_level=rospy.INFO) 
    imu_listener = rospy.Subscriber("mantaray/imu", Imu, imu_callback)
    rospy.spin()
