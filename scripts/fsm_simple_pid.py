import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from fsm_state import fsm_state
from pid_control import pid

IMU_TOPIC = "mantaray/imu"
DVL_TOPIC = "mantaray/dvl"

G = 9.80665

class fsm_simple_pid(fsm_state):
    def __init__(self):
        self.name = 'pcontrol'

        self.turn_pid = pid(3, 0.1, 0, 1.5) #YAW ONLY
        # self.burstMotor = 0
        self.rot_target = [0,0,0,0]
        self.init_thrust = -10

        self.pos_pid = pid(0,0,0)
        self.thrust_list = [0,0,0,0,0,0,0,0]
        self.rot_quat = [0,0,0,0]

        self.imu_listener = rospy.Subscriber(IMU_TOPIC, Imu, self.imu_callback)

    def imu_callback(self, data):
        imu_quat = np.zeros(4)

        imu_quat[0] = data.orientation.x
        imu_quat[1] = data.orientation.y
        imu_quat[2] = data.orientation.z
        imu_quat[3] = data.orientation.w
        
        self.rot_quat = imu_quat
        
    def set_rot_target(self, quat):
        self.rot_target = quat

    def get_rpy(self):
        return euler_from_quaternion(self.rot_quat,axes='sxyz')
        
    def compRun(self, dt, targetGoal):        # Axises are messed up so z is 0  
        self.turn_pid.set_state(euler_from_quaternion(self.rot_quat)[2])
        rospy.logdebug("turn pid is set to " + str(euler_from_quaternion(self.rot_quat)[2]))
        rospy.logdebug("Turn pid output is " + str(self.turn_pid.get_output()))
        self.turn_pid.update(targetGoal,dt) # radians

        # forward_bias = 17.5
        forward_bias = 23
        vertical_bias = -8.3  #vertical_bias = self.depth_pid.get_output()
        # self.thrust_list[0] = -self.turn_pid.get_output()# 0 and 3 match and 1 and 2 match
        # self.thrust_list[1] = self.turn_pid.get_output()
        
        # self.thrust_list[0] = forward_bias - 0.5 # 0 and 3 match and 1 and 2 match
        # self.thrust_list[1] = forward_bias
        # self.thrust_list[2] = forward_bias + 3
        # self.thrust_list[3] = forward_bias - 1
        # if (self.burstMotor < 10) :
        #     self.thrust_list[0] = -self.turn_pid.get_output()
        #     self.thrust_list[1] = self.turn_pid.get_output()
        #     self.thrust_list[2] = self.turn_pid.get_output()
        #     self.thrust_list[3] = -self.turn_pid.get_output()
        #     self.burstMotor += 1
        # elif self.burstMotor >= 10 and self.burstMotor:
        #     self.thrust_list[0] = 0
        #     self.thrust_list[1] = 0
        #     self.thrust_list[2] = 0
        #     self.thrust_list[3] = 0
        #     self.burstMotor = 0
        self.thrust_list[0] = -self.turn_pid.get_output()
        self.thrust_list[1] = self.turn_pid.get_output()
        # self.thrust_list[2] = self.turn_pid.get_output()
        # self.thrust_list[3] = -self.turn_pid.get_output()
        self.thrust_list[4] = vertical_bias - 1 + 0.5
        self.thrust_list[5] = vertical_bias + 0.5
        self.thrust_list[6] = vertical_bias
        self.thrust_list[7] = vertical_bias - 1
        # self.thrust_list[4] = 0
        # self.thrust_list[5] = 0
        # self.thrust_list[6] = 0
        # self.thrust_list[7] = 0
        if self.init_thrust < 0:
            self.init_thrust += 0.07
            rospy.logdebug(self.init_thrust)

    def enter(self):
        pass

    def exit(self):
        pass

    def get_thrust_list(self):
        return self.thrust_list

