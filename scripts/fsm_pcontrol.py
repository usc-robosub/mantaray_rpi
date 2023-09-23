#!/usr/bin/env python  

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from mantaray_rpi.msg import Dvl
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from qp_attitude_controller import QuaternionAttitudeController
from fsm_state import fsm_state
from pid_control import pid

ODOMETRY_TOPIC = '/odometry/filtered'
ACCEL_TOPIC = '/accel/filtered'
KF_ANGULAR = 30

G = 9.80665



class fsm_pcontrol(fsm_state):
    def __init__(self):
        self.name = 'pcontrol'

        self.last_imu_timestamp = 0

        # 0 is x, 1 is y, 2 is z
        self.rot_pid = [None]*3
        self.rot_pid[0] = pid(10,0,0)
        self.rot_pid[1] = pid(10,0,0)
        self.rot_pid[2] = pid(10,0,0) #150, 0 ,0
        
        self.qp_attitude_controller = QuaternionAttitudeController(2)

        self.rot_target = [0,0,0,0]

        self.thrust_list = [0,0,0,0,0,0,0,0]
        self.rot_quat = [0,0,0,0]
        self.position = [0,0,0]

        self.imu_listener = rospy.Subscriber(ODOMETRY_TOPIC, Odometry, self.odom_callback)
        
    def odom_callback(self, msg):
        msg_quat = msg.pose.pose.orientation
        self.rot_quat = [msg_quat.x, msg_quat.y, msg_quat.z, msg_quat.w]
        self.position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        
        

    def set_rot_target(self, quat):
        self.rot_target = quat

    def get_rpy(self):
        return euler_from_quaternion(self.rot_quat,axes='sxyz')
        

    def run(self, dt):
        rpy_vel_targets = self.qp_attitude_controller.get_angular_setpoint(self.rot_target, self.rot_quat)
        
        
        rospy.loginfo(self.position)

        # for i in range(3):
        #     self.rot_pid[i].set_state(rot_vel[i])
        #     self.rot_pid[i].update(rpy_vel_targets[i], dt)

        self.thrust_list[0] = -self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
        self.thrust_list[1] = self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
        self.thrust_list[2] = self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
        self.thrust_list[3] = -self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
        self.thrust_list[4] = -self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) - self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])
        self.thrust_list[5] = self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) - self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])
        self.thrust_list[6] = -self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) +  self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])
        self.thrust_list[7] = self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) + self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])

        


    def enter(self):
        pass

    def exit(self):
        pass

    def get_thrust_list(self):
        return self.thrust_list

