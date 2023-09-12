import rospy
import numpy as np
from sensor_msgs.msg import Imu
from mantaray_rpi.msg import Dvl
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from qp_attitude_controller import QuaternionAttitudeController
from fsm_state import fsm_state
from pid_control import pid

IMU_TOPIC = "mantaray/imu"
DVL_TOPIC = "mantaray/dvl"
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

        self.depth_pid = pid(30,0,0)
        
        self.qp_attitude_controller = QuaternionAttitudeController(2)

        self.rot_target = [0,0,0,0]

        self.pos_pid = pid(0,0,0)
        self.thrust_list = [0,0,0,0,0,0,0,0]
        self.rot_quat = [0,0,0,0]

        self.imu_listener = rospy.Subscriber(IMU_TOPIC, Imu, self.imu_callback)
        self.dvl_listener = rospy.Subscriber(DVL_TOPIC, Dvl, self.dvl_callback)
        
    def dvl_callback(self, data):
        if data.valid == True:
            self.Z[9:12] = np.array([data.velocity.x, data.velocity.y, data.velocity.z])
            self.dvl_cov = np.eye*data.fom
            self.dvl_cov = self.dvl_cov.flatten()
        
        

    def imu_callback(self, data):
        imu_quat = np.zeros(4)

        imu_quat[0] = data.orientation.x
        imu_quat[1] = data.orientation.y
        imu_quat[2] = data.orientation.z
        imu_quat[3] = data.orientation.w
        
        self.rot_quat = imu_quat
        
        imu_euler = euler_from_quaternion(imu_quat)

        self.Z[0:9] = np.array([imu_euler[0],
             imu_euler[1],
             imu_euler[2],
             data.angular_velocity.x*np.pi/180, #convert to radians per second
             data.angular_velocity.y*np.pi/180,
             data.angular_velocity.z*np.pi/180,
             data.linear_acceleration.x*G,
             data.linear_acceleration.y*G,
             data.linear_acceleration.z*G])
        
        
        

    def set_rot_target(self, quat):
        self.rot_target = quat

    def get_rpy(self):
        return euler_from_quaternion(self.rot_quat,axes='sxyz')
        

    def run(self, dt):
                
        
        rot_vel = self.Z[3:6]


        rpy_vel_targets = self.qp_attitude_controller.get_angular_setpoint(self.rot_target, self.rot_quat)
        
        #rospy.loginfo(self.thrust_list) 
        
        #rospy.loginfo(rpy_vel_targets)

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

        self.turn_pid.set_state(euler_from_quaternion(self.rot_quat)[2])
        self.turn_pid.update(1,dt) # radians


    def enter(self):
        pass

    def exit(self):
        pass

    def get_thrust_list(self):
        return self.thrust_list

