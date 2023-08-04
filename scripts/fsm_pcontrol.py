import rospy
import numpy as np
from sensor_msgs.msg import Imu
from mantaray_rpi.msg import Dvl
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from qp_attitude_controller import QuaternionAttitudeController
from fsm_state import fsm_state
from pid_control import pid
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

IMU_TOPIC = "mantaray/imu"
DVL_TOPIC = "mantaray/dvl"
KF_ANGULAR = 30

G = 9.80665

# initialize H for the imu

H = np.array([  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # roll
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # pitch
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # yaw
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0], # roll rate
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], # pitch rate
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], # yaw rate
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], # x accel
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], # y accel
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],# z accel
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #x vel
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], #y vel
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], #z vel
                [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]) #altitude


# initialize Q
Q = np.eye(18)*0.1


class fsm_pcontrol(fsm_state):
    def __init__(self):
        self.name = 'pcontrol'
        self.kf = KalmanFilter(dim_x=18, dim_z=13, dim_u=8)
        self.kf.x = np.zeros(18)
        self.kf.P *= 1000
        self.kf.R *= 0
        self.kf.H = H
        
        self.acc_cov = np.zeros(9)
        self.ang_cov = np.zeros(9)
        self.ori_cov = np.zeros(9)
        self.dvl_cov = np.zeros(9)
        
        self.Z = np.zeros(13)

        self.last_imu_timestamp = 0

        # 0 is x, 1 is y, 2 is z
        self.rot_pid = [None]*3
        self.rot_pid[0] = pid(10,0,0)
        self.rot_pid[1] = pid(10,0,0)
        self.rot_pid[2] = pid(10,0,0) #150, 0 ,0

        self.depth_pid = pid(30,0,0)

        self.turn_pid = pid(40,0,0) #YAW ONLY 
        
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
        
        # create covariance matrix for the imu within the 18x18 model
        
        self.ori_cov = np.array(data.orientation_covariance)
        self.ang_cov = np.array(data.angular_velocity_covariance)
        acc_cov = np.array(data.linear_acceleration_covariance)
        
        #self.acc_cov = np.eye(3)*0.2
        #self.acc_cov = self.acc_cov.flatten()
        
        # rospy.loginfo(imu_euler)

        # only print first 3 decimals
        
        #rospy.loginfo("%3f, %3f, %3f, %3f, %3f, %3f, %3f, %3f, %3f", self.Z[0], self.Z[1], self.Z[2], self.Z[3], self.Z[4], self.Z[5], self.Z[6], self.Z[7], self.Z[8])

      #  rospy.loginfo(self.kf.x[3:6])
        

    def set_rot_target(self, quat):
        self.rot_target = quat

    def get_rpy(self):
        return euler_from_quaternion(self.rot_quat,axes='sxyz')
        

    # def run(self, dt):

    #     A = np.array([  
    #             [1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0, 0, 0, 0, 0],
    #             [0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0, 0, 0, 0],
    #             [0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0, 0, 0],
    #             [0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0, 0],
    #             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0],
    #             [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2],
    #             [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
        
    #     self.R = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,self.ori_cov[0],self.ori_cov[1],self.ori_cov[2],0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,self.ori_cov[3],self.ori_cov[4],self.ori_cov[5],0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,self.ori_cov[6],self.ori_cov[7],self.ori_cov[8],0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,self.ang_cov[0],self.ang_cov[1],self.ang_cov[2],0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,self.ang_cov[3],self.ang_cov[4],self.ang_cov[5],0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,self.ang_cov[6],self.ang_cov[7],self.ang_cov[8],0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,self.acc_cov[0],self.acc_cov[1],self.acc_cov[2],0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,self.acc_cov[3],self.acc_cov[4],self.acc_cov[5],0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,self.acc_cov[6],self.acc_cov[7],self.acc_cov[8],0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

    #     self.kf.F = A

    #     B = np.array([  [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [0, 0, 0, 0, 0, 0, 0, 0],
    #             [-0.5, 0.5, 0.5, -0.5, 0, 0, 0, 0],
    #             [-0.8660251013910845, -0.8660257817756856, 0.8660253281861298, 0.8660255549809972,0.0, 0.0, 0.0, 0.0],
    #             [0.0, 0.0, 0.0, 0.0, 1, 1, 1, 1],
    #             [0.10219096196414795, 0.10219104224953088, -0.1021909887259633, -0.10219101548775772, 0.2, 0.2, -0.2, -0.2],
    #             [-0.059000061803713866, 0.0589999227453273, 0.0590000154509305, -0.058999969098135, 0.25, -0.25, 0.25, -0.25],
    #             [0.03268559890777384, -0.03268614782371279, 0.032685781879760206, -0.032535964930304084, 0, 0, 0, 0]])
                
        
    #     #rot_vel = self.kf.x[9:12]
        
    #     rot_vel = self.Z[3:6]


    #     rpy_vel_targets = self.qp_attitude_controller.get_angular_setpoint(self.rot_target, self.rot_quat)
        
    #     #rospy.loginfo(self.thrust_list) 
        
    #     #rospy.loginfo(rpy_vel_targets)

    #     # for i in range(3):
    #     #     self.rot_pid[i].set_state(rot_vel[i])
    #     #     self.rot_pid[i].update(rpy_vel_targets[i], dt)

    #     # self.thrust_list[0] = -self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
    #     # self.thrust_list[1] = self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
    #     # self.thrust_list[2] = self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
    #     # self.thrust_list[3] = -self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
    #     # self.thrust_list[4] = -self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) - self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])
    #     # self.thrust_list[5] = self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) - self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])
    #     # self.thrust_list[6] = -self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) +  self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])
    #     # self.thrust_list[7] = self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) + self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])

    #     self.turn_pid.set_state(euler_from_quaternion(self.rot_quat)[2])
    #     self.turn_pid.update(1,dt) # radians

    #     #self.depth_pid.set_state(PUT THE DEPTH READING HERE)
    #     #self.depth_pid.update(depth target, dt)

    #     forward_bias = 0
    #     vertical_bias = -20 #vertical_bias = self.depth_pid.get_output()
    #     self.thrust_list[0] = -self.turn_pid.get_output() + forward_bias# 0 and 3 match and 1 and 2 match
    #     self.thrust_list[1] = self.turn_pid.get_output() + forward_bias
    #     self.thrust_list[2] = self.turn_pid.get_output() + forward_bias
    #     self.thrust_list[3] = -self.turn_pid.get_output() + forward_bias
    #     self.thrust_list[4] = vertical_bias
    #     self.thrust_list[5] = vertical_bias
    #     self.thrust_list[6] = vertical_bias
    #     self.thrust_list[7] = vertical_bias


        


    #     # self.kf.predict(u=self.thrust_list)
    #     self.kf.predict()
    #     self.kf.update(self.Z)

    def compRun(self, dt, targetGoal):

        A = np.array([  
                [1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0, 0.5*dt**2],
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dt],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
        
        self.R = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,self.ori_cov[0],self.ori_cov[1],self.ori_cov[2],0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,self.ori_cov[3],self.ori_cov[4],self.ori_cov[5],0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,self.ori_cov[6],self.ori_cov[7],self.ori_cov[8],0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,self.ang_cov[0],self.ang_cov[1],self.ang_cov[2],0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,self.ang_cov[3],self.ang_cov[4],self.ang_cov[5],0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,self.ang_cov[6],self.ang_cov[7],self.ang_cov[8],0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,self.acc_cov[0],self.acc_cov[1],self.acc_cov[2],0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,self.acc_cov[3],self.acc_cov[4],self.acc_cov[5],0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,self.acc_cov[6],self.acc_cov[7],self.acc_cov[8],0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])

        self.kf.F = A

        B = np.array([  [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0],
                [-0.5, 0.5, 0.5, -0.5, 0, 0, 0, 0],
                [-0.8660251013910845, -0.8660257817756856, 0.8660253281861298, 0.8660255549809972,0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1, 1, 1, 1],
                [0.10219096196414795, 0.10219104224953088, -0.1021909887259633, -0.10219101548775772, 0.2, 0.2, -0.2, -0.2],
                [-0.059000061803713866, 0.0589999227453273, 0.0590000154509305, -0.058999969098135, 0.25, -0.25, 0.25, -0.25],
                [0.03268559890777384, -0.03268614782371279, 0.032685781879760206, -0.032535964930304084, 0, 0, 0, 0]])
                
        
        #rot_vel = self.kf.x[9:12]
        
        rot_vel = self.Z[3:6]


        rpy_vel_targets = self.qp_attitude_controller.get_angular_setpoint(self.rot_target, self.rot_quat)
        
        #rospy.loginfo(self.thrust_list) 
        
        #rospy.loginfo(rpy_vel_targets)

        # for i in range(3):
        #     self.rot_pid[i].set_state(rot_vel[i])
        #     self.rot_pid[i].update(rpy_vel_targets[i], dt)

        # self.thrust_list[0] = -self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
        # self.thrust_list[1] = self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
        # self.thrust_list[2] = self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
        # self.thrust_list[3] = -self.rot_pid[2].get_output_ff(KF_ANGULAR, rpy_vel_targets[2])
        # self.thrust_list[4] = -self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) - self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])
        # self.thrust_list[5] = self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) - self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])
        # self.thrust_list[6] = -self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) +  self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])
        # self.thrust_list[7] = self.rot_pid[0].get_output_ff(KF_ANGULAR, rpy_vel_targets[0]) + self.rot_pid[1].get_output_ff(KF_ANGULAR, rpy_vel_targets[1])

        self.turn_pid.set_state(euler_from_quaternion(self.rot_quat)[2])
        self.turn_pid.update(targetGoal,dt) # radians

        #self.depth_pid.set_state(PUT THE D EPTH READING HERE)
        #self.depth_pid.update(depth target, dt)

        forward_bias = 0
        vertical_bias = -15.1 #vertical_bias = self.depth_pid.get_output()
        self.thrust_list[0] = -self.turn_pid.get_output() + forward_bias# 0 and 3 match and 1 and 2 match
        self.thrust_list[1] = self.turn_pid.get_output() + forward_bias
        self.thrust_list[2] = self.turn_pid.get_output() + forward_bias
        self.thrust_list[3] = -self.turn_pid.get_output() + forward_bias
        self.thrust_list[4] = vertical_bias - 3
        self.thrust_list[5] = vertical_bias - 3.4
        self.thrust_list[6] = vertical_bias + 3
        self.thrust_list[7] = vertical_bias + 3
        # self.thrust_list[4] = 0
        # self.thrust_list[5] = 0
        # self.thrust_list[6] = 0
        # self.thrust_list[7] = 0

        # self.kf.predict(u=self.thrust_list)
        self.kf.predict()
        self.kf.update(self.Z)


    def enter(self):
        pass

    def exit(self):
        pass

    def get_thrust_list(self):
        return self.thrust_list

