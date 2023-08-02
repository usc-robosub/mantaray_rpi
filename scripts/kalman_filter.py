import numpy as np

class kalmanfilter():
    
    # A: state transition matrix
    # B: control input matrix
    # P: error covariance matrix
    # H: measurement matrix
    # X: state vector
    def __init__(self):
        self.X_ = np.zeros((18, 1))
        self.A_ = np.zeros((18,18))
        self.B_ = np.zeros((18,8))
        self.P_ = np.eye(18)
        
    # H: measurement matrix
    # U: control input
    # Z: measurement
    # Q: process noise covariance
    # R: measurement noise covariance
    # dt: time step
    def update(self, H, U, Z, Q, R, dt):
        
        # create transition matrix for a xyzrpy constant acceleration model
        self.A = np.array([  
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
        
        X_ = np.dot(self.A_, self.X_) + np.dot(self.B_, U) #project the state estimate
        P_ = (np.dot(np.dot(self.A_, self.P_), self.A_.transpose())) + Q #project the error covariance
        print(np.dot(H.transpose(),np.dot(H, P_))+R)
        K = np.dot(np.dot(P_, H.transpose()), np.linalg.inv(np.dot(H.transpose(),np.dot(H, P_))+R)) #compute the Kalman gain
        X_ = X_ + np.dot(K,(Z - np.dot(H,X_))) #update the estimate via z
        P_ = np.dot((np.eye(H.shape[0]) - np.dot(K,self.H_)),P_)
        
        self.X_ = X_
        self.P_ = P_

    
    def get_rpy_vel(self):
        return [self.X_[9],self.X_[10], self.X_[11]]

    def get_rpy(self):
        return [self.X_[3], self.X_[4], self.X_[5]]

    def get_xyz(self):
        return [self.X_[0], self.X_[1], self.X_[2]]

    def get_xyz_vel(self):
        return [self.X_[6], self.X_[7], self.X_[8]]
    




#create control input matrix based on thruster allocation matrix

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
                


# initialize P
P = np.eye(18)*0.1



# initialize R for the dvl
R_dvl = np.eye(3)*0.1


# initialize H for the DVL

H_dvl = np.array([  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0], # x velocity
                    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 ,0, 0, 0, 0, 0, 0, 0], # y velocity
                    [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 ,0, 0, 0, 0, 0, 0, 0]])# z velocity

              

