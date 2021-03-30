from cv2 import KalmanFilter
import numpy as np
import quaternion

CV_32F = 5
CV_64F = 6

class Filter(KalmanFilter):
    def __init__(self, num_states, num_meas, num_control=0, type=CV_64F):
        super().__init__(num_states, num_meas, num_control, type=type)
        
        self.num_states = num_states
        self.num_meas = num_meas
        self.num_control = num_control
        
        self.set_initial_states()
        self.set_initial_calibration()
        
        self.set_initial_error_states()
        
    def set_initial_states(self):
        self.p = np.array([0., 0., 0.])
        self.v = np.array([0., 0., 0.])
        self.q = np.quaternion(1, 0, 0, 0)
        
        self.bw = np.array([0., 0., 0.1])
        self.ba = np.array([0.21, 0., 0.])
        
        self.scale = 1.
        
        self.p_ic = np.array([0., 0., 0.])
        self.q_ic = np.quaternion(1, 0, 0, 0)
        
    def set_initial_error_states(self):
        self.dp = np.array([0., 0., 0.])
        self.dv = np.array([0., 0., 0.])
        self.qerr = np.array([0., 0., 0.])
        
        self.dbw = np.array([0., 0., 0.])
        self.dba = np.array([0., 0., 0.])
        
        self.dscale = 0.
        
        self.dp_ic = np.array([0., 0., 0.])
        self.qerr_ic = np.array([0., 0., 0.])

        self.statePost = np.hstack((self.dp, self.dv, self.qerr, \
                self.dbw, self.dba, self.dscale, self.dp_ic, self.qerr_ic))
        self.errorCovPost = np.eye(self.num_states, self.num_states) * 2.
        
    def set_initial_calibration(self):
        self.g = np.array([0., 0., 9.81])
        self.sq_sigma_omega = 1.
        self.sq_sigma_a = 1.
        self.sq_sigma_b_omega = 0.1
        self.sq_sigma_b_a = 0.1
        self.dt = 1/400.0
        
    def set_calibration(self, sq_sigma_omega, sq_sigma_a,
            sq_sigma_b_omega, sq_sigma_b_a, dt, g):
        self.g = g
        self.sq_sigma_omega = sq_sigma_omega
        self.sq_sigma_a = sq_sigma_a
        self.sq_sigma_b_omega = sq_sigma_b_omega
        self.sq_sigma_b_a = sq_sigma_b_a
        self.dt = dt
        
        
        