from cv2 import KalmanFilter
import numpy as np
import quaternion
from math import factorial

CV_32F = 5
CV_64F = 6

def skew(x):
    return np.array([[0,    -x[2], x[1]],
                     [x[2],    0, -x[0]],
                     [-x[1], x[0],    0]])

class SimpleFilter(KalmanFilter):
    def __init__(self, num_states, num_meas, num_control=0, type=CV_64F):
        super().__init__(num_states, num_meas, num_control, type=type)
        
        self.num_states = num_states
        self.num_meas = num_meas
        self.num_control = num_control
        
        self.dt = None
        
        self.p = None
        self.v = None
        self.q = None
        
        self.sig_p = None
        self.sig_v = None
        self.sig_q = None
        
    @property
    def states(self):
        return np.hstack((self.p, self.v, self.q))
        
    def set_states(self, p, v, q):
        self.p = p
        self.v = v
        self.q = q
        self.rot = quaternion.as_rotation_matrix(q)
        
    @property
    def covariance(self):
        return np.hstack((self.sig_p, self.sig_v, self.sig_q))
    
    def set_covariance(self, s_p, s_v, s_om):
        self.sig_p = s_p
        self.sig_v = s_v
        self.sig_q = s_om
        
        sig_p = np.ones((3, )) * s_p
        sig_v = np.ones((3, )) * s_v
        sig_om = np.ones((3, )) * s_om
        
        stack = np.hstack((sig_p, sig_v, sig_om))
        self.P = np.diag(stack)

    def propagate(self, a_meas, om_meas):
        Omega = np.quaternion(0, om_meas[0], om_meas[1], om_meas[2])
        
        self.rot = np.eye(3, 3) # TEMP
        
        
        self.v += self.rot @ a_meas
        self.p += self.dt * self.v

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
        
    def set_states(self, p, q):
        self.p = p
        self.q = q
        
    def set_calibration(self, sq_sigma_omega, sq_sigma_a,
            sq_sigma_b_omega, sq_sigma_b_a, dt, g):
        self.g = g
        self.sq_sigma_omega = sq_sigma_omega
        self.sq_sigma_a = sq_sigma_a
        self.sq_sigma_b_omega = sq_sigma_b_omega
        self.sq_sigma_b_a = sq_sigma_b_a
        self.dt = dt
        
    def update_transition_matrix(self, acc, om):
        Fd = np.eye(self.num_states, self.num_states)
        rot = quaternion.as_rotation_matrix(self.q)
        
        dt = self.dt
        delt2 = dt**2/2
        delt3 = dt**3/factorial(3)
        delt4 = dt**4/factorial(4)
        delt5 = dt**5/factorial(5)
        
        acc_sk = skew(acc)
        om_sk = skew(om)
        om_sk_sq = om_sk * om_sk
        
        A = -rot.T * acc_sk * ( delt2 - delt3 * om_sk + delt4 * om_sk_sq )
        B = -rot.T * acc_sk * ( delt3 + delt4 * om_sk - delt5 * om_sk_sq )
        C = -rot.T * acc_sk * ( dt    - delt2 * om_sk + delt3 * om_sk_sq )
        D = -A
        E = np.eye(3, 3) - dt * om_sk + delt2 * om_sk_sq
        F = -dt + delt2 * om_sk - delt3 * om_sk_sq
        
        Fd[0:3, 3:6] = np.eye(3, 3) * dt
        Fd[0:3, 6:9] = A
        Fd[0:3, 9:12] = B
        Fd[0:3, 12:15] = -rot * delt2
        
        Fd[3:6, 6:9] = C
        Fd[3:6, 9:12] = D
        Fd[3:6, 12:15] = -rot * dt
        
        Fd[6:9, 6:9] = E
        Fd[6:9, 9:12] = F
        
        self.transitionMatrix = Fd
        
    def update_control_matrix(self):
        Gc = np.zeros((self.num_states, self.num_control))
        rot = quaternion.as_rotation_matrix(self.q)
        
        Gc[3:6, 0:3] = -rot.T
        Gc[6:9, 6:9] = -np.eye(3, 3)
        Gc[9:12, -3:] = np.eye(3, 3)
        Gc[12:15, 3:6] = np.eye(3, 3)
        
        self.controlMatrix = self.dt * Gc
        
    def update_states_from_error_states(self):
        self.p += self.dp
        self.v += self.dv
        
        self.bw += self.dbw
        self.ba += self.dba
        
        self.scale += self.dscale
        
        self.p_ic += self.dp_ic
        
    def predict(self, a_meas, om_meas):
        control = np.float64( \
            np.random.normal(loc=0., scale=1.0, size=(self.num_control,)))
        
        self.update_transition_matrix(a_meas, om_meas)
        self.update_control_matrix()
        
        super().predict(control)
        self.update_states_from_error_states()
        
    def correct(self, measurement):
        super().correct(measurement)
        self.update_states_from_error_states()
        
        
        
        