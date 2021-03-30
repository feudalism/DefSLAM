import numpy as np
from math import factorial

def skew(x):
    return np.array([[0,    -x[2], x[1]],
                     [x[2],    0, -x[0]],
                     [-x[1], x[0],    0]])
                     
def Fd(num_states, dt, rot, acc, om):
    Fd = np.eye(num_states, num_states, dtype=np.float32)
    
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
    
    return Fd

def Gc(num_states, num_control, rot):
    Gd = np.zeros((num_states, num_control), dtype=np.float32)
    Gd[3:6, 0:3] = -rot.T
    Gd[6:9, 6:9] = -np.eye(3, 3)
    Gd[9:12, -3:] = np.eye(3, 3)
    Gd[12:15, 3:6] = np.eye(3, 3)
    return Gd

def Hp(rot, scale):
    rot.T * scale