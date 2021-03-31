import numpy as np
import matplotlib.pyplot as plt

from aux import load_data
from Filter import SimpleFilter
from Trajectory import Trajectory

# load data
slam, imu = load_data()

# kalman filter
num_states = 10
num_meas = 7
num_control = 6
kf = SimpleFilter(num_states, num_meas, num_control)

# noise
sigma_acc = 0.2
sigma_om = 0.02

# main loop
traj = Trajectory("slam")
for i, t in enumerate(slam.ts):
    current_slam = slam.at_index(i)
    traj.append_data(t, current_slam.vec)
    
    # initial states
    if i == 0:
        p0 = current_slam.pos
        v0 = np.array([0., 0., 0.])
        q0 = current_slam.qrot
        
        s_p = sigma_acc
        s_v = sigma_acc
        s_q = sigma_om
        
        kf.set_states(p0, v0, q0)
        kf.set_covariance(s_p, s_v, s_q)
        
    # propagate
    imu_queue = imu.get_imu_queue(t)
    for j in range(imu_queue.t.size):
        current_imu = imu_queue.at_index(j)
        kf.dt = current_imu.t
        kf.propagate(current_imu.acc, current_imu.om)
    
traj.plot(axes=None)

# plt.legend()
# plt.show()