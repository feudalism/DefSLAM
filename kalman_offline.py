import cv2
import numpy as np
from math import factorial
import quaternion
import matplotlib.pyplot as plt

from traj_parser import plot
from aux import load_data
from Measurement import VisualMeasurement
from Trajectory import parse, Trajectory
from Filter import Filter

# load data
DefSLAM, IMU = load_data()
dt0 = IMU.ts[1] - IMU.ts[0]

# kalman filter (uses error states)
num_states = 22
num_meas = 7
num_control = 12
kf = Filter(num_states, num_meas, num_control)
kf.set_calibration(0.002/400,			# sq_sigma_omega
        0.05/400,						# sq_sigma_a
        0.01,							# sq_sigma_b_omega
        0.01,							# sq_sigma_b_a
        dt0,						    # Delta_t
        np.array([0,0,9.82]))			# g

# main filtering loop
DefSLAM_init = DefSLAM.at_index(0)
first_imu_idx = IMU.get_first_index(DefSLAM_init.t)
last_DefSLAM_idx = 0

traj = Trajectory("random walk")
traj_corr = Trajectory("kalman")

last_t = IMU.ts[0]
last_ds_idx = 0
for i, t in enumerate(IMU.ts[0:400]):
    # print(f"t={t}, {last_ds_idx}: {DefSLAM.at_index(last_ds_idx).t}")

    # # IMU measurements arrive -- propagate
    current_imu = IMU.at_index(i)
    kf.dt = current_imu.t - last_t
    
    if i == 0:
        # # initial states from DefSLAM
        pos = DefSLAM_init.pos
        qrot = DefSLAM_init.qrot
        kf.set_states(pos, qrot)
        traj.append_data(t, kf.statePost[0:7])
        last_ds_idx += 1
        continue
          
    kf.predict(current_imu.acc, current_imu.om)
    traj.append_data(t, kf.statePost[0:7])
    
    # image measurements arrive -- update/correct
    t_ds = DefSLAM.at_index(last_ds_idx).t
    if t_ds <= t:
        # print(f"DefSLAM ts: {t_ds} <= t: {t}\n" +
            # "New image detected, performing measurement update")
            
        current_ds_idx = last_ds_idx
        last_ds_idx += 1
        
        current_defslam = DefSLAM.at_index(current_ds_idx)
        kf.correct(current_defslam.vec)
        traj_corr.append_data(t, kf.statePost[0:7])
    
axes = None
axes = traj.plot(axes)
axes = traj_corr.plot(axes)

plt.legend()
plt.show()