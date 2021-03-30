import cv2
import numpy as np
from math import factorial
import quaternion
import matplotlib.pyplot as plt

from traj_parser import plot
from aux import load_data, skew, Fd, Gc
from Measurement import VisualMeasurement
from Trajectory import parse
from Filter import Filter

# load data
DefSLAM, IMU = load_data()

# kalman filter (uses error states)
num_states = 22
num_meas = 7
num_control = 12
kf = Filter(num_states, num_meas, num_control)
kf.set_calibration(0.002/400,			# sq_sigma_omega
        0.05/400,						# sq_sigma_a
        0.01,							# sq_sigma_b_omega
        0.01,							# sq_sigma_b_a
        1/400.0,						# Delta_t
        np.array([0,0,9.82]))			# g

# main filtering loop
ts_start = 220
ts_end = 310

DefSLAM_init = DefSLAM.at_index(0)
first_imu_idx = IMU.get_first_index(DefSLAM_init.t)

last_DefSLAM_idx = 0

traj = {}
traj_corr = {}
traj_labels = ['ts', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw']
for label in traj_labels:
    traj[label] = []
    traj_corr[label] = []
traj['ts'] = IMU.ts
# traj_corr['ts'] = DefSLAM['ts']

last_t = IMU.ts[0]
last_ds_idx = 0
acc = dacc0
for i, t in enumerate(IMU.ts):
    # # IMU measurements arrive -- propagate
    control = np.float64( \
        np.random.normal(loc=0., scale=1.0, size=(num_control,)))
        
    imu_meas = IMU.at_index(i)
    dt = imu_meas.t - last_t
    
    if i == 0:
        # # initial states
        pos = DefSLAM_init.pos
        rot = quaternion.as_rotation_matrix(DefSLAM_init.qrot)
    
    input()
    kf.transitionMatrix = Fd(num_states, dt, rot, acc, dom0)
    kf.controlMatrix = Gc(num_states, num_control, rot)   
    kf.predict(control)
    
    traj['px'].append(kf.statePost[0])
    traj['py'].append(kf.statePost[1])
    traj['pz'].append(kf.statePost[2])
    
    traj['qx'].append(kf.statePost[3])
    traj['qy'].append(kf.statePost[4])
    traj['qz'].append(kf.statePost[5])
    traj['qw'].append(kf.statePost[6])
    
    # # image measurements arrive -- update/correct
    ts_ds = DefSLAM.at_index(last_ds_idx).t
    
    if ts_ds <= t:
        # print(f"DefSLAM ts: {ts_ds} <= t: {t}")
        # print("New image detected, performing measurement update")
        current_ds_idx = last_ds_idx
        last_ds_idx += 1
        
        measurement = DefSLAM.at_index(current_ds_idx).vec        
        kf.correct(measurement)
    
        traj_corr['ts'].append(t)
        traj_corr['px'].append(kf.statePost[0])
        traj_corr['py'].append(kf.statePost[1])
        traj_corr['pz'].append(kf.statePost[2])
        
        traj_corr['qx'].append(kf.statePost[3])
        traj_corr['qy'].append(kf.statePost[4])
        traj_corr['qz'].append(kf.statePost[5])
        traj_corr['qw'].append(kf.statePost[6])
        
    w = kf.statePost[6]
    x = kf.statePost[3]
    y = kf.statePost[4]
    z = kf.statePost[5]
    quat = np.quaternion(w, x, y, z)
    rot = quaternion.as_euler_angles(quat)
    
    # update states for next iteration
    last_t = imu_meas.t
        
    # input("----debug---pause----")
    
    
    # break
    
# axes = None
# axes = plot(traj['ts'], traj, traj_labels, 'rw', ts_start, ts_end, axes=axes)
# axes = plot(traj_corr['ts'], traj_corr, traj_labels, 'kalman', ts_start, ts_end, axes=axes)

# plt.legend()
# plt.show()