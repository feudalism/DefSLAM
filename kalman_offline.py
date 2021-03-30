import cv2
import numpy as np
from math import factorial
import quaternion
import matplotlib.pyplot as plt

from traj_parser import parse, plot
from aux import skew, Fd, Gc

# load images (this simulates a monocular defslam run)
defslam_traj = "./Apps/traj_mandala0_mono.txt"
defslam_labels = ['ts', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
defslam = parse(defslam_traj, defslam_labels)
nImages = len(defslam['ts'])

# load imu (generated from stereo defslam trajectory)
imu_raw = "./Apps/traj_mandala0_gt_imuraw_noisy.txt"
imu_labels = ['ts', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
imu = parse(imu_raw, imu_labels)

# kalman filter (uses error states)
num_states = 22
num_meas = 7
num_control = 12
kf = cv2.KalmanFilter(num_states, num_meas, num_control)

# # initial states

# # initial error states
dt = imu['ts'][1] - imu['ts'][0]
dpos0 = np.array([0., 0., 0.], dtype=np.float32)
dvel0 = np.array([0., 0., 0.], dtype=np.float32)
drot0 = np.eye(3, 3, dtype=np.float32)
err_quat0 = np.array([0., 0., 0.], dtype=np.float32)
dbw0 = np.array([0.1, 0.1, 0.1], dtype=np.float32)
dba0 = np.array([0.21, 0.12, 0.21], dtype=np.float32)
dscale0 = np.float32(0.)
dtrans_imu_cam0 = np.array([0., 0., 0.], dtype=np.float32)
drot_imu_cam0 = np.eye(3, 3, dtype=np.float32)
err_quat_imu_cam0 = np.array([0., 0., 0.], dtype=np.float32)
dacc0 = np.array([0., 0., 0.], dtype=np.float32)
dom0 = np.array([0., 0, 0.], dtype=np.float32)

kf.statePost = np.hstack((dpos0, dvel0, err_quat0, \
        dbw0, dba0, dscale0, dtrans_imu_cam0, err_quat_imu_cam0))
kf.errorCovPost = np.eye(num_states, num_states, dtype=np.float32) * 2.
        
kf.transitionMatrix = Fd(num_states, dt, drot0, dacc0, dom0)
kf.controlMatrix = Gc(num_states, num_control, drot0)

# main filtering loop
ts_start = 220
ts_end = 310

first_imu_idx = 0
while imu['ts'][first_imu_idx] <= defslam['ts'][0]:
    first_imu_idx += 1
first_imu_idx -= 1

last_defslam_idx = 0
last_defslam_ts = defslam['ts'][last_defslam_idx]

traj = {}
traj_corr = {}
traj_labels = ['ts', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw']
for label in traj_labels:
    traj[label] = []
    traj_corr[label] = []
traj['ts'] = imu['ts']
# traj_corr['ts'] = defslam['ts']

last_ds_idx = 0
rot = drot0
acc = dacc0
for t in imu['ts']:

    # # imu measurements arrive -- propagate
    control = np.float32( \
        np.random.normal(loc=0., scale=1.0, size=(num_control,))) 
    kf.transitionMatrix = Fd(num_states, dt, rot, acc, dom0)
    kf.controlMatrix = Gc(num_states, num_control, drot0)   
    kf.predict(control)
    
    traj['px'].append(kf.statePost[0])
    traj['py'].append(kf.statePost[1])
    traj['pz'].append(kf.statePost[2])
    
    traj['qx'].append(kf.statePost[3])
    traj['qy'].append(kf.statePost[4])
    traj['qz'].append(kf.statePost[5])
    traj['qw'].append(kf.statePost[6])
    
    # # image measurements arrive -- update/correct
    ts_ds = defslam['ts'][last_ds_idx]
    if ts_ds <= t:
        # print(f"DefSLAM ts: {ts_ds} <= t: {t}")
        # print("New image detected, performing measurement update")
        current_ds_idx = last_ds_idx
        last_ds_idx += 1
        
        measurement = np.zeros((num_meas, ), dtype=np.float32)
        measurement[0] = defslam['x'][current_ds_idx]
        measurement[1] = defslam['y'][current_ds_idx]
        measurement[2] = defslam['z'][current_ds_idx]
        measurement[3] = defslam['qx'][current_ds_idx]
        measurement[4] = defslam['qy'][current_ds_idx]
        measurement[5] = defslam['qz'][current_ds_idx]
        measurement[6] = defslam['qw'][current_ds_idx]
        
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
        
    # input("----debug---pause----")
    
    
    # break
    
axes = None
axes = plot(traj['ts'], traj, traj_labels, 'rw', ts_start, ts_end, axes=axes)
axes = plot(traj_corr['ts'], traj_corr, traj_labels, 'kalman', ts_start, ts_end, axes=axes)

plt.legend()
plt.show()