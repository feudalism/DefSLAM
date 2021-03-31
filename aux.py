import numpy as np
from math import factorial

from Trajectory import parse, VisualTraj, ImuTraj

def Hp(rot, scale):
    rot.T * scale
    
def load_data():
    # load images (this simulates a monocular defslam run)
    defslam_traj = "./Apps/traj_mandala0_mono.txt"
    DefSLAM = VisualTraj(defslam_traj)

    # load imu (generated from stereo defslam trajectory)
    imu_raw = "./Apps/traj_mandala0_gt_imuraw_noisy.txt"
    Imu = ImuTraj(imu_raw)
    
    return DefSLAM, Imu