import matplotlib.pyplot as plt
from traj_parser import plot, add_noise, generate_raw_imu_data

from Trajectory import parse

traj_data = {
            'stereoGT': "./Apps/traj_mandala0_gt.txt",
            }
            
raw_imu_data = {
            'raw': "./Apps/traj_mandala0_gt_imuraw.txt",
            'noisy': "./Apps/traj_mandala0_gt_imuraw_noisy.txt"
            }
            
data_labels = ['ts', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
raw_data_labels = ['ts', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']

filename_imu = generate_raw_imu_data(traj_data['stereoGT'], data_labels,
                num_imu_between_frames = 100)
add_noise(filename_imu)
    
axes = None
for label, filepath in traj_data.items():
    data_containers = parse(filepath, data_labels)
    t_arr = data_containers['ts']
    
    if label == 'stereoGT':
        min_t = min(t_arr)
        max_t = max(t_arr)
    else:
        min_t = min([min_t, min(t_arr)])
        max_t = max([max_t, max(t_arr)])
    
    axes = plot(t_arr, data_containers, data_labels, label, min_t, max_t, axes=axes)

axes = None
for label, filepath in raw_imu_data.items():
    data_containers = parse(filepath, raw_data_labels)
    t_arr = data_containers['ts']
    
    if label == 'raw':
        min_t = min(t_arr)
        max_t = max(t_arr)
    else:
        min_t = min([min_t, min(t_arr)])
        max_t = max([max_t, max(t_arr)])
    
    axes = plot(t_arr, data_containers, raw_data_labels, label, min_t, max_t, axes=axes)
    
plt.show()