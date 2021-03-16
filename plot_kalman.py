import matplotlib.pyplot as plt
from traj_parser import parse, plot

traj_data = {
            'stereoGT': "./Apps/traj_mandala0_gt.txt",
            # 'measurements': "./Apps/traj_mandala0_gt_imu_noisy.txt",
            'rw': "./Apps/trajectory_offline_rw.txt",
            'kalman' : "./Apps/trajectory_offline.txt",
            'mono' : "./Apps/traj_mandala0_mono.txt",
            }
data_labels = ['ts', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    
axes = None
for label, filepath in traj_data.items():
    data_containers = parse(filepath, data_labels)
    t_arr = data_containers['ts']
    
    if label == 'stereoGT' or label == 'measurements':
        min_t = min(t_arr)
        max_t = max(t_arr)
    elif label != 'rw':
        min_t = min([min_t, min(t_arr)])
        max_t = min([max_t, max(t_arr)])
    
    axes = plot(t_arr, data_containers, data_labels, label, min_t, max_t, axes=axes)
    
plt.legend()
plt.show()