import matplotlib.pyplot as plt
from traj_parser import parse, plot, add_noise, imu_interpolate

traj_data = {'stereoGT': "./Apps/traj_mandala0_gt.txt",
            'imu': "./Apps/traj_mandala0_gt_imu.txt",
            'noisy' : "./Apps/traj_mandala0_gt_imu_noisy.txt",
            }
data_labels = ['ts', 'x', 'y', 'z', 'q1', 'q2', 'q3', 'q4']
filename_imu = imu_interpolate(traj_data['stereoGT'], data_labels, 
    num_imu_between_frames = 10)
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
    
plt.legend()
plt.show()