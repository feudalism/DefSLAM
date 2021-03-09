import matplotlib.pyplot as plt
from traj_parser import parse, plot

traj_data = {'stereoGT': "./Apps/traj_mandala0_gt.txt",
            'measurements': "./Apps/traj_mandala0_gt_noisy.txt",
            'rw': "./Apps/trajectory_rw.txt",
            'kalman' : "./Apps/trajectory.txt",
            'mono' : "./Apps/traj_mandala0_mono.txt"}
data_labels = ['ts', 'x', 'y', 'z', 'q1', 'q2', 'q3', 'q4']
    
axes = None
for label, filepath in traj_data.items():
    data_containers = parse(filepath, data_labels)
    t_arr = data_containers['ts']
    
    if label == 'stereoGT':
        min_t = min(t_arr)
        max_t = max(t_arr)
    elif label != 'rw':
    # else:
        min_t = min([min_t, min(t_arr)])
        max_t = min([max_t, max(t_arr)])
    
    axes = plot(t_arr, data_containers, data_labels, label, min_t, max_t, axes=axes)
    
plt.legend()
plt.show()