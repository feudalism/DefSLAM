import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def parse(filepath, data_labels):
    data_containers = {}
    for label in data_labels:
        data_containers[label] = []

    with open(filepath, 'r') as f:
        for i, line in enumerate(f):
            data = line.split()
            ts = float(data[0])

            data_containers['ts'].append(ts)
            data_containers['x'].append(float(data[1]))
            data_containers['y'].append(float(data[2]))
            data_containers['z'].append(float(data[3]))
            data_containers['q1'].append(float(data[4]))
            data_containers['q2'].append(float(data[5]))
            data_containers['q3'].append(float(data[6]))
            data_containers['q4'].append(float(data[7].rstrip()))

    # Convert list to numpy array
    for label in data_labels:
        data_containers[label] = np.asarray(data_containers[label])

    return data_containers

def plot(t, data, data_labels, file_label, min_t, max_t, offset=0, axes=None):
    if axes is None:
        fig, axes = plt.subplots(4, 2)
        fig.tight_layout()

    # raw data
    for i, label in enumerate(data_labels):
        if label == 'ts':
            continue

        ai = i - offset

        if ai <= 3:
            row = ai
            col = 0
        else:
            row = ai - 4
            col = 1

        if 'meas' in file_label or 'imu' in file_label:
            axes[row][col].scatter(t, data[label], color='black', marker="o", s=2., label=file_label)
        elif 'stereo' in file_label or 'mono' in file_label:
            axes[row][col].plot(t, data[label], linewidth=1., linestyle='--', label=file_label)
        elif 'rw' in file_label:
            axes[row][col].plot(t, data[label], linewidth=1., linestyle=':', label=file_label)
        else:
            axes[row][col].plot(t, data[label], linewidth=1., marker="o", markersize=1.5, label=file_label)
        axes[row][col].set_xlim(left=min_t, right=max_t)

        if len(label) == 1:
            latex_label = '$' + label + '$'
        else:
            latex_label = '$' + label[0] + '_' + label[1] + '$'

        axes[row][col].set_title(latex_label)
        axes[row][col].grid(True)

    return axes
    
def imu_interpolate(filepath, data_labels, num_imu_between_frames=2):
    filename, ext = os.path.splitext(filepath)
    filename_imu = filename + '_imu' + ext
    
    data_containers = parse(filepath, data_labels)
    
    tmin = data_containers['ts'][0]
    tmax = data_containers['ts'][-1]
    num_cam_datapoints = len(data_containers['ts'])
    
    num_imu_datapoints = (num_cam_datapoints - 1) * num_imu_between_frames + 1
    t_imu = np.linspace(tmin, tmax, num=num_imu_datapoints)
    
    
    for label in data_labels:
        if label == 'ts':
            continue
        
        f = interp1d(data_containers['ts'], data_containers[label], kind='linear')
        data_containers[label] = f(t_imu)
    
    data_containers['ts'] = t_imu

    with open(filename_imu, 'w+') as f:
        for i, t in enumerate(data_containers['ts']):
            x = data_containers['x'][i]
            y = data_containers['y'][i]
            z = data_containers['z'][i]
            q1 = data_containers['q1'][i]
            q2 = data_containers['q2'][i]
            q3 = data_containers['q3'][i]
            q4 = data_containers['q4'][i]
            data_str = f"{t:.6f} {x:.9f} {y:.9f} {z:.9f} {q1:.9f} {q2:.9f} {q3:.9f} {q4:.9f}"
            f.write(data_str + '\n')
            
    return filename_imu
    
def add_noise(filepath):
    filename, ext = os.path.splitext(filepath)
    filename_noisy = filename + '_noisy' + ext

    file_noisy = open(filename_noisy, 'w+')

    with open(filepath, 'r') as f:
        for i, line in enumerate(f):
            data = line.split()

            # time
            file_noisy.write(data[0] + ' ')
            data.pop(0)

            data = [float(i) for i in data]
            npdata = np.asarray(data, dtype=np.float64).flatten()

            noise_xyz = 0.005 * np.random.randn(3) - 0.00025
            noise_q = 0.005 * np.random.randn(4) - 0.00025
            noisy_data = npdata + np.hstack((noise_xyz, noise_q))

            noisy_data_str = np.array2string(noisy_data,
                    formatter={'float_kind':lambda x: "%.9f" % x}, max_line_width=200)
            noisy_data_str = noisy_data_str[1:-1]
            file_noisy.write(noisy_data_str + '\n')

    file_noisy.close()
    
    return filename_noisy


