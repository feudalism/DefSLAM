import os

import numpy as np
from scipy.interpolate import interp1d
import quaternion
from quaternion.numba_wrapper import xrange

import matplotlib.pyplot as plt

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
        axes[row][col].legend()

    return axes
    
def imu_interpolate(data_containers, num_imu_between_frames):
    tmin = data_containers['ts'][0]
    tmax = data_containers['ts'][-1]
    num_cam_datapoints = len(data_containers['ts'])
    
    num_imu_datapoints = (num_cam_datapoints - 1) * num_imu_between_frames + 1
    t_imu = np.linspace(tmin, tmax, num=num_imu_datapoints)
    
    for label, data in data_containers.items():
        if label == 'ts':
            continue
        
        f = interp1d(data_containers['ts'], data, kind='linear')
        data_containers[label] = f(t_imu)
    
    data_containers['ts'] = t_imu
    
    return data_containers
    
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

            noise_a = 0.0025 * np.random.randn(3) - 0.00025
            noise_gx = 0.05 * np.random.randn(1) - 0.025
            noise_gy = 0.005 * np.random.randn(1) - 0.00025
            noise_gz = 0.05 * np.random.randn(1) - 0.025
            noisy_data = npdata + np.hstack((noise_a, noise_gx, noise_gy, noise_gz))

            noisy_data_str = np.array2string(noisy_data,
                    formatter={'float_kind':lambda x: "%.9f" % x}, max_line_width=200)
            noisy_data_str = noisy_data_str[1:-1]
            file_noisy.write(noisy_data_str + '\n')

    file_noisy.close()
    
    return filename_noisy

def generate_raw_imu_data(filepath, data_labels, num_imu_between_frames=2):
    filename, ext = os.path.splitext(filepath)
    filename_imuraw = filename + '_imuraw' + ext
    
    data_containers = parse(filepath, data_labels)
    
    raw_data_containers = {}
    raw_data_labels = ['ts', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
    
    t = data_containers['ts']
    raw_data_containers['ts'] = t
    dt = t[1] - t[0]
        
    raw_data_containers['ax'] = get_acceleration(data_containers['x'], dt)
    raw_data_containers['ay'] = get_acceleration(data_containers['y'], dt)
    raw_data_containers['az'] = get_acceleration(data_containers['z'], dt)
    
    rx, ry, rz = get_euler_angles_from_quats(data_containers)
    raw_data_containers['gx'] = np.gradient(rx, dt)
    raw_data_containers['gy'] = np.gradient(ry, dt)
    raw_data_containers['gz'] = np.gradient(rz, dt)
    
    interp_data_containers = imu_interpolate(raw_data_containers, num_imu_between_frames)

    with open(filename_imuraw, 'w+') as f:
        for i, t in enumerate(raw_data_containers['ts']):
            ax = raw_data_containers['ax'][i]
            ay = raw_data_containers['ay'][i]
            az = raw_data_containers['az'][i]
            gx = raw_data_containers['gx'][i]
            gy = raw_data_containers['gy'][i]
            gz = raw_data_containers['gz'][i]
            data_str = f"{t:.6f} {ax:.9f} {ay:.9f} {az:.9f} {gx:.9f} {gy:.9f} {gz:.9f}"
            f.write(data_str + '\n')
            
    return filename_imuraw
    
def get_acceleration(position, dt):
    vx = np.gradient(position, dt)
    return np.gradient(vx, dt)
    
def get_euler_angles_from_quats(data_containers):
    len_t = len(data_containers['ts'])
    rx = np.zeros((len_t,))
    ry = np.zeros((len_t,))
    rz = np.zeros((len_t,))
    
    for i in range(len_t):
        x = data_containers['qx'][i]
        y = data_containers['qy'][i]
        z = data_containers['qz'][i]
        w = data_containers['qw'][i]
        
        quat = np.quaternion(w, x, y, z)
        rx[i], ry[i], rz[i] = quaternion.as_euler_angles(quat)
        
    return rx, ry, rz
