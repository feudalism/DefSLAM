import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import rowan # (w, x, y, z)
import quaternion
from quaternion.numba_wrapper import xrange

def parse(filepath, data_labels):
    data_containers = {}
    num_labels = len(data_labels)
    
    for label in data_labels:
        data_containers[label] = []

    with open(filepath, 'r') as f:
        for i, line in enumerate(f):
            data = line.split()
            ts = float(data[0])
            
            for j, label in enumerate(data_labels):
                if label == 'ts':
                    data_containers['ts'].append(ts)
                    continue
                
                if j == (num_labels - 1):
                    meas = float(data[j].rstrip())
                else:
                    meas = float(data[j])
                    
                data_containers[label].append(meas)

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

def generate_raw_imu_data(filepath, data_labels):
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
    
    quat_array = np.zeros((len(t),), dtype=np.quaternion)
    qinv_array = np.zeros((len(t),), dtype=np.quaternion)
    for i, _ in enumerate(t):
        x = data_containers['qx'][i]
        y = data_containers['qy'][i]
        z = data_containers['qz'][i]
        w = data_containers['qw'][i]
        
        quat = np.quaternion(w, x, y, z)
        qinv = quaternion.quaternion.inverse(quat)
        quat_array[i] = quat
        qinv_array[i] = qinv
    
    print(qinv_array)
    qd_array = quat_derivative(quat_array, t)
    qdd_array = quat_derivative(qd_array, t)
    
    # https://math.stackexchange.com/questions/1792826/estimate-angular-velocity-and-acceleration-from-a-sequence-of-rotations
    w = 2 * qd_array * qinv_array
    print(w)
    
def get_acceleration(position, dt):
    vx = np.gradient(position, dt)
    return np.gradient(vx, dt)
    
def quat_derivative(f, t):
    # copied from quaternion source code: calculus.py
    
    dfdt = np.empty_like(f)
    
    for i in xrange(2):
        t_i = t[i]
        t1 = t[0]
        t2 = t[1]
        t3 = t[2]
        t4 = t[3]
        t5 = t[4]
        h1 = t1 - t_i
        h2 = t2 - t_i
        h3 = t3 - t_i
        h4 = t4 - t_i
        h5 = t5 - t_i
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        dfdt[i] = (-((h2 * h3 * h4 + h2 * h3 * h5 + h2 * h4 * h5 + h3 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[0]
                   + ((h1 * h3 * h4 + h1 * h3 * h5 + h1 * h4 * h5 + h3 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[1]
                   - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[2]
                   + ((h1 * h2 * h3 + h1 * h2 * h5 + h1 * h3 * h5 + h2 * h3 * h5) / (h14 * h24 * h34 * h45)) * f[3]
                   - ((h1 * h2 * h3 + h1 * h2 * h4 + h1 * h3 * h4 + h2 * h3 * h4) / (h15 * h25 * h35 * h45)) * f[4])

    for i in xrange(2, len(t) - 2):
        t1 = t[i - 2]
        t2 = t[i - 1]
        t3 = t[i]
        t4 = t[i + 1]
        t5 = t[i + 2]
        h1 = t1 - t3
        h2 = t2 - t3
        h4 = t4 - t3
        h5 = t5 - t3
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        dfdt[i] = (-((h2 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[i - 2]
                   + ((h1 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[i - 1]
                   - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[i]
                   + ((h1 * h2 * h5) / (h14 * h24 * h34 * h45)) * f[i + 1]
                   - ((h1 * h2 * h4) / (h15 * h25 * h35 * h45)) * f[i + 2])

    for i in xrange(len(t) - 2, len(t)):
        t_i = t[i]
        t1 = t[-5]
        t2 = t[-4]
        t3 = t[-3]
        t4 = t[-2]
        t5 = t[-1]
        h1 = t1 - t_i
        h2 = t2 - t_i
        h3 = t3 - t_i
        h4 = t4 - t_i
        h5 = t5 - t_i
        h12 = t1 - t2
        h13 = t1 - t3
        h14 = t1 - t4
        h15 = t1 - t5
        h23 = t2 - t3
        h24 = t2 - t4
        h25 = t2 - t5
        h34 = t3 - t4
        h35 = t3 - t5
        h45 = t4 - t5
        dfdt[i] = (-((h2 * h3 * h4 + h2 * h3 * h5 + h2 * h4 * h5 + h3 * h4 * h5) / (h12 * h13 * h14 * h15)) * f[-5]
                   + ((h1 * h3 * h4 + h1 * h3 * h5 + h1 * h4 * h5 + h3 * h4 * h5) / (h12 * h23 * h24 * h25)) * f[-4]
                   - ((h1 * h2 * h4 + h1 * h2 * h5 + h1 * h4 * h5 + h2 * h4 * h5) / (h13 * h23 * h34 * h35)) * f[-3]
                   + ((h1 * h2 * h3 + h1 * h2 * h5 + h1 * h3 * h5 + h2 * h3 * h5) / (h14 * h24 * h34 * h45)) * f[-2]
                   - ((h1 * h2 * h3 + h1 * h2 * h4 + h1 * h3 * h4 + h2 * h3 * h4) / (h15 * h25 * h35 * h45)) * f[-1])

    return dfdt

        

FILEPATH = "./Apps/traj_mandala0_gt.txt"
data_labels = ['ts', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
generate_raw_imu_data(FILEPATH, data_labels)