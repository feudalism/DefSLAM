import numpy as np
import matplotlib.pyplot as plt
from scipy import integrate

MANDALA_IMU_FILEPATH = "/home/user3/slam/datasets/mandala0/imu.txt"
MANDALA_TS_FILEPATH = "/home/user3/slam/datasets/mandala0/timestamps/timestamps_copy.txt"



# Get first visual timestamp
FRAME_SKIP = 200
with open(MANDALA_TS_FILEPATH, 'r') as f:
    for i, data in enumerate(f):
        if i == 200:
            FIRST_FRAME_TS = int(data) * 1e-6 * 1e-3
            break

# Get raw IMU data
raw_data_labels = ['ts', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
raw_data_containers = {}
for label in raw_data_labels:
    raw_data_containers[label] = []

with open(MANDALA_IMU_FILEPATH, 'r') as f:
    for i, line in enumerate(f):
        data = line.split(',')
        
        ts = int(data[0]) * 1e-3
        
        if ts < FIRST_FRAME_TS:
            continue
        
        raw_data_containers['ts'].append(ts)
        raw_data_containers['ax'].append(float(data[1]))
        raw_data_containers['ay'].append(float(data[2]))
        raw_data_containers['az'].append(float(data[3]))
        raw_data_containers['gx'].append(float(data[4]))
        raw_data_containers['gy'].append(float(data[5]))
        raw_data_containers['gz'].append(float(data[6].rstrip()))

# Convert list to numpy array
for label in raw_data_labels:
    raw_data_containers[label] = np.asarray(raw_data_containers[label])

def plot_data(t, data_containers, labels, offset=0, nrows=4):
    fig, axes = plt.subplots(nrows, 2)

    # raw data
    for i, label in enumerate(labels):
        if label == 'ts':
            continue
            
        ai = i - offset
            
        if ai <= (nrows - 1):
            row = ai
            col = 0
        else:
            row = ai - nrows
            col = 1
        
        axes[row][col].plot(t, data_containers[label])
        axes[row][col].set_xlim(0, max(t))
        
        if len(label) == 1:
            latex_label = '$' + label + '$'
        else:
            latex_label = '$' + label[0] + '_' + label[1] + '$'
            
        axes[row][col].set_title(latex_label)
        axes[row][col].grid(True)

    fig.tight_layout()

        
ts_arr = raw_data_containers['ts']
ts_normed = ts_arr - min(ts_arr)

# # Plot raw data
# plot_data(ts_normed, raw_data_containers, raw_data_labels, offset=1, nrows=3)    

# # Plot integrated data
data_labels = ['ts', 'x', 'y', 'z', 'rx', 'ry', 'rz']
data_containers = {}
data_containers['ts'] = ts_arr

for i, label in enumerate(data_labels):
    if i == 0:
        continue
        
    raw_label = raw_data_labels[i]
    v = integrate.cumtrapz(raw_data_containers[raw_label], x=ts_normed)
    data_containers[label] = integrate.cumtrapz(v, x=ts_normed[:-1])
        
plot_data(ts_normed[:-2], data_containers, data_labels, offset=1, nrows=3)

quat_data_labels = ['ts', 'x', 'y', 'z', 'q1', 'q2', 'q3', 'q4']
quat_data_containers = {}
quat_data_containers['ts'] = ts_arr
quat_data_containers['x'] = data_containers['x']
quat_data_containers['y'] = data_containers['y']
quat_data_containers['z'] = data_containers['z']
    
def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) \
        - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) \
        + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) \
        - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) \
        + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

roll = data_containers['rx']        
pitch = data_containers['ry']        
yaw = data_containers['rz']        
[q1, q2, q3, q4] = euler_to_quaternion(roll, pitch, yaw)

quat_data_containers['q1'] = q1
quat_data_containers['q2'] = q2
quat_data_containers['q3'] = q3
quat_data_containers['q4'] = q4

plot_data(ts_normed[:-2], quat_data_containers, quat_data_labels, offset=0, nrows=4)

plt.show()

