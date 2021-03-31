import numpy as np
from Measurement import VisualMeasurement, ImuMeasurement
import matplotlib.pyplot as plt

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

class Trajectory(object):
    def __init__(self, name):
        self.t = []
        self.x = []
        self.y = []
        self.z = []
        self.qx = []
        self.qy = []
        self.qz = []
        self.qw = []
        self.name = name
        
    def append_data(self, t, data):
        self.t.append(t)
        
        self.x.append(data[0])
        self.y.append(data[1])
        self.z.append(data[2])
        
        self.qx.append(data[3])
        self.qy.append(data[4])
        self.qz.append(data[5])
        self.qw.append(data[6])
        
    def plot(self, axes=None):
        if axes is None:
            fig, axes = plt.subplots(4, 2)
            fig.tight_layout()
            
        labels = ['x', 'y', 'z', 'qw', 'qx', 'qy', 'qz']
        
        for i, label in enumerate(labels):
            ai = i + 1

            if ai <= 3:
                row = ai
                col = 0
            else:
                row = ai - 4
                col = 1
            
            exec(f"{label} = axes[{row}][{col}].plot(self.t, self.{label}, label=self.name)")

            if len(label) == 1:
                latex_label = '$' + label + '$'
            else:
                latex_label = '$' + label[0] + '_' + label[1] + '$'
            axes[row][col].set_title(latex_label)
            axes[row][col].grid(True)
        
        # legend on last plot
        axes[row][col].legend()

        return axes
        
class VisualTraj(object):
    def __init__(self, filepath):
        self.filepath = filepath
        
        labels = ['ts', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
        data = parse(self.filepath, labels)
        
        self.ts = data['ts']
        self.x = data['x']
        self.y = data['y']
        self.z = data['z']
        self.qx = data['qx']
        self.qy = data['qy']
        self.qz = data['qz']
        self.qw = data['qw']
        
    def at_index(self, index):
        t = self.ts[index]
        
        x = self.x[index]
        y = self.y[index]
        z = self.z[index]        
        pos = np.array([x, y, z])
        
        qx = self.qx[index]
        qy = self.qy[index]
        qz = self.qz[index]
        qw = self.qw[index]
        rot = np.array([qx, qy, qz, qw])
        
        return VisualMeasurement(t, pos, rot)

class ImuTraj(object):
    def __init__(self, filepath):
        self.filepath = filepath
        
        labels = ['ts', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
        data = parse(self.filepath, labels)
        
        self.ts = data['ts']
        self.ax = data['ax']
        self.ay = data['ay']
        self.az = data['az']
        self.gx = data['gx']
        self.gy = data['gy']
        self.gz = data['gz']
        
        self.next_frame_index = 0
        self.queue_first_ts = 0
        
    def get_next_frame_index(self, cam_t):
        return max([i for i, t in enumerate(self.ts) if t <= cam_t])
        
    def get_imu_queue(self, cam_t):
        start_index = self.next_frame_index
        next_index = self.get_next_frame_index(cam_t)
        
        if start_index == next_index:
            queue = self.at_index(start_index)
        else:
            t = self.ts[start_index:next_index+1]
        
            ax = self.ax[start_index:next_index+1]
            ay = self.ay[start_index:next_index+1]
            az = self.az[start_index:next_index+1]
            acc = np.vstack((ax, ay, az))
            
            gx = self.gx[start_index:next_index+1]
            gy = self.gy[start_index:next_index+1]
            gz = self.gz[start_index:next_index+1]
            om = np.vstack((gx, gy, gz))
            
            queue = ImuMeasurement(t, acc, om)
            
        self.next_frame_index = next_index + 1
        return queue
        
    def at_index(self, index):
        t = self.ts[index]
        
        ax = self.ax[index]
        ay = self.ay[index]
        az = self.az[index]        
        acc = np.array([ax, ay, az])
        
        gx = self.gx[index]
        gy = self.gy[index]
        gz = self.gz[index]
        om = np.array([gx, gy, gz])
        
        return ImuMeasurement(t, acc, om)