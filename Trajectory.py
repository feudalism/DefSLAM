import numpy as np
from Measurement import VisualMeasurement, ImuMeasurement

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
    def __init__(self):
        

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
        
    def get_first_index(self, cam_index):
        first_index = 0
        while self.ts[first_index] <= cam_index:
            first_index += 1
        return first_index - 1
        
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