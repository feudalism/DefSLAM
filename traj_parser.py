import numpy as np
import matplotlib.pyplot as plt

def parse(filepath, data_labels):
    data_containers = {}
    for label in data_labels:
        data_containers[label] = []
        
    with open(filepath, 'r') as f:
        for i, line in enumerate(f):
            data = line.split()
            ts = int(float(data[0]))

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
        
        axes[row][col].plot(t, data[label], marker="o", markersize=2., label=file_label)
        # axes[row][col].plot(t, data[label], linestyle="", marker="o", markersize=2.5, label=file_label)
        axes[row][col].set_xlim(left=min_t, right=max_t)
        
        if len(label) == 1:
            latex_label = '$' + label + '$'
        else:
            latex_label = '$' + label[0] + '_' + label[1] + '$'
            
        axes[row][col].set_title(latex_label)
        axes[row][col].grid(True)
    
    return axes