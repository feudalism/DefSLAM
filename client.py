from DataSocket import TCPReceiveSocket

import threading
import sys

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim

PORT = 8080
ADDR = "127.0.0.1"
STOP_FLAG = threading.Event()

data_headers = ['t', 'x', 'y', 'z', 'q1', 'q2', 'q3', 'q4']

def get_datapoint(data):
    posdata = str(data, 'utf-8').rstrip().split(' ')
    data_point = {}
    
    for i, label in enumerate(data_headers):
        data_point[label] = float(posdata[i])
        
    print(data_point)
    
    return data_point

# Main receive socket function
receive_socket = TCPReceiveSocket(tcp_port=PORT,
        tcp_ip=ADDR,
        receive_as_raw=True,
        handler_function=get_datapoint)

def start_receive_socket():
    print("Starting receive socket...")
    receive_socket.start()

thread = threading.Thread(target=start_receive_socket)
thread.start()

# Stop receiving
input("Press enter to stop receiving data.\n")
STOP_FLAG.set()
thread.join()

print("Stopping receive socket.")
receive_socket.stop()
sys.exit(0)