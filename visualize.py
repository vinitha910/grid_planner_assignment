"""
Plots paths outputted from the xytheta example.

Usage:
    $ python visualize.py [cfg file] [solution file]

"""
import sys
import re
import numpy as np
import matplotlib.pyplot as plt

def get_map(filename):
    f = open(filename, 'r')
    lines = f.readlines()
    width = len(lines[0].split())
    height = len(lines)
    data = [p.strip().split() for p in lines]
    flattened_values = map(int, [item for sublist in data for item in
                                 sublist])
    map_values = np.array(flattened_values).reshape(height, width)
    return map_values

def plot_planned_path(filename, ax):
    f = open(filename, 'r')
    lines = f.readlines()
    sol_values = np.array([map(int, p.strip().split()) for p in lines])
    ax.plot(sol_values[:,0], sol_values[:,1], 'y-')

if __name__ == '__main__':
    map_values = get_map(sys.argv[1])
    fig, ax = plt.subplots(figsize=(10,10))
    plt.imshow(map_values, vmin=0, vmax=1)
    plt.ylim([0, map_values.shape[0]])
    plt.xlim([0, map_values.shape[1]])

    plot_planned_path(sys.argv[2], ax)
    plt.show()
