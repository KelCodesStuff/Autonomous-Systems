# environment_3d.py

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Environment3D:
    def __init__(self, x_range, y_range, z_range, obstacles):
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.obstacles = obstacles  # List of obstacles (cuboids defined by their bottom-left and top-right coordinates)

    def is_in_obstacle(self, point):
        """Check if a given point is inside any of the obstacles."""
        for (bl, tr) in self.obstacles:
            if bl[0] <= point[0] <= tr[0] and bl[1] <= point[1] <= tr[1] and bl[2] <= point[2] <= tr[2]:
                return True
        return False

    def plot(self):
        """Plot the 3D environment with obstacles."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(0, self.x_range)
        ax.set_ylim(0, self.y_range)
        ax.set_zlim(0, self.z_range)

        for (bl, tr) in self.obstacles:
            x = [bl[0], tr[0]]
            y = [bl[1], tr[1]]
            z = [bl[2], tr[2]]
            ax.bar3d(bl[0], bl[1], bl[2], tr[0] - bl[0], tr[1] - bl[1], tr[2] - bl[2], color='gray', alpha=0.6)

        return ax
