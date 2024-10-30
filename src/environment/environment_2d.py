# environment_2d.py

import numpy as np
import matplotlib.pyplot as plt

# Define the 2D environment with obstacles
class Environment2D:
    def __init__(self, x_range, y_range, obstacles):
        """
        Initializes the environment.
        :param x_range: Width of the environment (x-axis limit).
        :param y_range: Height of the environment (y-axis limit).
        :param obstacles: List of obstacles, where each obstacle is defined by bottom-left and top-right coordinates.
        """
        self.x_range = x_range  # X range of the environment (size in x direction)
        self.y_range = y_range  # Y range of the environment (size in y direction)
        self.obstacles = obstacles  # List of obstacles, defined by their coordinates

    def is_in_obstacle(self, point):
        """
        Check if a given point is inside any obstacle.
        :param point: A tuple representing the (x, y) coordinates of the point to check.
        :return: True if the point is within any obstacle, False otherwise.
        """
        for (bl, tr) in self.obstacles:  # Loop through all obstacles
            # Check if point is within obstacle's x and y bounds
            if bl[0] <= point[0] <= tr[0] and bl[1] <= point[1] <= tr[1]:
                return True  # Point is inside obstacle
        return False  # No collision

    def plot(self):
        """
        Plot the environment, including obstacles.
        :return: Matplotlib axis object for further plotting.
        """
        fig, ax = plt.subplots()  # Create a new figure
        ax.set_xlim(0, self.x_range)  # Set the x-axis limit
        ax.set_ylim(0, self.y_range)  # Set the y-axis limit

        # Draw each obstacle as a gray rectangle
        for (bl, tr) in self.obstacles:
            ax.add_patch(plt.Rectangle(bl, tr[0] - bl[0], tr[1] - bl[1], fill=True, color="gray"))

        return ax  # Return the axis object to allow additional plotting
