# main_2d.py

import numpy as np
import matplotlib.pyplot as plt

from src.environment.environment_2d import Environment2D
from src.planning.rrt_star_2d import RRTStar

# Define the environment with obstacles (rectangles defined by bottom-left and top-right corners)
obstacles = [((2, 2), (4, 4)), ((5, 5), (7, 7))]
env = Environment2D(10, 10, obstacles)  # Create environment with a 10x10 grid and given obstacles

# Set start and goal positions for the robot/drone
start = (0, 0)  # Starting point at (0, 0)
goal = (9, 9)  # Goal point at (9, 9)

# Create an instance of the RRTStar class
rrt_star = RRTStar(start, goal, env)

# Run the RRT* algorithm to find a path from start to goal
path = rrt_star.plan()

# Visualize the environment and the resulting path, if one was found
ax = env.plot()  # Plot the environment with obstacles

# If a path was found, plot it
if path:
    path = np.array(path)  # Convert path to numpy array for easy plotting
    ax.plot(path[:, 0], path[:, 1], '-o', label="RRT* Path")  # Plot the path on the environment
    plt.legend()  # Show the legend
else:
    print("No path found.")  # Print a message if no path is found

plt.show()  # Show the plot

