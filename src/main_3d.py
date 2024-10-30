# main_3d.py

from environment.environment_3d import Environment3D  # Import the Environment3D class
from obstacles.dynamic_obstacle import DynamicObstacle, Environment3DDynamic  # Import dynamic obstacles
from planning.rrt_star_3d import RRTStar3D  # Import the RRT* algorithm

import numpy as np
import matplotlib.pyplot as plt

# Define the 3D environment with dynamic obstacles
obstacles_3d = [
    DynamicObstacle((2, 2, 1), (4, 4, 3), (0.1, 0.0, 0.1)),  # Obstacle with velocity
    DynamicObstacle((5, 5, 2), (7, 7, 5), (-0.1, 0.0, -0.1))  # Another moving obstacle
]

# Initialize the dynamic environment with obstacles
env_dynamic = Environment3DDynamic(10, 10, 10, obstacles_3d)

# Set start and goal positions in the 3D environment
start_3d = (0, 0, 0)
goal_3d = (9, 9, 9)

# Initialize the RRT* algorithm in 3D
rrt_star_3d = RRTStar3D(start_3d, goal_3d, env_dynamic, max_iter=200, step_size=1.5)

# Create a non-blocking figure
plt.ion()  # Turn on interactive mode for dynamic plotting
fig = plt.figure()
ax_3d = fig.add_subplot(111, projection='3d')

# Simulate for 10 steps, moving obstacles and replanning at each step
for step in range(10):
    print(f"Step {step + 1}: Updating obstacles and replanning...")

    # Clear the previous plot
    ax_3d.cla()

    # Move the dynamic obstacles
    env_dynamic.update_obstacles()

    # Replan the path using RRT*
    path_3d = rrt_star_3d.plan()

    if path_3d:
        # Visualize the environment, path, and obstacles
        ax_3d = env_dynamic.plot()
        # Plot the path found by RRT*
        path_3d = np.array(path_3d)
        ax_3d.plot(path_3d[:, 0], path_3d[:, 1], path_3d[:, 2], '-o', label="RRT* Path in 3D")
        plt.legend()
    else:
        # If no path is found, display a warning and continue moving obstacles
        print(f"Step {step + 1}: No valid path found.")
        ax_3d = env_dynamic.plot()

    # Enhance visualization with axis labels and grid
    ax_3d.set_xlabel('X Axis')
    ax_3d.set_ylabel('Y Axis')
    ax_3d.set_zlabel('Z Axis')
    ax_3d.grid(True)

    # Update the figure dynamically
    plt.draw()
    plt.pause(1)  # Pause for 0.5 seconds before the next iteration

# Keep the plot open after the simulation finishes
plt.ioff()  # Turn off interactive mode
plt.show()

