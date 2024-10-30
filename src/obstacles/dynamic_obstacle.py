# dynamic_obstacle.py

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from src.environment.environment_3d import Environment3D  # Import the base Environment3D class


class DynamicObstacle:
    def __init__(self, bl, tr, velocity):
        # Convert bottom-left and top-right coordinates and velocity to float arrays to avoid casting errors
        self.bl = np.array(bl, dtype=float)
        self.tr = np.array(tr, dtype=float)
        self.velocity = np.array(velocity, dtype=float)

    def move(self):
        """Update obstacle position by its velocity."""
        self.bl += self.velocity
        self.tr += self.velocity


class Environment3DDynamic(Environment3D):  # Inherit from Environment3D
    def __init__(self, x_range, y_range, z_range, dynamic_obstacles):
        super().__init__(x_range, y_range, z_range, [])  # Call the parent class constructor
        self.dynamic_obstacles = dynamic_obstacles

    def update_obstacles(self):
        """Update the positions of all dynamic obstacles."""
        for obstacle in self.dynamic_obstacles:
            obstacle.move()
            # Ensure the obstacles stay within the bounds
            obstacle.bl = np.clip(obstacle.bl, [0, 0, 0], [self.x_range, self.y_range, self.z_range])
            obstacle.tr = np.clip(obstacle.tr, [0, 0, 0], [self.x_range, self.y_range, self.z_range])

    def is_in_obstacle(self, point):
        """Check if a given point is inside any dynamic obstacle."""
        for obstacle in self.dynamic_obstacles:
            if obstacle.bl[0] <= point[0] <= obstacle.tr[0] and obstacle.bl[1] <= point[1] <= obstacle.tr[1] and obstacle.bl[2] <= point[2] <= obstacle.tr[2]:
                return True
        return False

    def plot(self):
        """Plot the 3D environment with dynamic obstacles."""
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(0, self.x_range)
        ax.set_ylim(0, self.y_range)
        ax.set_zlim(0, self.z_range)

        # Plot dynamic obstacles
        for obstacle in self.dynamic_obstacles:
            # Create vertices for the cuboid obstacle
            bl = obstacle.bl
            tr = obstacle.tr
            vertices = [[bl, [tr[0], bl[1], bl[2]], [tr[0], tr[1], bl[2]], [bl[0], tr[1], bl[2]]],
                        [[bl[0], bl[1], tr[2]], [tr[0], bl[1], tr[2]], tr, [bl[0], tr[1], tr[2]]],
                        [bl, [bl[0], bl[1], tr[2]], [bl[0], tr[1], tr[2]], [bl[0], tr[1], bl[2]]],
                        [[tr[0], bl[1], bl[2]], tr, [tr[0], tr[1], bl[2]], [tr[0], bl[1], tr[2]]],
                        [[bl[0], tr[1], bl[2]], [tr[0], tr[1], bl[2]], tr, [bl[0], tr[1], tr[2]]],
                        [[bl[0], bl[1], bl[2]], [tr[0], bl[1], bl[2]], [tr[0], bl[1], tr[2]], [bl[0], bl[1], tr[2]]]]

            # Plot the cuboid obstacle
            ax.add_collection3d(Poly3DCollection(vertices, facecolors='gray', linewidths=1, edgecolors='r', alpha=.25))

        return ax

