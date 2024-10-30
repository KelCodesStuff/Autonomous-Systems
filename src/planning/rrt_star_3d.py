# rrt_star_3d.py

import random
import math

class Node3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = 0.0

class RRTStar3D:
    def __init__(self, start, goal, env, max_iter=200, step_size=1.5):
        self.start = Node3D(*start)
        self.goal = Node3D(*goal)
        self.env = env
        self.max_iter = max_iter  # Max iterations for planning
        self.step_size = step_size  # Step size for extending the tree
        self.radius = 3.0  # Radius for rewiring the tree
        self.nodes = [self.start]

    def distance(self, node1, node2):
        """Calculate Euclidean distance between two nodes in 3D."""
        return math.sqrt((node2.x - node1.x)**2 + (node2.y - node1.y)**2 + (node2.z - node1.z)**2)

    def nearest_node(self, point):
        """Find the nearest node in the tree to a given point."""
        return min(self.nodes, key=lambda node: self.distance(node, Node3D(*point)))

    def steer(self, from_node, to_point):
        """Steer from one node towards a point in 3D."""
        theta_xy = math.atan2(to_point[1] - from_node.y, to_point[0] - from_node.x)
        theta_z = math.atan2(to_point[2] - from_node.z, self.distance(Node3D(from_node.x, from_node.y, 0), Node3D(*to_point[:2], 0)))
        new_x = from_node.x + self.step_size * math.cos(theta_xy)
        new_y = from_node.y + self.step_size * math.sin(theta_xy)
        new_z = from_node.z + self.step_size * math.sin(theta_z)
        return Node3D(new_x, new_y, new_z)

    def is_collision_free(self, node):
        """Check if the path to the node is collision-free."""
        if not (0 <= node.x <= self.env.x_range and 0 <= node.y <= self.env.y_range and 0 <= node.z <= self.env.z_range):
            return False
        if self.env.is_in_obstacle((node.x, node.y, node.z)):
            return False
        return True

    def biased_sample(self):
        """Sample a random point with a bias towards the goal."""
        if random.random() < self.goal_bias:
            # With probability goal_bias, sample the goal
            return self.goal.x, self.goal.y, self.goal.z
        else:
            # Otherwise, sample randomly from the environment
            return random.uniform(0, self.env.x_range), random.uniform(0, self.env.y_range), random.uniform(0, self.env.z_range)

    def plan(self):
        for i in range(self.max_iter):
            # Generate a random point in the environment
            rand_point = (random.uniform(0, self.env.x_range), random.uniform(0, self.env.y_range), random.uniform(0, self.env.z_range))

            # Find nearest node in the tree
            nearest_node = self.nearest_node(rand_point)
            new_node = self.steer(nearest_node, rand_point)

            # Check for collisions
            if self.is_collision_free(new_node):
                # Add new node to tree
                new_node.parent = nearest_node
                self.nodes.append(new_node)

                # Rewire the tree within a certain radius
                self.rewire(new_node)

            # Check if goal is reached
            if self.distance(new_node, self.goal) <= self.step_size:
                self.goal.parent = new_node
                self.nodes.append(self.goal)
                return self.extract_path()

        # Return None if no path is found after max_iter
        print("Warning: No valid path found after max iterations.")
        return None

    def rewire(self, new_node):
        """Rewire the tree to ensure optimal paths."""
        for node in self.nodes:
            if node == new_node:
                continue
            if self.distance(node, new_node) <= self.radius:
                potential_cost = new_node.cost + self.distance(node, new_node)
                if potential_cost < node.cost:
                    node.parent = new_node
                    node.cost = potential_cost

    def extract_path(self):
        """Extract the final path from start to goal."""
        path = []
        node = self.goal
        while node:
            path.append((node.x, node.y, node.z))
            node = node.parent
        return path[::-1]  # Reverse the path to start from the beginning
