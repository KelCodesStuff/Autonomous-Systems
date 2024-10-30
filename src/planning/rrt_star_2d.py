# rrt_star_2d.py

import random
import math


# Class to define each node in the RRT* tree
class Node2D:
    def __init__(self, x, y):
        """
        Initializes a new node in the RRT* tree.
        :param x: X-coordinate of the node.
        :param y: Y-coordinate of the node.
        """
        self.x = x  # X-coordinate
        self.y = y  # Y-coordinate
        self.parent = None  # Pointer to the parent node in the tree (used to trace the path)
        self.cost = 0.0  # Cost to reach this node from the start (used for path optimization)


# Class to define and implement the RRT* algorithm
class RRTStar:
    def __init__(self, start, goal, env, max_iter=500):
        """
        Initializes the RRT* algorithm.
        :param start: Start point (x, y) coordinates.
        :param goal: Goal point (x, y) coordinates.
        :param env: The environment object which includes obstacles and dimensions.
        :param max_iter: Maximum number of iterations to run the algorithm.
        """
        self.start = Node2D(*start)  # Starting point node
        self.goal = Node2D(*goal)  # Goal point node
        self.env = env  # Environment with obstacles
        self.max_iter = max_iter  # Max iterations for the algorithm
        self.step_size = 1.0  # Step size for extending the tree
        self.radius = 3.0  # Radius for rewiring nodes (to ensure optimality)
        self.nodes = [self.start]  # List of nodes in the tree, initialized with the start node

    def distance(self, node1, node2):
        """
        Calculate the Euclidean distance between two nodes.
        :param node1: First node.
        :param node2: Second node.
        :return: Euclidean distance between node1 and node2.
        """
        return math.hypot(node2.x - node1.x, node2.y - node1.y)

    def nearest_node(self, point):
        """
        Find the nearest node in the tree to a given point.
        :param point: A tuple representing the (x, y) coordinates of the target point.
        :return: Nearest node in the current tree to the point.
        """
        return min(self.nodes, key=lambda node: self.distance(node, Node2D(*point)))  # Find node with minimum distance

    def steer(self, from_node, to_point):
        """
        Steer from one node towards a point, creating a new node in that direction.
        :param from_node: The node to start from.
        :param to_point: The target point (x, y).
        :return: A new node created by stepping from 'from_node' towards 'to_point'.
        """
        theta = math.atan2(to_point[1] - from_node.y, to_point[0] - from_node.x)  # Angle towards the target point
        new_x = from_node.x + self.step_size * math.cos(theta)  # New x position after stepping
        new_y = from_node.y + self.step_size * math.sin(theta)  # New y position after stepping
        return Node2D(new_x, new_y)  # Return the new node

    def is_collision_free(self, node):
        """
        Check if a node is within the environment bounds and is collision-free.
        :param node: The node to check.
        :return: True if the node is in free space, False if it collides with an obstacle.
        """
        # Check if node is out of bounds
        if not (0 <= node.x <= self.env.x_range and 0 <= node.y <= self.env.y_range):
            return False

        # Check if node is inside any obstacle
        if self.env.is_in_obstacle((node.x, node.y)):
            return False

        return True  # No collisions, return True

    def plan(self):
        """
        Run the RRT* algorithm to plan a path from start to goal.
        :return: The path as a list of (x, y) tuples, or None if no path is found.
        """
        for i in range(self.max_iter):
            # Step 1: Generate a random point in the environment
            rand_point = (random.uniform(0, self.env.x_range), random.uniform(0, self.env.y_range))

            # Step 2: Find the nearest node in the tree
            nearest_node = self.nearest_node(rand_point)
            new_node = self.steer(nearest_node, rand_point)  # Create a new node in that direction

            # Step 3: Check for collisions
            if self.is_collision_free(new_node):
                # Step 4: If collision-free, add new node to the tree
                new_node.parent = nearest_node  # Set parent of the new node
                new_node.cost = nearest_node.cost + self.distance(nearest_node, new_node)  # Update cost
                self.nodes.append(new_node)  # Add to node list

                # Step 5: Rewire the tree to improve path optimality
                self.rewire(new_node)

            # Step 6: Check if the goal is reached
            if self.distance(new_node, self.goal) <= self.step_size:
                self.goal.parent = new_node  # Set goal's parent
                self.goal.cost = new_node.cost + self.distance(new_node, self.goal)  # Update goal's cost
                self.nodes.append(self.goal)  # Add goal to node list
                return self.extract_path()  # Extract the optimal path

        return None  # Return None if no path is found within max iterations

    def rewire(self, new_node):
        """
        Rewire the tree to ensure that all nodes within a certain radius have the optimal parent.
        :param new_node: The newly added node.
        """
        for node in self.nodes:
            if node == new_node:
                continue  # Skip the new node itself
            if self.distance(node, new_node) <= self.radius:  # Check if node is within rewiring radius
                potential_cost = new_node.cost + self.distance(node, new_node)  # Calculate the new potential cost
                if potential_cost < node.cost:  # Rewire if the new path is cheaper
                    node.parent = new_node  # Set new parent
                    node.cost = potential_cost  # Update node cost

    def extract_path(self):
        """
        Extract the final path by backtracking from the goal to the start.
        :return: A list of (x, y) coordinates representing the path.
        """
        path = []
        node = self.goal  # Start from the goal node
        while node:
            path.append((node.x, node.y))  # Add the node's coordinates to the path
            node = node.parent  # Move to the parent node
        return path[::-1]  # Return the reversed path (from start to goal)
