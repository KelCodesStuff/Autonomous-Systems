# Autonomous Systems
This project implements the RRT* (Rapidly-exploring Random Tree Star) algorithm for real-time path planning in both 2D and 3D environments with dynamic obstacles. The RRT* algorithm is known for its ability to efficiently explore large search spaces and optimize the path to the goal over time, making it well-suited for autonomous navigation, drones, and robotics applications.

## Features:
Real-Time Path Planning with RRT* and Dynamic Obstacles
- A 2D and 3D environment with static and dynamic obstacles.
- RRT* algorithm for real-time path planning.
- Dynamic obstacle avoidance by continuous replanning.
- Visualization of the environment, obstacles, and paths using Matplotlib.

### 2D Path Planning

The 2D environment includes static rectangular obstacles, and the RRT* algorithm is used to find a collision-free path from the start point to the goal. The algorithm explores random points in the environment and rewires the tree to optimize the path.

- Obstacles: Defined as rectangular regions with specific bottom-left and top-right coordinates.
- Path Planning: RRT* explores the environment, avoiding obstacles and finding the optimal path to the goal.
- Visualization: The environment and the planned path are visualized using Matplotlib, with obstacles drawn as filled rectangles and the path as a line with markers.

### 3D Path Planning

The 3D environment is more complex, featuring dynamic obstacles that move with predefined velocities. The RRT* algorithm continuously replans the path as obstacles move, ensuring that the robot or drone avoids collisions. The dynamic nature of the obstacles makes the environment suitable for real-world scenarios such as drone navigation or autonomous vehicles.

- Obstacles: Defined as cuboids with bottom-left and top-right coordinates. Each obstacle has a velocity vector, causing it to move over time.
- Path Planning: RRT* generates a path and replans at each time step to avoid moving obstacles.
- Visualization: The 3D environment and the dynamic obstacles are visualized using Matplotlib's 3D plotting features. Obstacles are shown as semi-transparent cuboids, and the path is updated in real time.

## Running the Simulations
To run the 2D simulation where RRT* finds a path through a static environment with rectangular obstacles, use the following command:
```bash
python main_2d.py
```

To run the 3D simulation where RRT* continuously replans a path through a dynamic environment with moving cuboid obstacles, use the following command:
```bash
python main_3d.py
