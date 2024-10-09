# RRT Subgoal Enhanced Algorithm

## Overview

This project implements an enhanced version of the Rapidly-exploring Random Tree (RRT) algorithm, called RRT Subgoal Enhanced. The algorithm includes several improvements, such as bottleneck detection, subgoal sampling, path smoothing, and batch rewiring to enhance the efficiency and quality of path planning in environments with obstacles.

## Features

- Subgoal Sampling: Biased sampling towards subgoals in bottleneck areas to improve the exploration of complex regions.
- Path Smoothing: Removes unnecessary intermediate nodes to produce smoother paths.
- KDTree Nearest Neighbor Search: Efficiently finds the nearest node in the RRT tree using a KDTree.
- Batch Rewiring: Rewires nearby nodes to reduce path cost while maintaining tree connectivity.
- Bottleneck Detection: Identifies and resolves bottlenecks where node connectivity is limited or local density is high.
- Multi-threading: Utilizes concurrent processing for efficient rewiring of nodes.

## Requirements

- Python 3.x
- numpy
- matplotlib
- scipy

You can install the necessary libraries using:

pip install numpy matplotlib scipy

markdown


## How to Run

1. Clone the repository:

git clone https://github.com/your_username/RRT-Subgoal-Enhanced.git

css

2. Navigate to the project directory:

cd RRT-Subgoal-Enhanced

css

3. Run the code:

python main.py

markdown


The script will generate 10 random scenarios and visualize the planned path for each scenario, if a path is found.

## Code Explanation

### Node Class

The `Node` class represents a single node in the RRT tree. It tracks the node's coordinates, parent, children, and the cost to reach the node from the start node.

### RRTSubgoalEnhanced Class

The `RRTSubgoalEnhanced` class implements the main algorithm with the following key methods:
- `distance(node1, node2)`: Computes the Euclidean distance between two nodes.
- `get_random_node()`: Generates a random node within the map, biased towards subgoals.
- `nearest_node(random_node)`: Finds the nearest node in the tree using a KDTree.
- `is_collision(node1, node2)`: Checks for collisions between two nodes.
- `steer(from_node, to_node)`: Generates a new node in the direction of a target node, constrained by the step size.
- `rewiring(new_node)`: Rewires nearby nodes to improve connectivity and reduce path cost.
- `planning()`: Executes the RRT algorithm to find a path from the start node to the goal.
- `smooth_path(path)`: Smooths the generated path by removing unnecessary intermediate nodes.
- `generate_path(goal_node)`: Generates the path from the start node to the goal node.

### Simulation Parameters

- Map Size: Set to `(200, 200)` by default.
- Step Size: Set to `5`.
- Max Iterations: Increased to `2000` to ensure comprehensive exploration in complex maps.
- Obstacles: Random rectangular obstacles are generated and placed within the map to simulate real-world constraints.

## Visualization

The code generates 10 random scenarios, each visualized with:
- Obstacles (gray rectangles)
- Start Node (blue point)
- Goal Node (red point)
- Planned Path (green line)

If no valid path is found, the scenario will display a "No Path Found" message.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.

## Author

- [Your Name](https://github.com/your_username)

## Contributions

Feel free to fork this project, create pull requests, or submit issues if you find any bugs or want to contribute new features.

---

Thank you for checking out this project! Happy path planning!
