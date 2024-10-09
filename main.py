import numpy as np
import matplotlib.pyplot as plt
import random
from scipy.spatial import KDTree
from concurrent.futures import ThreadPoolExecutor

# Node Class with Children Tracking
class Node:
    """
    Represents a node in the RRT tree.

    Attributes:
        x (float): The x-coordinate of the node.
        y (float): The y-coordinate of the node.
        parent (Node): The parent node in the tree.
        cost (float): The cost to reach this node from the start node.
        children (list of Node): The list of child nodes connected to this node.
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0
        self.children = []  # Track child nodes

# Enhanced RRT Class with Corrected Smoothing
class RRTSubgoalEnhanced:
    """
    An enhanced Rapidly-exploring Random Tree (RRT) algorithm implementation with several improvements,
    including bottleneck detection, subgoal sampling, and path smoothing.

    Attributes:
        start (Node): The start node.
        goal (Node): The goal node.
        obstacles (list of matplotlib.patches): The list of obstacles in the map.
        map_size (tuple): The size of the map (width, height).
        step_size (float): The maximum distance between nodes.
        max_iter (int): The maximum number of iterations to run the algorithm.
        goal_tolerance (float): The distance threshold to consider the goal as reached.
        nodes (list of Node): The list of all nodes in the tree.
        subgoals (list of Node): The list of subgoal nodes used for biased sampling.
        kd_tree (KDTree): The KDTree for efficient nearest neighbor search.
        bottleneck_density_threshold (int): The threshold for node density to detect bottlenecks.
        new_nodes_batch (list of Node): The batch of new nodes for batch processing in rewiring.
    """
    def __init__(self, start, goal, obstacles, map_size, step_size=5, max_iter=2000, goal_tolerance=5):
        """
        Initializes the RRTSubgoalEnhanced instance.

        Args:
            start (tuple): The (x, y) coordinates of the start position.
            goal (tuple): The (x, y) coordinates of the goal position.
            obstacles (list of matplotlib.patches): The list of obstacles in the map.
            map_size (tuple): The size of the map (width, height).
            step_size (float, optional): The maximum distance between nodes. Default is 5.
            max_iter (int, optional): The maximum number of iterations to run the algorithm. Default is 2000.
            goal_tolerance (float, optional): The distance threshold to consider the goal as reached. Default is 5.
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = obstacles
        self.map_size = map_size
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_tolerance = goal_tolerance
        self.nodes = [self.start]
        self.subgoals = []
        self.kd_tree = KDTree([(self.start.x, self.start.y)])
        self.bottleneck_density_threshold = 10  # Threshold for node density bottleneck detection
        self.new_nodes_batch = []

    def distance(self, node1, node2):
        """
        Calculates the Euclidean distance between two nodes.

        Args:
            node1 (Node): The first node.
            node2 (Node): The second node.

        Returns:
            float: The Euclidean distance between node1 and node2.
        """
        return np.hypot(node1.x - node2.x, node1.y - node2.y)

    def get_random_node(self):
        """
        Generates a random node within the map boundaries.
        Implements biased sampling towards subgoals to enhance exploration in bottleneck areas.

        Returns:
            Node: A randomly generated node.
        """
        if self.subgoals and random.random() < 0.5:  # 50% chance to sample around a bottleneck/subgoal
            subgoal = random.choice(self.subgoals)
            x = random.gauss(subgoal.x, self.map_size[0] * 0.05)
            y = random.gauss(subgoal.y, self.map_size[1] * 0.05)
        else:
            x = random.uniform(0, self.map_size[0])
            y = random.uniform(0, self.map_size[1])
        return Node(x, y)

    def nearest_node(self, random_node):
        """
        Finds the nearest node in the tree to the given random node using a KDTree.

        Args:
            random_node (Node): The random node to find the nearest neighbor for.

        Returns:
            Node: The nearest node in the tree to the random_node.
        """
        distance, index = self.kd_tree.query([random_node.x, random_node.y])
        return self.nodes[index]

    def is_collision(self, node1, node2):
        """
        Checks for collision between two nodes by sampling points along the line connecting them.

        Args:
            node1 (Node): The starting node.
            node2 (Node): The ending node.

        Returns:
            bool: True if a collision is detected, False otherwise.
        """
        distance = self.distance(node1, node2)
        if distance == 0:
            return False  # No need to check collision for zero distance
        steps = max(1, int(distance / (self.step_size * 0.5)))  # Ensure at least one step
        for i in range(steps + 1):
            t = i / steps
            x = node1.x + t * (node2.x - node1.x)
            y = node1.y + t * (node2.y - node1.y)
            for obstacle in self.obstacles:
                if obstacle.contains_point((x, y)):
                    return True
        return False

    def steer(self, from_node, to_node):
        """
        Generates a new node in the direction from from_node to to_node, constrained by the step size.

        Args:
            from_node (Node): The node to start from.
            to_node (Node): The target node to move towards.

        Returns:
            Node: The new node created in the direction towards to_node.
        """
        distance = self.distance(from_node, to_node)
        step = min(self.step_size, distance)  # Adaptive step size
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + step * np.cos(theta)
        new_y = from_node.y + step * np.sin(theta)
        new_node = Node(new_x, new_y)
        new_node.parent = from_node
        from_node.children.append(new_node)  # Maintain parent-child relationship
        new_node.cost = from_node.cost + step
        return new_node

    def update_kd_tree(self):
        """
        Updates the KDTree after adding new nodes to ensure efficient nearest neighbor searches.
        """
        points = [(node.x, node.y) for node in self.nodes]
        self.kd_tree = KDTree(points)

    def add_subgoals(self):
        """
        Adds subgoals by generating random nodes within the map.
        Subgoals are used to bias the sampling towards certain areas, like bottlenecks.
        """
        self.subgoals = [
            self.get_random_node() for _ in range(min(10, len(self.nodes)))
        ]

    def compute_local_density(self, node, radius=10):
        """
        Computes the number of nodes within a given radius of the specified node.

        Args:
            node (Node): The node around which to compute the density.
            radius (float, optional): The radius within which to count neighboring nodes. Default is 10.

        Returns:
            int: The number of nodes within the specified radius.
        """
        neighbors = self.kd_tree.query_ball_point([node.x, node.y], r=radius)
        return len(neighbors)

    def detect_bottleneck(self, node):
        """
        Detects if a node is a bottleneck based on its connectivity and local node density.

        Args:
            node (Node): The node to check for being a bottleneck.

        Returns:
            bool: True if the node is considered a bottleneck, False otherwise.
        """
        # Bottleneck based on node connectivity
        if len(node.children) == 1 and node.parent and len(node.parent.children) > 1:
            return True
        # Bottleneck based on local node density
        density = self.compute_local_density(node)
        if density > self.bottleneck_density_threshold:
            return True
        return False

    def rewire_node(self, new_node, radius):
        """
        Rewires nearby nodes to the new node if it results in a lower cost path.

        Args:
            new_node (Node): The node to potentially rewire other nodes to.
            radius (float): The radius within which to consider nodes for rewiring.
        """
        neighbors = self.kd_tree.query_ball_point([new_node.x, new_node.y], r=radius)
        for index in neighbors:
            node = self.nodes[index]
            if node is not new_node.parent and node is not new_node:
                potential_cost = new_node.cost + self.distance(new_node, node)
                if potential_cost < node.cost and not self.is_collision(new_node, node):
                    # Update parent and cost
                    if node.parent:
                        node.parent.children.remove(node)  # Remove from old parent's children
                    node.parent = new_node
                    new_node.children.append(node)  # Add to new parent's children
                    node.cost = potential_cost

    def rewiring(self, new_node):
        """
        Performs batch rewiring of nodes to improve the tree's connectivity and path cost.

        Args:
            new_node (Node): The latest node added to the tree.
        """
        self.new_nodes_batch.append(new_node)
        if len(self.new_nodes_batch) >= 50:
            radius = 15.0 / np.sqrt(len(self.nodes)) if len(self.nodes) > 0 else 15.0  # Adaptive rewiring radius
            with ThreadPoolExecutor() as executor:
                futures = []
                for node in self.new_nodes_batch:
                    futures.append(executor.submit(self.rewire_node, node, radius))
                for future in futures:
                    future.result()
            self.new_nodes_batch.clear()

    def planning(self):
        """
        Executes the RRT algorithm to find a path from the start node to the goal node.

        Returns:
            list of tuple: The smoothed path from start to goal if found, None otherwise.
        """
        for i in range(self.max_iter):
            if i % 50 == 0:
                self.add_subgoals()
                # Adjust bottleneck density threshold based on average density
                densities = [self.compute_local_density(node) for node in self.nodes]
                if densities:
                    avg_density = sum(densities) / len(densities)
                    self.bottleneck_density_threshold = max(10, int(avg_density * 1.5))

            random_node = self.get_random_node()
            nearest_node = self.nearest_node(random_node)
            new_node = self.steer(nearest_node, random_node)

            if not self.is_collision(nearest_node, new_node):
                if all(self.distance(new_node, n) > 1.0 for n in self.nodes):
                    self.nodes.append(new_node)
                    self.update_kd_tree()
                self.rewiring(new_node)

                # Check for bottlenecks
                if self.detect_bottleneck(nearest_node):
                    print(f"Bottleneck detected at ({nearest_node.x}, {nearest_node.y}).")
                    if nearest_node not in self.subgoals:
                        self.subgoals.append(nearest_node)

                # Check if goal is reached
                if self.distance(new_node, self.goal) <= self.goal_tolerance and not self.is_collision(new_node, self.goal):
                    path = self.generate_path(new_node)
                    return self.smooth_path(path)

        return None

    def smooth_path(self, path):
        """
        Smooths the path by removing unnecessary intermediate nodes using line-of-sight checks.

        Args:
            path (list of tuple): The original path from start to goal.

        Returns:
            list of tuple: The smoothed path.
        """
        if not path:
            return path

        # Initial line-of-sight smoothing
        smoothed_path = [path[0]]
        current_index = 0
        while current_index < len(path) - 1:
            next_index = len(path) - 1
            while next_index > current_index + 1:
                if not self.is_collision(Node(*path[current_index]), Node(*path[next_index])):
                    break
                next_index -= 1
            smoothed_path.append(path[next_index])
            current_index = next_index

        # Return the smoothed path without applying splines
        return smoothed_path

    def generate_path(self, goal_node):
        """
        Generates the path from the start node to the given goal node.

        Args:
            goal_node (Node): The goal node reached by the algorithm.

        Returns:
            list of tuple: The path from start to goal as a list of (x, y) coordinates.
        """
        path = [(goal_node.x, goal_node.y)]
        node = goal_node
        while node.parent is not None:
            node = node.parent
            path.append((node.x, node.y))
        return path[::-1]  # Return path from start to goal

# Simulation parameters
map_size = (200, 200)  # Increased map size
step_size = 5
max_iter = 2000  # Increased max iterations

# Function to generate random complex obstacles
def generate_random_obstacles(num_obstacles, map_size):
    """
    Generates a list of random rectangular obstacles with random positions, sizes, and orientations.

    Args:
        num_obstacles (int): The number of obstacles to generate.
        map_size (tuple): The size of the map (width, height).

    Returns:
        list of matplotlib.patches.Rectangle: The list of generated obstacles.
    """
    obstacles = []
    for _ in range(num_obstacles):
        ox = random.uniform(0, map_size[0] * 0.8)
        oy = random.uniform(0, map_size[1] * 0.8)
        width = random.uniform(map_size[0] * 0.05, map_size[0] * 0.2)
        height = random.uniform(map_size[1] * 0.05, map_size[1] * 0.2)
        angle = random.uniform(0, 360)
        obstacle = plt.Rectangle((ox, oy), width, height, angle=angle, color='gray')
        obstacles.append(obstacle)
    return obstacles

# Function to check if a point is inside any of the obstacles
def is_point_in_obstacle(point, obstacles):
    """
    Checks whether a given point is inside any of the provided obstacles.

    Args:
        point (tuple): The (x, y) coordinates of the point to check.
        obstacles (list of matplotlib.patches.Rectangle): The list of obstacles.

    Returns:
        bool: True if the point is inside an obstacle, False otherwise.
    """
    for obstacle in obstacles:
        if obstacle.contains_point(point):
            return True
    return False

# Update the RRTSubgoalEnhanced class's is_collision method to use the new function
RRTSubgoalEnhanced.is_point_in_obstacle = staticmethod(is_point_in_obstacle)

# Plotting 10 random scenarios
num_scenarios = 10
fig, axs = plt.subplots(2, 5, figsize=(25, 10))
axs = axs.flatten()

for i in range(num_scenarios):
    # Generate random obstacles
    num_obstacles = random.randint(5, 15)
    obstacles = generate_random_obstacles(num_obstacles, map_size)

    # Generate start and goal positions avoiding obstacles
    while True:
        start = (random.uniform(0, map_size[0]), random.uniform(0, map_size[1]))
        if not is_point_in_obstacle(start, obstacles):
            break

    while True:
        goal = (random.uniform(0, map_size[0]), random.uniform(0, map_size[1]))
        if not is_point_in_obstacle(goal, obstacles):
            break

    enhanced_rrt = RRTSubgoalEnhanced(start, goal, obstacles, map_size, step_size, max_iter)
    path = enhanced_rrt.planning()

    # Plotting the scenario
    ax = axs[i]
    ax.set_xlim(0, map_size[0])
    ax.set_ylim(0, map_size[1])
    ax.set_title(f"Scenario {i+1}")

    # Plot obstacles
    for obstacle in obstacles:
        ax.add_patch(obstacle)

    # Plot nodes and path
    if path:
        x, y = zip(*path)
        ax.plot(x, y, color='g', linewidth=2, label="Path")
        ax.scatter([start[0]], [start[1]], color='b', s=50, label="Start")
        ax.scatter([goal[0]], [goal[1]], color='r', s=50, label="Goal")
    else:
        ax.text(0.5, 0.5, 'No Path Found', horizontalalignment='center',
                verticalalignment='center', transform=ax.transAxes, color='red')

    ax.legend()
    ax.grid(True)

plt.tight_layout()
plt.show()
