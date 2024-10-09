Enhanced RRT Path Planning Algorithm

An implementation of an Enhanced Rapidly-exploring Random Tree (RRT) algorithm with multiple improvements to increase efficiency and performance in complex environments.
Table of Contents

    Introduction
    Algorithm Enhancements
    Dependencies
    Installation
    Usage
    Examples
    Project Structure
    Contributing
    License
    Acknowledgements

Introduction

Path planning is crucial in robotics and autonomous systems for navigating from a start point to a goal point while avoiding obstacles. The Rapidly-exploring Random Tree (RRT) algorithm is popular due to its efficiency in high-dimensional spaces.

This project implements an enhanced version of the RRT algorithm with several improvements, making it more efficient and reliable in complex environments.
Algorithm Enhancements

The enhanced RRT algorithm includes the following key improvements:

    Efficient Nearest Neighbor Search using KD-Tree
        Utilizes a KD-Tree for efficient nearest neighbor searches, significantly reducing computation time.

    Adaptive Step Size in Steer Function
        Adjusts the step size dynamically based on the distance to the target node, allowing for finer movements near obstacles or the goal.

    Enhanced Collision Checking
        Performs collision checks by sampling multiple points along the path between nodes, improving the accuracy of obstacle detection.

    Rewiring of Nearby Nodes (Similar to RRT*)
        Optimizes the tree by rewiring nodes if a shorter path is found, reducing the overall path cost.

    Subgoal Sampling and Bottleneck Detection
        Detects bottlenecks based on node connectivity and local density.
        Samples subgoals around bottlenecks to guide the exploration more effectively.

    Parallel Processing with ThreadPoolExecutor
        Accelerates the rewiring process by parallelizing computations using Python's ThreadPoolExecutor.

    Path Smoothing using Line-of-Sight
        Removes unnecessary intermediate nodes by checking for collision-free straight lines between nodes, resulting in a smoother path.

Dependencies

The project requires the following Python libraries:

    Python 3.6 or higher
    NumPy
    Matplotlib
    SciPy

Installation

    Clone the Repository

    bash

git clone https://github.com/yourusername/enhanced_rrt.git
cd enhanced_rrt

Create a Virtual Environment (Optional but Recommended)

bash

python3 -m venv venv
source venv/bin/activate  # On Windows, use venv\Scripts\activate

Install Dependencies

bash

pip install -r requirements.txt

If a requirements.txt file is not provided, install dependencies manually:

bash

    pip install numpy matplotlib scipy

Usage

The main script generates multiple random scenarios and visualizes the path planning results.

To run the script:

bash

python enhanced_rrt.py

Parameters:

    map_size: Tuple specifying the width and height of the map (e.g., (200, 200)).
    step_size: Maximum distance between nodes (default is 5).
    max_iter: Maximum number of iterations for the algorithm (default is 2000).
    num_scenarios: Number of random scenarios to simulate (default is 10).

You can modify these parameters directly in the script or adapt the code to accept command-line arguments.
Examples

The script generates num_scenarios random scenarios, each with:

    Randomly generated start and goal positions that are not inside obstacles.
    Randomly generated obstacles with varying sizes, positions, and orientations.
    Visualization of the path found by the enhanced RRT algorithm, or a message indicating if no path was found.

Sample Output:

After running the script, a window displays multiple subplots, each representing a different scenario. Obstacles are shown in gray, the start point in blue, the goal point in red, and the planned path in green.

Note: Replace the placeholder with an actual screenshot of the output.
Project Structure

bash

enhanced_rrt/
├── enhanced_rrt.py         # Main script containing the RRT implementation
├── requirements.txt        # List of Python dependencies
├── README.md               # Project documentation
├── LICENSE                 # License information
└── images/
    └── sample_output.png   # Sample output image

Contributing

Contributions are welcome! If you'd like to contribute:

    Fork the repository.
    Create a new branch for your feature or bug fix.
    Commit your changes with descriptive messages.
    Push your branch and create a pull request.

Please ensure your code adheres to the existing style and includes appropriate documentation.
License

This project is licensed under the MIT License. See the LICENSE file for details.
Acknowledgements

    The RRT algorithm is widely used in robotics and path planning research.
    This implementation was inspired by various resources and academic papers on RRT and its variants.
    Special thanks to the open-source community for providing the tools and libraries that make projects like this possible.
