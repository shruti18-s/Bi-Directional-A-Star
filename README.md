# Bidirectional A* Pathfinding Visualization with OpenCV

This project implements a bidirectional A* pathfinding algorithm with visualization in Python using OpenCV. The robot navigates a 2D grid with obstacles to find a path from a start position to a goal position, with a meeting point of the paths searched from both ends.

## Features

- **Bidirectional A\***: Efficient pathfinding algorithm searching from both start and goal.
- **Obstacle Generation**: Dynamic creation of circular and border obstacles.
- **Visualization**: Real-time visualization of the search process and final path.

## Requirements

- **Python 3.x**
- **OpenCV**
- **NumPy**

## Installation

1. **Clone the repository:**
    ```bash
    git clone https://https://github.com/shruti18-s/Bi-Directional-A-Star.git
    cd bidirectional-a-star
    ```

2. **Install the required packages:**
    ```bash
    pip install opencv-python numpy
    ```

## Usage

1. **Run the bidirectional A\* script:**
    ```bash
    python bidirectional_astar.py
    ```

2. The algorithm will visualize the pathfinding process and display the canvas with the start, goal, obstacles, and the found path.

## How It Works

- **Obstacle Creation**: Obstacles are defined as circular and border regions on a 300x600 grid.
- **Bidirectional Search**: The A\* algorithm is executed simultaneously from both the start and goal positions.
- **Meeting Point**: The algorithm searches until the paths from both directions meet, indicating the solution.
- **Path Reconstruction**: The path from start to goal is reconstructed using the meeting point and displayed on the canvas.

## License

This project is licensed under the MIT License.
