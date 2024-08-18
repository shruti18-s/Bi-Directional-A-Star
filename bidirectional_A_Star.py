import cv2  # OpenCV for image processing and visualization
import numpy as np  # NumPy for numerical operations
import heapq as hq  # Heap queue algorithm (priority queue) for efficient pathfinding
import math  # Math module for mathematical operations
import time  # Time module to manage delays in visualization
import matplotlib.pyplot as plt  # Matplotlib for plotting, although not used in this code

# Create a blank canvas (white background) of size 300x600 pixels with 3 color channels (RGB)
canvas = np.ones((300, 600, 3)) * 255

# Create a set and a list to manage obstacle coordinates
obstacle_set = set()  # Set for quick lookups
obstacle_list = []  # List for visualization

# Create grids (2D arrays) to track the cost-to-come (c2c) and total cost (tc) for each grid point
# Initialize all values with infinity (indicating initially all points are unreachable)
c2c_node_grid = [[float('inf')] * 300 for _ in range(600)]
tc_node_grid = [[float('inf')] * 300 for _ in range(600)]

# Sets to keep track of the explored nodes (points) from both the start and goal
closed_set = set()  # For the forward search from the start
closed_set_goal = set()  # For the backward search from the goal

# Dictionaries to store parent nodes, which help in reconstructing the final path
parents = {}
parents_goal = {}

# Robot and motion parameters
R = 66 / 20  # Radius parameter (arbitrary)
L = 28.7  # Length parameter (arbitrary)
T = 0.5  # Time step for motion
t_max = 3.5  # Maximum time for motion
t_min = 0.1  # Minimum time for motion
x_start, y_start, theta_start = 0, 150, 0  # Start position and orientation (0 degrees)
x_goal, y_goal, theta_goal = 590, 150, 0  # Goal position and orientation (0 degrees)
rpm1, rpm2 = 10, 20  # Rotations per minute for the robot's wheels
min_rpm = min(rpm1, rpm2)  # Minimum RPM between the two wheels
t = round(((t_max - t_min) * (min_rpm - 75) / (5 - 75)) + t_min, 2)  # Calculate a time factor based on RPM

# Create obstacles on the canvas (these are circular and border-like obstacles)
for y in range(300):
    for x in range(600):
        # Create top and bottom borders
        if 0 <= y <= T or 300 - T <= y <= 300:
            obstacle_set.add((x, y))
            obstacle_list.append((x, y))
        # Create circular obstacles at specific positions
        if (x-112)**2 + (y-242.5)**2 <= 40**2 or \
           (x-263)**2 + (y-90)**2 <= 70**2 or \
           (x-445)**2 + (y-220)**2 <= 37.5**2:
            obstacle_set.add((x, y))
            obstacle_list.append((x, y))

# Color the obstacle points red on the canvas
for point in obstacle_list:
    canvas[point[1], point[0]] = [255, 0, 0]

# Initial setup for bidirectional A* search (one from start, one from goal)
open_list = []  # Open list for start-to-goal search
open_list_goal = []  # Open list for goal-to-start search
initial_node_start = (0, 0, 1, [], (x_start, y_start), theta_start)  # Initial node for the start
initial_node_goal = (0, 0, 1, [], (x_goal, y_goal), theta_goal)  # Initial node for the goal

# Add initial nodes to their respective open lists (priority queues)
hq.heappush(open_list, initial_node_start)
hq.heappush(open_list_goal, initial_node_goal)

# Set the initial parents for start and goal nodes to None (no parent)
parents[(x_start, y_start)] = None
parents_goal[(x_goal, y_goal)] = None

# Define possible actions the robot can take (combinations of wheel speeds)
action_lists = [(0, rpm1), (rpm1, 0), (rpm1, rpm1), (rpm1, rpm2), (rpm2, rpm1), (0, rpm2), (rpm2, 0), (rpm2, rpm2)]

# Function to calculate the next position and costs based on a given action
def action(node, rpm1, rpm2, goal_x, goal_y):
    ul = 2 * math.pi * rpm1 / 60  # Calculate angular velocity for wheel 1
    ur = 2 * math.pi * rpm2 / 60  # Calculate angular velocity for wheel 2
    new_heading = (node[5] + np.rad2deg((R / L) * (ul - ur) * t)) % 360  # Calculate new heading (orientation)
    x_vel = (R / 2) * (ur + ul) * np.cos(np.deg2rad(new_heading))  # Calculate x-velocity
    y_vel = (R / 2) * (ur + ul) * np.sin(np.deg2rad(new_heading))  # Calculate y-velocity
    x = round(node[4][0] + x_vel * t)  # Update x-position
    y = round(node[4][1] + y_vel * t)  # Update y-position
    c2c = node[1] + math.sqrt((x_vel * t) ** 2 + (y_vel * t) ** 2)  # Calculate cost-to-come
    c2g = math.sqrt((goal_y - y) ** 2 + (goal_x - x) ** 2)  # Calculate cost-to-go (heuristic)
    tc = c2c + c2g  # Total cost
    return (x, y), new_heading, tc, c2c  # Return the new position, heading, and costs

# Function to check if a point is within valid bounds and not in an obstacle
def valid_point(point):
    x, y = point
    return 0 <= x < 600 and 0 <= y < 300 and point not in obstacle_set

# Function to expand a node (generate its neighboring nodes based on possible actions)
def expand_node(node, open_list, closed_set, goal_x, goal_y, parents_dict, tc_node_grid, c2c_node_grid):
    for action_set in action_lists:  # Iterate through each possible action
        point, new_heading, tc, c2c = action(node, action_set[0], action_set[1], goal_x, goal_y)  # Get the new node
        if valid_point(point) and point not in closed_set:  # Check if the new node is valid and not already explored
            x, y = point
            if tc < tc_node_grid[x][y]:  # If the new node has a lower cost, update the cost grids
                tc_node_grid[x][y] = tc
                c2c_node_grid[x][y] = c2c
                new_node = (tc, c2c, len(parents_dict) + 1, node[3] + [node[2]], point, new_heading)  # Create a new node tuple
                hq.heappush(open_list, new_node)  # Add the new node to the open list
                parents_dict[point] = node[4]  # Update the parent dictionary

# Main loop to search for a meeting point between the start and goal paths
meeting_point = None
while open_list and open_list_goal:
    # Expand the nodes from the start side
    if open_list:
        node = hq.heappop(open_list)  # Get the node with the lowest cost
        closed_set.add(node[4])  # Mark this node as explored
        if node[4] in closed_set_goal:  # Check if this node is also in the goal's explored set
            meeting_point = node[4]  # If yes, we've found a meeting point
            break  # Exit the loop
        expand_node(node, open_list, closed_set, x_goal, y_goal, parents, tc_node_grid, c2c_node_grid)  # Expand the node

    # Expand the nodes from the goal side (only if no meeting point has been found)
    if open_list_goal and not meeting_point:
        node_goal = hq.heappop(open_list_goal)  # Get the node with the lowest cost
        closed_set_goal.add(node_goal[4])  # Mark this node as explored
        if node_goal[4] in closed_set:  # Check if this node is also in the start's explored set
            meeting_point = node_goal[4]  # If yes, we've found a meeting point
            break  # Exit the loop
        expand_node(node_goal, open_list_goal, closed_set_goal, x_start, y_start, parents_goal, tc_node_grid, c2c_node_grid)  # Expand the node

# Function to reconstruct the path from start to goal using the meeting point
def reconstruct_path(meeting_point, parents, parents_goal):
    path = []
    step = meeting_point
    while step:  # Trace the path back from the meeting point to the start
        path.append(step)
        step = parents.get(step)
    path.reverse()  # Reverse to get the path from start to meeting point

    step = parents_goal.get(meeting_point)  # Trace the path from the meeting point to the goal
    while step:
        path.append(step)
        step = parents_goal.get(step)

    return path

# If a meeting point was found, reconstruct the path and visualize it
if meeting_point:
    path = reconstruct_path(meeting_point, parents, parents_goal)
    print("Path found")
    for point in path:  # Draw the path on the canvas in green
        canvas[max(0, point[1]-1):min(canvas.shape[0], point[1]+2), max(0, point[0]-1):min(canvas.shape[1], point[0]+2)] = [0, 255, 0]

# Visualize the start and goal positions on the canvas
cv2.circle(canvas, (x_start, y_start), 5, (0, 0, 255), -1)  # Red circle for the start position
cv2.circle(canvas, (x_goal, y_goal), 5, (255, 0, 0), -1)  # Blue circle for the goal position

# Display the final canvas with obstacles, start, goal, and path
cv2.imshow("Canvas", cv2.flip(canvas, 0))  # Flip the canvas vertically before displaying
cv2.waitKey(0)  # Wait indefinitely until a key is pressed
cv2.destroyAllWindows()  # Close all OpenCV windows
