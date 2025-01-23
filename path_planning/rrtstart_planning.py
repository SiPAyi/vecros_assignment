import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

# Define grid size and initialize the grid with obstacles
GRID_SIZE = 101  # Smaller grid for simplicity
grid = np.zeros((GRID_SIZE, GRID_SIZE, GRID_SIZE))

# Function to assign obstacles
def assign_obstacles(grid, num_obstacles=50):
    # Randomly assign obstacles (value 1)
    obstacle_indices = np.random.randint(0, GRID_SIZE, (num_obstacles, 3))
    for idx in obstacle_indices:
        grid[tuple(idx)] = 1  # Mark obstacle cells as 1

# Function to generate random start and end points
def generate_random_points(grid_size, num_points):
    start_end_sets = []
    for _ in range(num_points):
        start = (random.randint(0, grid_size-1), random.randint(0, grid_size-1), random.randint(0, grid_size-1))
        end = start
        while end == start:
            end = (random.randint(0, grid_size-1), random.randint(0, grid_size-1), random.randint(0, grid_size-1))
        start_end_sets.append((start, end))
    return start_end_sets

# Improved RRT* Algorithm Implementation with better obstacle handling
def rrt_star(start, end, grid, max_iters=1000, step_size=1, radius=2):
    # Randomized Tree initialization
    tree = [start]
    parent = {start: None}
    cost = {start: 0}
    
    def valid_move(x, y, z):
        return 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE and 0 <= z < GRID_SIZE and grid[x, y, z] == 0

    def distance(p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))
    
    def get_nearest_node(p):
        return min(tree, key=lambda node: distance(node, p))

    def new_point(nearest, rand_point):
        direction = np.array(rand_point) - np.array(nearest)
        norm = np.linalg.norm(direction)
        if norm > step_size:
            direction = direction / norm * step_size
        return tuple(np.array(nearest) + direction)

    # Function to check if the path between two nodes is valid (no obstacles)
    def path_valid(node1, node2):
        # Check if the straight line path between node1 and node2 is free of obstacles
        x1, y1, z1 = node1
        x2, y2, z2 = node2
        # We use Bresenham's line algorithm to check the path in 3D (discrete steps)
        steps = max(abs(x2 - x1), abs(y2 - y1), abs(z2 - z1))
        for i in range(steps):
            x = int(x1 + i * (x2 - x1) / steps)
            y = int(y1 + i * (y2 - y1) / steps)
            z = int(z1 + i * (z2 - z1) / steps)
            if not valid_move(x, y, z):
                return False
        return True
    
    def rewire(new_node):
        for node in tree:
            if distance(node, new_node) < radius and cost[node] + distance(node, new_node) < cost.get(new_node, float('inf')) and path_valid(node, new_node):
                parent[new_node] = node
                cost[new_node] = cost[node] + distance(node, new_node)

    for _ in range(max_iters):
        rand_point = (random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1), random.randint(0, GRID_SIZE-1))
        
        nearest_node = get_nearest_node(rand_point)
        new_node = new_point(nearest_node, rand_point)
        
        if valid_move(*new_node) and path_valid(nearest_node, new_node):
            tree.append(new_node)
            parent[new_node] = nearest_node
            cost[new_node] = cost[nearest_node] + distance(nearest_node, new_node)
            rewire(new_node)
        
        if distance(new_node, end) < step_size:  # End is reached
            break

    # Backtrack to get the path
    path = []
    node = new_node
    while node is not None:
        path.append(node)
        node = parent[node]
    
    return path[::-1]

# Function to plot the obstacles and paths
def plot_paths(paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot obstacles
    obstacle_indices = np.argwhere(grid == 1)
    xs, ys, zs = zip(*obstacle_indices)
    ax.scatter(xs, ys, zs, c='r', marker='s', label="Obstacle", s=50)

    # Plot paths
    for path in paths:
        if path:
            xs, ys, zs = zip(*path)
            ax.plot(xs, ys, zs, label="Path")

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('RRT* Path Planning with Obstacles')
    ax.legend()
    plt.show()

# Example usage:
assign_obstacles(grid, num_obstacles=100)  # Assign 100 obstacles to the grid
random_start_end_sets = generate_random_points(GRID_SIZE, 5)
for start, end in random_start_end_sets:
    path = rrt_star(start, end, grid)
    plot_paths([path])  # Plot the path from start to end
