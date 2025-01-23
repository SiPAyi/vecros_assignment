import numpy as np
import heapq
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import random

# Define the 3D grid size
GRID_SIZE = 101  # Smaller grid for simplicity

# Initialize the grid with zeros (0 = free space, 1 = obstacle)
grid = np.zeros((GRID_SIZE, GRID_SIZE, GRID_SIZE))

# Function to assign obstacles
def assign_obstacles(grid, num_obstacles=50):
    # Randomly assign obstacles (value 1)
    obstacle_indices = np.random.randint(0, GRID_SIZE, (num_obstacles, 3))
    for idx in obstacle_indices:
        grid[tuple(idx)] = 1  # Mark obstacle cells as 1

# Assign obstacles to the grid
assign_obstacles(grid)

# A* algorithm with grid and obstacle consideration
def astar(start, end, grid):
    def heuristic(a, b):
        # Using Euclidean distance as heuristic
        return np.linalg.norm(np.array(a) - np.array(b))

    def valid_move(x, y, z):
        # Check if the move is within bounds and not an obstacle or occupied cell
        return 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE and 0 <= z < GRID_SIZE and grid[x, y, z] == 0

    def get_neighbors(x, y, z):
        neighbors = []
        for dx, dy, dz in [(-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0), (0, 0, -1), (0, 0, 1),
                             (-1, -1, 0), (1, 1, 0), (-1, 0, -1), (1, 0, 1), (0, -1, -1), (0, 1, 1),
                             (-1, -1, -1), (1, 1, 1)]:  # Diagonal and straight neighbors
            nx, ny, nz = x + dx, y + dy, z + dz
            if valid_move(nx, ny, nz):
                neighbors.append((nx, ny, nz))
        return neighbors

    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, end), 0, start))
    g_costs = {start: 0}
    came_from = {}

    while open_list:
        _, g, current = heapq.heappop(open_list)

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return path from start to end

        for neighbor in get_neighbors(*current):
            tentative_g = g + 1  # Fixed cost of 1 to move in all directions
            if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                g_costs[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, end)
                heapq.heappush(open_list, (f, tentative_g, neighbor))
                came_from[neighbor] = current

    return None  # No path found

# Function to mark the path in the grid
def mark_path_in_grid(path, grid):
    for cell in path:
        grid[cell] = np.random.randint(2, 10)  # Mark path cells with a value > 1 (indicating occupied)

# Function to plot the paths
def plot_paths(paths):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the obstacles
    obstacle_indices = np.argwhere(grid == 1)
    xs, ys, zs = zip(*obstacle_indices)
    ax.scatter(xs, ys, zs, c='r', marker='o', label="Obstacle", s=50)

    for path in paths:
        if path:
            xs, ys, zs = zip(*path)
            ax.plot(xs, ys, zs)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

# Function to handle multiple sets of start and end points
def find_and_plot_paths(start_end_sets):
    paths = []
    for start, end in start_end_sets:
        path = astar(start, end, grid)
        if path:
            paths.append(path)
            mark_path_in_grid(path, grid)  # Mark the path as occupied in the grid

    plot_paths(paths)


# Function to generate random start and end points
def generate_random_points(grid_size, num_points):
    start_end_sets = []
    for _ in range(num_points):
        # Random start point
        start = (random.randint(0, grid_size-1), random.randint(0, grid_size-1), random.randint(0, grid_size-1))
        
        # Random end point (make sure it's not the same as start)
        end = start
        while end == start:
            end = (random.randint(0, grid_size-1), random.randint(0, grid_size-1), random.randint(0, grid_size-1))
        
        start_end_sets.append((start, end))
    return start_end_sets

# Example: Generate 5 random start-end pairs within a grid of size 10x10x10
random_start_end_sets = generate_random_points(GRID_SIZE, 5)

# Now you can pass these start_end_sets to the find_and_plot_paths function
find_and_plot_paths(random_start_end_sets)
