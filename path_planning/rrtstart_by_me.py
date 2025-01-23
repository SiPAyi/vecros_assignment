import matplotlib.pyplot as plt
import numpy as np
import random

# for visualizing
def visualize_terrain(matrix, current_path, path_color=4, show_time = 0.01):
    # color the shortest path pixels
    for pose in current_path:
        matrix[pose[0]][pose[1]] = path_color
        
    # Define the color mapping: 0 -> white, 1 -> black, 2 -> blue
    color_map = {
        0: [255, 255, 255],  # White
        1: [0, 0, 0],  # Black
        2: [255, 0, 0], # setpoints
        3: [255, 0, 255], # random points
        4: [0, 0, 255],   # Blue - explored points
        5: [0, 255, 0] # Green - optimal path
    }
           
    # Convert the matrix into a color map array
    image_data = np.array([[color_map[val] for val in row] for row in matrix])
    
    # Plot the terrain as an image
    plt.clf()  # Clear the previous plot
    plt.imshow(image_data, interpolation='nearest')
    plt.axis('off')  # Remove axes for better visualization
    plt.pause(show_time)  # Pause for 1 second to display the plot

####  creating a plane
length = 40
width = 40
plane = []
# creating obstacles
for i in range(length):
    line = []
    for j in range(width):
        obstacle_or_not = 1 if random.randint(0, 100) < 1 else 0
        line.append(obstacle_or_not)
    plane.append(line)
# creating walls
for i in range(len(plane)):
    plane[i][0] = 1
    plane[i][-1] = 1 
for j in range(len(plane[0])):
    plane[0][j] = 1
    plane[-1][j] = 1

# visualize_terrain(plane, [[0,0]], 3, 4)
    

start = (1, 1)  # Starting position
end = (length-2, width-2)    # Ending position
plane[end[0]][end[1]] = 2
plane[start[0]][start[1]] = 2


junctions_of_created_tree = [{"location": start, "parent_location": start, "parent_cost": 0}] #initialize with the starting point

# add some dummy junctions
junctions_of_created_tree.append({"location": (10,5), "parent_location": start, "parent_cost": 9.848857801796104})
junctions_of_created_tree.append({"location": (10,10), "parent_location": (10,5), "parent_cost": 5})
junctions_of_created_tree.append({"location": (10,12), "parent_location": (10,10), "parent_cost": 2})

def random_point_generator(latest_map):
    while True:
        x, y = (random.randint(0, len(latest_map)-1), random.randint(0, len(latest_map[0])-1))
        if latest_map[x][y] == 0:
            # print("random_point", (x, y))
            break
        # else:
        #     print((x, y), "failed")
        
    return (x,y)


### step1: select a random point
random_point = random_point_generator(plane)
print("random point location: ", random_point[0], random_point[1], "\n")

### step2: select near candidate and eligible candidates(junction from junctions_of_created_tree)
max_travel_cost = 5     # to find get the junction in the radius of the random point
min_travel_cost = float('inf')  # to find the nearest junction
junctions_inside_the_radius = [] #to get all the junctions which are withing the radius
nearest_junction = {}

for junction in junctions_of_created_tree:
    change_in_x = junction["location"][0] - random_point[0]
    change_in_y = junction["location"][1] - random_point[1]
    
    # if moving diagonally and moving to sides is different
    travel_cost = pow(pow((change_in_x),2)+pow((change_in_y),2), 0.5)
    
    # if moving diagonally and moving to sides is not different
    travel_cost = max(change_in_x, change_in_y)
    
    total_cost = travel_cost + junction["parent_cost"]
    
    # print("junction location \t travel cost \t ")
    print(junction["location"],"\t", total_cost)
    
    if travel_cost <= max_travel_cost:
        junctions_inside_the_radius.append(junction)
    
    # find the lowest cost to reach random point via junction
    if total_cost < min_travel_cost:
        min_travel_cost = total_cost
        nearest_junction = junction

# for debugging
all_junctions = [junction["location"] for junction in junctions_of_created_tree]
near_junctions = [junction["location"] for junction in junctions_inside_the_radius]
print(nearest_junction["location"], "is near to ", random_point, "with total cost ", min_travel_cost)
print("locations inside the radius are ", near_junctions)

if len(near_junctions):
    #for visualization of all junctions
    plane[random_point[0]][random_point[1]] = 3 
    visualize_terrain(plane, all_junctions, 4)

    #for visualization of junctions near to random point
    visualize_terrain(plane, near_junctions, 5, 3)
else:
    plane[random_point[0]][random_point[1]] = 3 
    visualize_terrain(plane, [], 4, 3)


# go in the direction of random point until got obstacle or max_travel_cost

