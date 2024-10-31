import random
import math
import matplotlib.pyplot as plt
import numpy as np
import time


# Global variables
start_coordinates = [0, 0]
end_coordinates = [100, 100]  # Example destination
current_coordinates = start_coordinates
obstacle_coordinates = []  # Will be populated with obstacles
obstacle_station_coords = []
avoidance_radius = 6
generate_radius = 2  # Distance the drone will move in each segment
final_radius = 6  # If the drone is within this radius of the destination, move to it
path = [start_coordinates]  # Path will be a list of nodes to the destination


# Toggle between preset or random obstacles
use_preset_obstacles = False  # Set to True to use preset obstacles, False for random ones


# Preset obstacle locations
preset_obstacles = [(30, 30), (50, 50), (75, 75), (30, 50), (50, 30), (30, 75), (75, 30), (50, 75), (75, 50)]


# Helper function to calculate distance between two points
def dist(p1, p2):
   return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# Randomly generate obstacles within a certain range
def generate_random_obstacles(num_obstacles, x_range, y_range):
   for _ in range(num_obstacles):
       obstacle = [
           random.uniform(x_range[0], x_range[1]),
           random.uniform(y_range[0], y_range[1])
       ]
       obstacle_coordinates.append(tuple(obstacle))
   print(f"{num_obstacles} random obstacles generated.")

def generate_station_obstacles(num_obstacles, x_range, y_range):
   for _ in range(num_obstacles):
       obstacle = [
           random.uniform(x_range[0], x_range[1]),
           random.uniform(y_range[0], y_range[1])
       ]
       obstacle_station_coords.append(tuple(obstacle))
   print(f"{num_obstacles} random obstacles generated.")


# Check if a node is within the avoidance radius of an obstacle
def check_collision(node, obstacles, avoidance_radius):
   for obs in obstacles:
       if dist(node, obs) < avoidance_radius:
           print(f"Collide! Node {node} is within {avoidance_radius} units of obstacle {obs}")
           return True
   return False


# Generate random sample nodes within a radius, ensuring nodes are not too close
def generate_random_nodes(current_coordinates, generate_radius, num_samples):
   sample_nodes = []
   for _ in range(num_samples):
       random_node = [
           random.uniform(current_coordinates[0] - generate_radius, current_coordinates[0] + generate_radius),
           random.uniform(current_coordinates[1] - generate_radius, current_coordinates[1] + generate_radius)
       ]
       sample_nodes.append(random_node)
   return sample_nodes


# Function to move straight towards the goal
def move_towards_goal(current, goal, step_size):
   direction = [(goal[0] - current[0]), (goal[1] - current[1])]
   norm = math.sqrt(direction[0] ** 2 + direction[1] ** 2)
   unit_vector = [direction[0] / norm, direction[1] / norm]


   next_position = [current[0] + unit_vector[0] * step_size, current[1] + unit_vector[1] * step_size]
   return next_position


# Updated cost function
def cost_function(node, end, obstacles):
   if check_collision(node, obstacles, avoidance_radius):
       object_num = len(obstacles)
       nearest_obstacle = min(obstacles, key=lambda obs: dist(node, obs))
       return dist(node, end) * (object_num + 1) / dist(node, nearest_obstacle)
   else:
       return dist(node, end)


# Track computation time
start_time = time.time()


#(latitude, longitude) for start and end points
start_coordinates = [float(input("Enter start X coordinate: ")), float(input("Enter start Y coordinate: "))]
end_coordinates = [float(input("Enter end X coordinate: ")), float(input("Enter end Y coordinate: "))]


if use_preset_obstacles:
   obstacle_coordinates = preset_obstacles  # Use the preset obstacle locations
   print("Using preset obstacles at locations:", preset_obstacles)
else:
   generate_random_obstacles(num_obstacles=20, x_range=(0, 100), y_range=(0, 100))
   generate_station_obstacles(num_obstacles=20, x_range=(0, 100), y_range=(0, 100))


# C-REST Process Simulation
while dist(current_coordinates, end_coordinates) > final_radius: 
   next_position = move_towards_goal(current_coordinates, end_coordinates, generate_radius)


   move_cost = cost_function(next_position, end_coordinates, obstacle_coordinates)
   print(f"Cost at node {next_position}: {move_cost}")

   if check_collision(next_position, obstacle_coordinates, avoidance_radius):
       print("Avoidance needed! Using RRT* to find a valid path.")


       sampled_nodes = generate_random_nodes(current_coordinates, generate_radius, 25)


       valid_nodes = []
       for node in sampled_nodes:
           if not check_collision(node, obstacle_coordinates, avoidance_radius):
               valid_nodes.append(node)


       if not valid_nodes:
           print("No valid nodes found, re-sampling...")
           continue


       # Choose lowest cost node
       chosen_node = min(valid_nodes, key=lambda node: cost_function(node, end_coordinates, obstacle_coordinates))


      
       current_coordinates = chosen_node
       path.append(chosen_node)
       print(f"RRT* Path Chosen. New location: {current_coordinates}")
   else:
       current_coordinates = next_position
       path.append(current_coordinates)
       print(f"Straight-line move. New location: {current_coordinates}")


path.append(end_coordinates)
current_coordinates = end_coordinates
print(f"Drone has reached the destination: {end_coordinates}")


# Calculate computation time
computation_time = time.time() - start_time
print(f"Computation Time: {computation_time:.2f} seconds")


total_distance = 0
cumulative_distances = [0] 
for i in range(1, len(path)):
  segment_distance = dist(path[i - 1], path[i])
  total_distance += segment_distance
  cumulative_distances.append(total_distance)


print(f"Total Distance Traveled: {total_distance:.2f} units")


# Function to randomly move obstacles
def move_obstacles(obstacle_coordinates, move_radius=5, x_bounds=(0, 100), y_bounds=(0, 100)):
    new_obstacle_coords = []
    for obs in obstacle_coordinates:
        new_x = obs[0] + random.uniform(-move_radius, move_radius)
        new_y = obs[1] + random.uniform(-move_radius, move_radius)

        # Keep obstacles within bounds
        new_x = max(min(new_x, x_bounds[1]), x_bounds[0])
        new_y = max(min(new_y, y_bounds[1]), y_bounds[0])

        new_obstacle_coords.append((new_x, new_y))

    return new_obstacle_coords


# Plot the path step-by-step with moving obstacles
path = [start_coordinates]
current_coordinates = start_coordinates
fig, ax1 = plt.subplots(figsize=(8, 8))
 # Set title, labels, and grid
ax1.set_title('DR-RRT* Pathfinding with Moving Obstacles')

    # Plot start and end points
ax1.scatter(start_coordinates[0], start_coordinates[1], color='green', s=100, label='Start')
ax1.scatter(end_coordinates[0], end_coordinates[1], color='red', s=100, label='End')
time.sleep(10)
green_blue = 0

while dist(current_coordinates, end_coordinates) > final_radius:
    ax1.clear()  
    obstacle_coordinates = move_obstacles(obstacle_coordinates)

    next_position = move_towards_goal(current_coordinates, end_coordinates, generate_radius)

    move_cost = cost_function(next_position, end_coordinates, obstacle_coordinates)
    print(f"Cost at node {next_position}: {move_cost}")

    # Check if the next position is in collision with any obstacle
    if check_collision(next_position, obstacle_coordinates, avoidance_radius):
        print("Avoidance needed! Using RRT* to find a valid path.")

        # Use DR-RRT* to avoid the obstacle
        sampled_nodes = generate_random_nodes(current_coordinates, generate_radius, 25)

        # Check for valid nodes not colliding with obstacles
        valid_nodes = []
        for node in sampled_nodes:
            if not check_collision(node, obstacle_coordinates, avoidance_radius):
                valid_nodes.append(node)

        # If no valid nodes are found, continue sampling
        if not valid_nodes:
            print("No valid nodes found, re-sampling...")
            continue

        # Choose the node with the minimum cost
        chosen_node = min(valid_nodes, key=lambda node: cost_function(node, end_coordinates, obstacle_coordinates))

        # Update current position and path
        current_coordinates = chosen_node
        path.append(chosen_node)
        print(f"RRT* Path Chosen. New location: {current_coordinates}")
    else:
        current_coordinates = next_position
        path.append(current_coordinates)
        print(f"Straight-line move. New location: {current_coordinates}")

    path_array = np.array(path)

    # Set title, labels, and grid
    ax1.set_title('DR-RRT* Pathfinding with Moving Obstacles')

    # Plot start and end points
    ax1.scatter(start_coordinates[0], start_coordinates[1], color='green', s=100, label='Start')
    ax1.scatter(end_coordinates[0], end_coordinates[1], color='red', s=100, label='End')

    # Plot moving obstacles with avoidance radius
    for obstacle in obstacle_coordinates:
        obstacle_circle = plt.Circle(obstacle, avoidance_radius, color='r', alpha=0.3)
        ax1.add_patch(obstacle_circle)
        obstacle_circle_drone = plt.Circle(obstacle, 2, color='b', alpha=0.3)
        ax1.add_patch(obstacle_circle_drone)

    # Plot the path up to the current point
    ax1.plot(path_array[:, 0], path_array[:, 1], 'b-', linewidth=2, label='Path')
    if green_blue%2 == 0:
        ax1.scatter(path_array[:, 0], path_array[:, 1], color='green', s=30, label='Nodes')
        green_blue += 1
    elif green_blue%2 == 1:
        ax1.scatter(path_array[:, 0], path_array[:, 1], color='blue', s=30, label='Nodes')
        green_blue -= 1
    

    # Set plot limits and labels
    ax1.set_xlabel('X Coordinates')
    ax1.set_ylabel('Y Coordinates')
    ax1.legend(loc='best')
    ax1.grid(True)
    ax1.set_xlim(-10, 110)
    ax1.set_ylim(-10, 110)

   
    plt.pause(0.1)  

# Once within the final radius, move directly to the destination
path.append(end_coordinates)
#current_coordinates = end_coordinates
print(f"Drone has reached the destination: {end_coordinates}")

# Close the figure after the animation is done
#time.sleep(10)
ax1.scatter(path_array[:, 0], path_array[:, 1], color='green', s=30, label='Nodes')
ax1.plot(path_array[:, 0], path_array[:, 1], 'b-', linewidth=2, label='Path')
plt.pause(10)
plt.close(fig)

# Plotting the path, cumulative distance, and computation time
# fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 8))

# # Plot the cumulative distance graph
# ax2.set_title('Cumulative Distance Traveled')
# ax2.plot(cumulative_distances, 'g-', linewidth=2, label='Cumulative Distance')
# ax2.set_xlabel('Steps')
# ax2.set_ylabel('Cumulative Distance (units)')
# ax2.legend(loc='best')
# ax2.grid(True)


# # Annotate the cumulative distance
# ax2.text(len(cumulative_distances) - 1, cumulative_distances[-1],
#        f'{total_distance:.2f} units', fontsize=12, color='black',
#        verticalalignment='bottom', horizontalalignment='right')


# # Plot the computation time as a bar graph
# ax3.set_title('Computation Time')
# ax3.bar(['Computation Time'], [computation_time], color='orange')
# ax3.set_ylabel('Time (seconds)')
# ax3.grid(True)


# # Annotate the computation time
# ax3.text(0, computation_time, f'{computation_time:.5f} sec', fontsize=12,
#        verticalalignment='bottom', horizontalalignment='center')


# # Show all plots
# plt.tight_layout()
# plt.show()