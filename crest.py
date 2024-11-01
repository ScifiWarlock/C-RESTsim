import random
import math
import matplotlib.pyplot as plt
import numpy as np
import time


# Global variables
start_coordinates = [0, 0]
end_coordinates = [100, 100]  # Example destination
current_coordinates = start_coordinates
obstacle_coordinates = []
obc1 = []
obc2 = []
obc3 = []
obc4 = []  # Will be populated with obstacles
avoidance_radius = 6
generate_radius = 2  # Distance the drone will move in each segment
final_radius = 3  # If the drone is within this radius of the destination, move to it
path = [start_coordinates]  # Path will be a list of nodes to the destination


# Toggle between preset or random obstacles
use_preset_obstacles = False  # Set to True to use preset obstacles, False for random ones


# Preset obstacle locations
preset_obstacles = [
   (30, 30),
   (50, 50),
   (75, 75)
]


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


   # Move by the step size (generate_radius) in the direction of the goal
   next_position = [current[0] + unit_vector[0] * step_size, current[1] + unit_vector[1] * step_size]
   return next_position


# Updated cost function
def cost_function(node, end, obstacles):
   if check_collision(node, obstacles, avoidance_radius):
       # Count the number of obstacles
       object_num = len(obstacles)
       nearest_obstacle = min(obstacles, key=lambda obs: dist(node, obs))
       return dist(node, end) * (object_num + 1) / dist(node, nearest_obstacle)
   else:
       return dist(node, end)

def obstacle_randmov(obstacle_list):
   new_list = []
   for element in obstacle_list:
      x = element[0] + random.uniform(-10, 10)
      y = element[1] + random.uniform(-10, 10)
    
   new_list.append((x, y))
   return new_list

# Track computation time
start_time = time.time()


# Input user coordinates (latitude, longitude) for start and end points
start_coordinates = [float(input("Enter start X coordinate: ")), float(input("Enter start Y coordinate: "))]
end_coordinates = [float(input("Enter end X coordinate: ")), float(input("Enter end Y coordinate: "))]


# Generate obstacles based on user choice
if use_preset_obstacles:
   obstacle_coordinates = preset_obstacles  # Use the preset obstacle locations
   print("Using preset obstacles at locations:", preset_obstacles)
else:
   generate_random_obstacles(num_obstacles=10, x_range=(0, 100), y_range=(0, 100))
   


fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 8))
 # Set title, labels, and grid
ax1.set_title('DR-RRT* Pathfinding with Moving Obstacles')

    # Plot start and end points
ax1.scatter(start_coordinates[0], start_coordinates[1], color='green', s=100, label='Start')
ax1.scatter(end_coordinates[0], end_coordinates[1], color='red', s=100, label='End')

np_path = np.array(path)

# DR-RRT* Process Simulation
while dist(current_coordinates, end_coordinates) > final_radius:  # Run until we are within final_radius of the destination
   # Try to move straight towards the goal
   obstacle_coordinates = obstacle_randmov(obstacle_coordinates)
   next_position = move_towards_goal(current_coordinates, end_coordinates, generate_radius)


   # Calculate the cost at the next position
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
       np_path = np.array(path)
       print(f"RRT* Path Chosen. New location: {current_coordinates}")
   else:
       # If no collision is detected, move straight towards the goal
       current_coordinates = next_position
       path.append(current_coordinates)
       np_path = np.array(path)
       print(f"Straight-line move. New location: {current_coordinates}")

   ax1.clear()
   print("1")
       #fig, ax1 = plt.subplots(figsize=(8, 8))
   ax1.set_title('DR-RRT* Pathfinding with Collision Avoidance')
   ax1.scatter(start_coordinates[0], start_coordinates[1], color='green', s=100, label='Start')
   ax1.scatter(end_coordinates[0], end_coordinates[1], color='red', s=100, label='End')
   for obstacle in obstacle_coordinates:
    obstacle_circle = plt.Circle(obstacle, avoidance_radius, color='r', alpha=0.3)
    ax1.add_patch(obstacle_circle)
   ax1.plot(np_path[:, 0], np_path[:, 1], 'b-', linewidth=2, label='Path')
   ax1.scatter(np_path[:, 0], np_path[:, 1], color='blue', s=30, label='Nodes')
   ax1.set_xlabel('X Coordinates')
   ax1.set_ylabel('Y Coordinates')
   ax1.legend(loc='best')
   ax1.grid(True)
   ax1.set_xlim(-10, 110)
   ax1.set_ylim(-10, 110)
    #print("2")
    #plt.show()
    #print("3")
   plt.pause(0.1)
    #print("4")
    #plt.close(ax1)
    #print("5")


# Once within the final radius, move directly to the destination
path.append(end_coordinates)
current_coordinates = end_coordinates
print(f"Drone has reached the destination: {end_coordinates}")


# Calculate computation time
computation_time = time.time() - start_time
print(f"Computation Time: {computation_time:.2f} seconds")


# Calculate the total distance traveled
total_distance = 0
cumulative_distances = [0]  # Store cumulative distances for graphing
for i in range(1, len(path)):
  segment_distance = dist(path[i - 1], path[i])
  total_distance += segment_distance
  cumulative_distances.append(total_distance)


print(f"Total Distance Traveled: {total_distance:.2f} units")


# Plotting the path, cumulative distance, and computation time



# Plot the path
# while dist(current_coordinates, end_coordinates) > final_radius:
    #    ax1.clear()
    #    print("1")
    #    #fig, ax1 = plt.subplots(figsize=(8, 8))
    #    ax1.set_title('DR-RRT* Pathfinding with Collision Avoidance')
    #    ax1.scatter(start_coordinates[0], start_coordinates[1], color='green', s=100, label='Start')
    #    ax1.scatter(end_coordinates[0], end_coordinates[1], color='red', s=100, label='End')
    #    for obstacle in obstacle_coordinates:
    #        obstacle_circle = plt.Circle(obstacle, avoidance_radius, color='r', alpha=0.3)
    #    ax1.add_patch(obstacle_circle)
    #    ax1.plot(path[:, 0], path[:, 1], 'b-', linewidth=2, label='Path')
    #    ax1.scatter(path[:, 0], path[:, 1], color='blue', s=30, label='Nodes')
    #    ax1.set_xlabel('X Coordinates')
    #    ax1.set_ylabel('Y Coordinates')
    #    ax1.legend(loc='best')
    #    ax1.grid(True)
    #    ax1.set_xlim(-10, 110)
    #    ax1.set_ylim(-10, 110)
    #    print("2")
    #    #plt.show()
    #    print("3")
    #    plt.pause(0.1)
    #    print("4")
    #    #plt.close(ax1)
    #    print("5")
