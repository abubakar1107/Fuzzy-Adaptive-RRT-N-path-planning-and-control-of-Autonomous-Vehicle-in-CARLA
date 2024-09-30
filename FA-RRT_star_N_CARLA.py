#!/usr/bin/env python3

import numpy as np
import cv2
import matplotlib.pyplot as plt
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import time  
import carla
import pygame
import random
import time
import numpy as np
import math



def connect_to_carla():
    for _ in range(10):  # Attempt to connect 10 times
        try:
            client = carla.Client('localhost', 2000)
            client.set_timeout(10.0)
            world = client.get_world()  # This should not throw an error if the server is ready
            print("Successfully connected to CARLA server!")
            return client, world
        except RuntimeError as e:
            print(f"Connection attempt failed: {e}")
            time.sleep(1)
    raise RuntimeError("Failed to connect to CARLA server after several attempts")


map_width, map_height = 6000, 2000  # Map dimensions in pixels

def create_map(width, height, clearance):
    """The below is an example map, be sure to change it according to the town selected in CARLA."""
    obstacle_map = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.rectangle(obstacle_map, (1500 - clearance, 0 - clearance), (1750 + clearance, 1000 + clearance), (255,0,0), -1)
    cv2.rectangle(obstacle_map, (2500 - clearance, 1000 - clearance), (2750 + clearance, 2000 + clearance), (0,255,0), -1)
    cv2.circle(obstacle_map, (4200, 800), 600 + clearance, (0, 0, 255), -1)
    return obstacle_map

def is_collision_free(x, y, obstacle_map):
    """Checks if the given position (x, y) is free from collisions."""
    return 0 <= x < obstacle_map.shape[1] and 0 <= y < obstacle_map.shape[0] and np.all(obstacle_map[int(y), int(x)] == [255, 255, 255])

def sample_free(obstacle_map, start, goal, goal_sampling_rate):
    """Bias sampling towards the goal with a specified probability."""
    if np.random.rand() < goal_sampling_rate:
        return goal
    else:
        while True:
            point = np.random.rand(2) * np.array([map_width, map_height])
            if is_collision_free(point[0], point[1], obstacle_map):
                return point

def distance(point1, point2):
    return np.linalg.norm(point1 - point2)

def nearest_vertex(point, vertices):
    """Finds the nearest vertex in the tree to the given point."""
    return min(vertices, key=lambda v: np.linalg.norm(point - v[:2]))

def steer(from_node, to_node, step_size, obstacle_map):
    """Steers from 'from_node' towards 'to_node' within a step size, avoiding collisions."""
    direction = to_node - from_node[:2]
    length = min(np.linalg.norm(direction), step_size)
    direction = direction / np.linalg.norm(direction) * length
    new_point = from_node[:2] + direction
    if is_collision_free(new_point[0], new_point[1], obstacle_map):
        return new_point
    return None

def fuzzy_logic_controller():
    """Sets up the fuzzy logic controller for determining step size with expanded categories."""
    # Define new ranges for 'distance' and 'step'
    distance = ctrl.Antecedent(np.arange(0, 5000, 1), 'distance')
    step = ctrl.Consequent(np.arange(0, 300, 1), 'step')

    # Custom membership functions for distance
    distance['very_close'] = fuzz.trimf(distance.universe, [0, 0, 1250])
    distance['close'] = fuzz.trimf(distance.universe, [0, 1250, 2500])
    distance['medium'] = fuzz.trimf(distance.universe, [1250, 2500, 3750])
    distance['far'] = fuzz.trimf(distance.universe, [2500, 3750, 5000])
    distance['very_far'] = fuzz.trimf(distance.universe, [3750, 5000, 5000])

    # Custom membership functions for step size
    step['very_small'] = fuzz.trimf(step.universe, [0, 0, 75])
    step['small'] = fuzz.trimf(step.universe, [0, 75, 150])
    step['medium'] = fuzz.trimf(step.universe, [75, 150, 225])
    step['large'] = fuzz.trimf(step.universe, [150, 225, 300])
    step['very_large'] = fuzz.trimf(step.universe, [225, 300, 300])

    # Define rules
    rule1 = ctrl.Rule(distance['very_close'], step['very_small'])
    rule2 = ctrl.Rule(distance['close'], step['small'])
    rule3 = ctrl.Rule(distance['medium'], step['medium'])
    rule4 = ctrl.Rule(distance['far'], step['large'])
    rule5 = ctrl.Rule(distance['very_far'], step['very_large'])

    # Create the control system
    system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
    return ctrl.ControlSystemSimulation(system)

def adaptive_rrt_star(start, goal, obstacle_map, num_iterations=1500, goal_sampling_rate=0.1):
    """Implements the Adaptive RRT* algorithm."""
    start_time = time.time()  # Start timing
    fuzzy_controller = fuzzy_logic_controller()
    vertices = [np.array([*start, 0])]  
    edges = []
    goal_reached = False
    nodes_explored = 0  # Initialize the node counter

    for _ in range(num_iterations):
        rand_point = sample_free(obstacle_map, start, goal, goal_sampling_rate)
        nearest = nearest_vertex(rand_point, vertices)
        distance_to_goal = np.linalg.norm(goal - nearest[:2])
        fuzzy_controller.input['distance'] = distance_to_goal
        fuzzy_controller.compute()
        step_size = fuzzy_controller.output['step']
        new_point = steer(nearest, rand_point, step_size, obstacle_map)
        if new_point is not None:
            new_cost = nearest[2] + np.linalg.norm(new_point - nearest[:2])
            vertices.append(np.append(new_point, new_cost))
            edges.append((nearest, new_point))
            nodes_explored += 1  # Increment the node counter
            if np.linalg.norm(new_point - goal) < 50:
                vertices.append(np.array([*goal, new_cost + np.linalg.norm(goal - new_point)]))
                edges.append((new_point, goal))
                goal_reached = True
                break

    execution_time = time.time() - start_time  # Calculate the execution time
    path = reconstruct_path(vertices, edges, start, goal) if goal_reached else []
    return vertices, edges, path, execution_time, nodes_explored  

def reconstruct_path(vertices, edges, start, goal):
    """Reconstructs the path from the goal to the start using the edges."""
    if np.linalg.norm(vertices[-1][:2] - goal) >= 50:
        print("Goal not reached within threshold.")
        return []

    path = []
    current = vertices[-1]
    path.append(current[:2])

    while not np.array_equal(current[:2], start):
        for edge in reversed(edges):
            if np.array_equal(edge[1][:2], current[:2]):
                current = edge[0]
                path.append(current[:2])
                break
        else:
            print("Path reconstruction failed.")
            break

    return path[::-1]

def plot_map(obstacle_map, vertices, edges, start, goal, path):
    """Visualizes the results"""
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.imshow(cv2.cvtColor(obstacle_map, cv2.COLOR_BGR2RGB), origin='lower', extent=[0, map_width, 0, map_height])
    for vertex, new_vertex in edges:
        plt.plot([vertex[0], new_vertex[0]], [vertex[1], new_vertex[1]], 'c-', alpha=0.4)
    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')
    plt.plot(goal[0], goal[1], 'rx', markersize=10, label='Goal')
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'r-', linewidth=2, label='Path to Goal')
    plt.title('FA-RRT*N Path Planning')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    clearance = 50 #int(input("Enter robot clearance in mm: "))
    obstacle_map = create_map(map_width, map_height, clearance)
    start = np.array([500, 1000])  # Start position
    goal = np.array([5800, 1900])  # Goal position
    vertices, edges, path, execution_time, nodes_explored = adaptive_rrt_star(start, goal, obstacle_map)
    plot_map(obstacle_map, vertices, edges, start, goal, path)
    print("Execution time: {:.2f} seconds".format(execution_time))
    print("Number of nodes explored:", nodes_explored)
    path_meter = []
    for i in path:
        path_meter.append([i[0]/1000, i[1]/1000])
    print("Path in meters:", path_meter)
    total_distance = 0
    for i in range(len(path_meter)-1):
        total_distance += distance(np.array(path_meter[i]), np.array(path_meter[i+1]))
    print("Total distance in meters: {:.2f}".format(total_distance))

    pygame.init()
    display = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("CARLA Simulator")

    client, world = connect_to_carla()
    world = client.load_world('Town03')
    
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.tesla.model3'))
    
    spawn_point = carla.Transform(carla.Location(x=240, y=-7, z=10), carla.Rotation(pitch=0, yaw=180, roll=0))
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(carla.Location(x=250, y=-7, z=10), carla.Rotation(pitch=-15)))

    # List of waypoints for the vehicle to follow
    # waypoints = [np.array([240, -7]), np.array([220, -7]), np.array([200, -7]), np.array([170, -7]),
    #              np.array([155, -7]),np.array([150, -30]),np.array([150, -35]),np.array([150, -45]),
    #              np.array([150, -55]),np.array([150, -65]),np.array([130, -73]),
    #              np.array([110, -70])]
    """
    Make sure the waypoints generated by running the path palanning algorithm on the map are passed
    """

    waypoints = path_meter

    def update_camera():
        camera_location = vehicle.get_transform().location + carla.Location(x=15, z=10)
        camera_rotation = carla.Rotation(pitch=-15, yaw=180)
        spectator.set_transform(carla.Transform(camera_location, camera_rotation))

    def drive_to_waypoint(waypoint):
        destination = carla.Location(x=float(waypoint[0]), y=float(waypoint[1]), z=10)
        while True:
            vehicle_transform = vehicle.get_transform()
            vehicle_location = vehicle_transform.location
            vehicle_yaw = vehicle_transform.rotation.yaw
            distance = np.linalg.norm(np.array([vehicle_location.x, vehicle_location.y]) - waypoint)

            if distance < 2:  # When close enough to the waypoint
                print("Reached waypoint")
                break

            direction_vector = np.array([destination.x - vehicle_location.x, destination.y - vehicle_location.y])
            direction_norm = np.linalg.norm(direction_vector)
            if direction_norm > 0:
                normalized_direction = direction_vector / direction_norm
                target_yaw = math.degrees(math.atan2(normalized_direction[1], normalized_direction[0]))
                delta_yaw = target_yaw - vehicle_yaw
                delta_yaw = (delta_yaw + 180) % 360 - 180  # Normalize the angle to [-180, 180]

                steer = delta_yaw / 180.0  # Normalize steering
                steer = max(min(steer, 1.0), -1.0)  # Clamp values to [-1, 1]

                print(f"Steering: {steer}, Target Yaw: {target_yaw}, Vehicle Yaw: {vehicle_yaw}, Delta Yaw: {delta_yaw}")
                vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=steer))

            update_camera()
            world.tick()
            time.sleep(0.05)


    # Navigate through each waypoint
    for waypoint in waypoints:
        drive_to_waypoint(waypoint)
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))  # Stop at each waypoint
        time.sleep(1)  # Pause for clarity at each waypoint

    vehicle.destroy()
    pygame.quit()

if __name__ == '__main__':
    main()