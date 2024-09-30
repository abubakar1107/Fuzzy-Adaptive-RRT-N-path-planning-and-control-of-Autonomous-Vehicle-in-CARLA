import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
import time  


map_width, map_height = 6000, 2000  # Map dimensions in pixels

def create_map(width, height, clearance):
    """Creates an obstacle map with specified clearance."""
    obstacle_map = np.ones((height, width, 3), dtype=np.uint8) * 255
    cv2.rectangle(obstacle_map, (1500 - clearance, 0), (1750 + clearance, 1000 + clearance), (255, 0, 0), -1)
    cv2.rectangle(obstacle_map, (2500 - clearance, 1000), (2750 + clearance, 2000 + clearance), (0, 255, 0), -1)
    cv2.circle(obstacle_map, (4200, 800), 600 + clearance, (0, 0, 255), -1)
    return obstacle_map

def is_collision_free(x, y, obstacle_map):
    """Checks if the given position (x, y) is free from collisions."""
    return 0 <= x < obstacle_map.shape[1] and 0 <= y < obstacle_map.shape[0] and np.all(obstacle_map[int(y), int(x)] == [255, 255, 255])

def sample_free(obstacle_map):
    """Randomly samples a free point from the obstacle map."""
    while True:
        point = np.random.rand(2) * np.array([map_width, map_height])
        if is_collision_free(point[0], point[1], obstacle_map):
            return point

def distance(point1, point2):
    """Calculates Euclidean distance between two points."""
    return np.linalg.norm(point1 - point2)

def nearest_vertex(point, vertices):
    """Finds the nearest vertex in the vertices list to the given point."""
    return min(vertices, key=lambda v: np.linalg.norm(point - v[:2]))

def steer(from_node, to_node, step_size):
    """Steers from 'from_node' towards 'to_node' within a step size, avoiding collisions."""
    direction = to_node - from_node[:2]
    length = min(np.linalg.norm(direction), step_size)
    direction = direction / np.linalg.norm(direction) * length
    new_point = from_node[:2] + direction
    return new_point

def rrt_star(start, goal, obstacle_map, num_iterations=1500, step_size=100):
    """Implements the RRT* algorithm."""
    vertices = [np.array([*start, 0])] 
    edges = []
    start_time = time.time()  # Start timing

    for _ in range(num_iterations):
        rand_point = sample_free(obstacle_map)
        nearest = nearest_vertex(rand_point, vertices)
        new_point = steer(nearest, rand_point, step_size)
        if is_collision_free(new_point[0], new_point[1], obstacle_map):
            new_cost = nearest[2] + np.linalg.norm(new_point - nearest[:2])
            vertices.append(np.append(new_point, new_cost))
            edges.append((nearest, new_point))
            if np.linalg.norm(new_point - goal) < step_size:
                vertices.append(np.array([*goal, new_cost + np.linalg.norm(goal - new_point)]))
                edges.append((new_point, goal))
                print("Goal reached.")
                break

    execution_time = time.time() - start_time  # End timing
    path = reconstruct_path(vertices, edges, start, goal)
    return vertices, edges, path, execution_time, len(vertices)

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

    return path[::-1]  # Reverse the path to start from the start position

def plot_map(obstacle_map, vertices, edges, start, goal, path):
    """Visualizes the results, including the path from start to goal."""
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.imshow(cv2.cvtColor(obstacle_map, cv2.COLOR_BGR2RGB), origin='lower', extent=[0, map_width, 0, map_height])

    # Plot all edges with light colors for exploration
    for vertex, new_vertex in edges:
        plt.plot([vertex[0], new_vertex[0]], [vertex[1], new_vertex[1]], 'c-', alpha=0.4)  # Cyan for exploration edges

    # Highlight the start and goal with distinct markers
    plt.plot(start[0], start[1], 'go', markersize=10, label='Start')  # Green circle for start
    plt.plot(goal[0], goal[1], 'rx', markersize=10, label='Goal')  # Red 'X' for goal

    # Plot the path in red if it exists
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, 'r-', linewidth=2, label='Path to Goal')  # Red for the path

    plt.title('RRT* Path Planning')
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
    vertices, edges, path, execution_time, num_vertices = rrt_star(start, goal, obstacle_map)
    print(f"Execution Time: {execution_time:.2f} seconds")
    print(f"Number of Nodes Explored: {num_vertices}")
    plot_map(obstacle_map, vertices, edges, start, goal, path)

    total_distance = 0
    for i in range(len(path)-1):
        total_distance += distance(np.array(path[i]/1000), np.array(path[i+1]/1000))
    print("Total distance in meters: {:.2f}".format(total_distance))

if __name__ == '__main__':
    main()
