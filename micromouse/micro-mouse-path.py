## imports
import cv2
import numpy as np
import random
import math
from matplotlib import pyplot as plt

from collections import deque
import heapq

from my_graph import Graph, Node

## image file
image_file = "continuous_example_2.jpeg"
# image_file = "test1.jpg"

## parameters unsafe size is 5 10
unsafe_kernel_size = 55
unsafe_iterations = 3
closing_kernel_size = 54

nodes_row_amount = 5*2
nodes_col_amount = 9*2

random.seed(42)

iterations = 500
k_connections = 3
max_connection_range = 300
start_location = (100,100)
goal_location = (450,450)

# Display an occupancy map
def load_image(image_file_path):
    image = image_file_path
    image = cv2.imread(image_file, cv2.IMREAD_GRAYSCALE)
    return image


def perspective_transform(image):
    image = image.copy()
    top_corner = 255
    left_corner = 415
    right_corner = 380
    bot_corner = 190
    # pts1 = np.float32([[436, 270], [3445, 270], [436, 1955], [3445, 1955]])
    # pts2 = np.float32([[0, 0], [3000, 0], [0, 3000], [3000, 3000]])
    src_points = np.array([
        [left_corner, top_corner],
        [image.shape[1] - right_corner, top_corner],
        [image.shape[1] - right_corner, image.shape[0] - bot_corner],
        [left_corner, image.shape[0] - bot_corner]
    ], dtype=np.float32)
    dst_points = np.array([
        [0, 0],
        [image.shape[1], 0],
        [image.shape[1], image.shape[0]],
        [0, image.shape[0]]
    ], dtype=np.float32)
    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    # matrix = cv2.getPerspectiveTransform(pts1, pts2)
    image = cv2.warpPerspective(image, matrix, (image.shape[1], image.shape[0]))
    return image


# Binary threshold 75
def binary_threshold(image, threshold_value=75):
    _, binary_image = cv2.threshold(image, threshold_value, 255, cv2.THRESH_BINARY)
    return binary_image

# Morphological Operations
def apply_closing(binary_image, kernel_size):
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    closed_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    return closed_image

def apply_opening(binary_image, kernel_size):
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    closed_image = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel)
    return closed_image

def remove_tiny_dots(image, min_size, remove_small=True):
    # Ensure the image is in grayscale
    copied_image = image.copy()

    if len(copied_image.shape) == 3:
        copied_image = cv2.cvtColor(copied_image, cv2.COLOR_BGR2GRAY)

    # Invert the image to treat black regions as foreground
    inverted_image = cv2.bitwise_not(copied_image)
    
    # Find all connected components (now black blobs in the inverted image)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(inverted_image, connectivity=8)
    print(f"Number of connected components: {num_labels}")
    
    # Create an output image to store the results
    
    # Keep only components larger than a certain size
    for i in range(1, num_labels):  # Skip the background label 0 (background)
        if (stats[i, cv2.CC_STAT_AREA] < min_size and remove_small):
            inverted_image[labels == i] = 0  # Set small components to black
        elif stats[i, cv2.CC_STAT_AREA] > min_size and not remove_small:
            inverted_image[labels == i] = 0

    cleaned_image = cv2.bitwise_not(inverted_image)
    
    return cleaned_image, inverted_image

def process_occupancy_map(image, kernel_size, iterations):
    kernel = np.ones((kernel_size, kernel_size), np.uint8) 
    unsafe_zone = cv2.erode(image, kernel, iterations=iterations)
    border = cv2.absdiff(unsafe_zone, image)
    color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    mask = border > 0
    color_image[mask] = [0, 0, 0]
    return color_image

def path_clear_vert_hor(image, x1, y1, x2, y2):
    if x1 == x2:  # Vertical line
        for y in range(min(y1, y2), max(y1, y2) + 1):
            if image[y, x1] != 255:
                return False
    elif y1 == y2:  # Horizontal line
        for x in range(min(x1, x2), max(x1, x2) + 1):
            if image[y1, x] != 255:
                return False
    else:
        return False
    return True

def gen_grid(image, rows, cols):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    node_id = 0
    height, width = image.shape
    print(image.shape)

    # x_node_dist = float(width // (size_of_node_grid + 1))
    x_node_dist = float(width // (cols))
    y_node_dist = float(height // (rows)) 
    # y_node_dist = float(height // (size_of_node_grid + 1))

    graph = Graph()

    ## Adding Nodes
    for i in range(1, rows):
        for j in range(1, cols):
            x = int(j * x_node_dist)
            y = int(i * y_node_dist)
            graph.add_node(node_id, x, y)
            node_id += 1
    
    # Connecting Nodes (Only neighboring nodes)
    for node_id1 in graph.get_nodes().keys():
        x1, y1 = graph.get_nodes()[node_id1].get_point()

        # Define possible neighbors (8 directions: up, down, left, right, and diagonals)
        neighbors = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # Horizontal and vertical
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonals
        ]

        for dx, dy in neighbors:
            neighbor_x = x1 + int(dx * x_node_dist)
            neighbor_y = y1 + int(dy * y_node_dist)

            # Find node_id2 corresponding to neighbor coordinates
            for node_id2 in graph.get_nodes().keys():
                x2, y2 = graph.get_nodes()[node_id2].get_point()
                if neighbor_x == x2 and neighbor_y == y2:
                    if path_clear_v2(image, x1, y1, x2, y2):
                        # graph.add_edge(node_id1, node_id2, 1)
                        # graph.add_edge(node_id1, node_id2, euclidean_distance(x1, y1, x2, y2))
                        weight = 0.0
                        ## if diagonal make it double weight
                        if dx != 0 and dy != 0:
                            weight = 2.1
                        else:
                            weight = 1.0
                        graph.add_edge(node_id1, node_id2, weight)

    return graph

def euclidean_distance(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

# This is a useful function which you may choose to impliment and use 
# It finds and returns the n closest nodes which are within the range
def find_closest_nodes(image ,graph, target_x, target_y, n,range):
    # print(graph.get_nodes())
    
    distances = []
    for node in graph.get_nodes().values():
        dist = euclidean_distance(node.x, node.y, target_x, target_y)
        if dist <= range:
            if (node.x, node.y) != (target_x, target_y):
                if prm_path_clear(image, node.x, node.y, target_x, target_y):
                    distances.append((node.id, dist))

    distances.sort(key=lambda x: x[1])
    return [node_id for node_id, _ in distances[:n]]


def path_clear_v2(image, x1, y1, x2, y2):
    num_points = max(abs(x2 - x1), abs(y2 - y1))

    x_points = np.linspace(x1, x2, num_points).astype(int)
    y_points = np.linspace(y1, y2, num_points).astype(int)
    
    for x, y in zip(x_points, y_points):
        if image[y, x] != 255:
            return False
    return True

def modify_image_with_grid(graph, image):
    display_image = image.copy()
    
    # Draw nodes
    for node in graph.get_nodes().values():
        cv2.circle(display_image, (node.x, node.y), 20, (0, 255, 0), -1) 
    
    # # Draw edges
    for node_id1, edges in graph.edges.items():
        for node_id2 in edges:
            x1, y1 = graph.nodes[node_id1].get_point()
            x2, y2 = graph.nodes[node_id2].get_point()
            cv2.line(display_image, (x1, y1), (x2, y2), (0, 125, 0), 10) 
            ## label each node with each node id
            cv2.putText(display_image, str(node_id1), (x1, y1), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 0), 10, cv2.LINE_AA)

    return display_image

def display_grid_with_path(graph, image, path):
    display_image = image.copy()
    for node in graph.get_nodes().values():
        cv2.circle(display_image, (node.x, node.y), 20, (0, 255, 0), -1)
    for node_id1, edges in graph.edges.items():
        for node_id2 in edges:
            x1, y1 = graph.nodes[node_id1].get_point()
            x2, y2 = graph.nodes[node_id2].get_point()
            cv2.line(display_image, (x1, y1), (x2, y2), (0, 125, 0), 10)
            cv2.putText(display_image, str(node_id1), (x1, y1), cv2.FONT_HERSHEY_PLAIN, 2.5, (0, 0, 0), 10, cv2.LINE_AA)

    for i in range(len(path) - 1):
        x1, y1 = graph.nodes[path[i]].get_point()
        x2, y2 = graph.nodes[path[i + 1]].get_point()
        cv2.line(display_image, (x1, y1), (x2, y2), (255, 0, 0), 10)

    return display_image

def bfs(graph, start_node_id, end_node_id):
    queue = deque()
    visited = set()
    
    queue.append(start_node_id)
    paths = {start_node_id: [start_node_id]}
    
    while queue:
        current_node = queue.popleft()
        
        if current_node == end_node_id: return paths[current_node]
        
        for neighbor in graph.edges.get(current_node, []):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append(neighbor)
                paths[neighbor] = paths[current_node] + [neighbor]
    
    return []

def dijkstra(graph, start_node_id, end_node_id):
    # Priority queue to handle nodes with cumulative weight (distance)
    queue = []
    heapq.heappush(queue, (0, start_node_id))  # (cumulative_weight, node_id)
    
    visited = set()
    distances = {start_node_id: 0}  # Dictionary to store the shortest distance to each node
    paths = {start_node_id: [start_node_id]}  # Dictionary to store the path to each node
    
    while queue:
        current_distance, current_node = heapq.heappop(queue)
        
        if current_node in visited:
            continue
        
        visited.add(current_node)
        
        # If we reached the destination, return the path
        if current_node == end_node_id:
            return paths[current_node]
        
        # Explore neighbors
        for neighbor, weight in graph.edges.get(current_node, {}).items():
            distance = current_distance + weight
            
            if neighbor not in distances or distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(queue, (distance, neighbor))
                paths[neighbor] = paths[current_node] + [neighbor]
    
    # If the destination is not reachable, return an empty list
    return []

def shift_and_thicken_walls(image, shift_factor):
    height, width = image.shape[:2]
    center_x, center_y = width // 2, height // 2

    # Create a new image to store the shifted and thickened walls
    shifted_image = image.copy()
    if len(image.shape) == 3:  # Check if image has more than one channel
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    for y in range(height):
        for x in range(width):
            if image[y, x] == 0:  # Only consider black pixels as part of the walls
                dx, dy = 0, 0
                # Calculate the shift based on the quadrant
                if x < center_x and y < center_y:  # Top left quadrant
                    dx = int(shift_factor * (center_x - x) / center_x)
                    dy = int(shift_factor * (center_y - y) / center_y)
                elif x >= center_x and y < center_y:  # Top right quadrant
                    dx = -int(shift_factor * (x - center_x) / center_x)
                    dy = int(shift_factor * (center_y - y) / center_y)
                elif x < center_x and y >= center_y:  # Bottom left quadrant
                    dx = int(shift_factor * (center_x - x) / center_x)
                    dy = -int(shift_factor * (y - center_y) / center_y)
                elif x >= center_x and y >= center_y:  # Bottom right quadrant
                    dx = -int(shift_factor * (x - center_x) / center_x)
                    dy = -int(shift_factor * (y - center_y) / center_y)

                new_x = min(max(x + dx, 0), width - 1)
                new_y = min(max(y + dy, 0), height - 1)

                # Draw a line between the original position and the new shifted position
                cv2.line(shifted_image, (x, y), (new_x, new_y), (0, 0, 0), thickness=2)

    return shifted_image


def shift_walls(image, shift_factor):
    height, width = image.shape[:2]
    center_x, center_y = width // 2, height // 2

    # Create a new image to store the shifted and thickened walls
    shifted_image = image.copy()
    if len(image.shape) == 3:  # Check if image has more than one channel
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    for y in range(height):
        for x in range(width):
            if image[y, x] == 0:  # Only consider black pixels as part of the walls
                dx, dy = 0, 0
                # Calculate the shift based on the quadrant
                if x < center_x and y < center_y:  # Top left quadrant
                    dx = int(shift_factor * (center_x - x) / center_x)
                    dy = int(shift_factor * (center_y - y) / center_y)
                elif x >= center_x and y < center_y:  # Top right quadrant
                    dx = -int(shift_factor * (x - center_x) / center_x)
                    dy = int(shift_factor * (center_y - y) / center_y)
                elif x < center_x and y >= center_y:  # Bottom left quadrant
                    dx = int(shift_factor * (center_x - x) / center_x)
                    dy = -int(shift_factor * (y - center_y) / center_y)
                elif x >= center_x and y >= center_y:  # Bottom right quadrant
                    dx = -int(shift_factor * (x - center_x) / center_x)
                    dy = -int(shift_factor * (y - center_y) / center_y)

                new_x = min(max(x + dx, 0), width - 1)
                new_y = min(max(y + dy, 0), height - 1)

                # Assign the new shifted position and remove the original
                shifted_image[y, x] = 255
                shifted_image[new_y, new_x] = 0

    return shifted_image

def crop_image(image, crop_amount):
    height, width = image.shape[:2]
    
    # Crop the image by the specified number of pixels on all four sides
    cropped_image = image.copy()
    cropped_image = image[crop_amount:height-crop_amount, crop_amount:width-crop_amount]
    
    return cropped_image


# Load and process the image
image = load_image(image_file) # image is returned as grayscale
image = perspective_transform(image) # image stays as grescale
binary_image = binary_threshold(image) # image is returned as binary

# Apply closing to thicken and connect the lines
# closed_image = apply_closing(binary_image, closing_kernel_size)
opened_image = apply_opening(binary_image, 3) # image stays as binary
closed_image = apply_closing(opened_image, closing_kernel_size) # image stays as binary

# for dots removed, 200 works when the image hasn't been thicked yet
# 50000
dots_removed, inverted_image = remove_tiny_dots(closed_image, 1000, remove_small=True)
# processed_image = process_occupancy_map(dots_removed, unsafe_kernel_size, unsafe_iterations)

# 3700 picks up on lone walls
lone_walls, inverted_image_lone_walls = remove_tiny_dots(dots_removed, 3700, remove_small=False)

# 5500 picks up on all obstacles
all_obstacles, inverted_image_all_obstacles = remove_tiny_dots(dots_removed, 55000, remove_small=False)

just_ob_inverted = cv2.bitwise_xor(inverted_image_all_obstacles, inverted_image_lone_walls)
just_obstacles = cv2.bitwise_not(just_ob_inverted)

no_obstacles_inverted = cv2.bitwise_not(dots_removed)
no_obstacles_inverted = cv2.bitwise_xor(no_obstacles_inverted, just_ob_inverted)
no_obstacles = cv2.bitwise_not(no_obstacles_inverted)

shift_factor = 200  # Adjust this value as needed
shifted_image = shift_walls(no_obstacles, shift_factor)

shifted_with_obst = cv2.bitwise_and(just_obstacles, shifted_image)

final_image = process_occupancy_map(shifted_with_obst, unsafe_kernel_size, unsafe_iterations)
cropped_final = crop_image(final_image, shift_factor)

# Generate the grid
graph = gen_grid(cropped_final, nodes_row_amount, nodes_col_amount)

# Modify the image with the grid
graph_image = modify_image_with_grid(graph, cropped_final)


path = dijkstra(graph, 0, 16)
pathed_image = display_grid_with_path(graph, cropped_final, path)



## Nice Display Section of Images for calibration and testing purposes
plt.figure()
plt.subplot(3, 4, 1)
plt.imshow(image, cmap='gray')
plt.title("Original Image")

plt.subplot(3, 4, 2)
plt.imshow(binary_image, cmap='gray')
plt.title("Binary Threshold")

plt.subplot(3, 4, 3)
plt.imshow(opened_image, cmap='gray')
plt.title("Opened Image")

plt.subplot(3, 4, 4)
plt.imshow(closed_image, cmap='gray')
plt.title("Closed Image")

plt.subplot(3, 4, 5)
plt.imshow(dots_removed, cmap='gray')
plt.title("dots removed Image")

plt.subplot(3, 4, 6)
plt.imshow(lone_walls, cmap='gray')
plt.title("lone walls Image")

plt.subplot(3, 4, 7)
plt.imshow(all_obstacles, cmap='gray')
plt.title("all obstacles Image")

plt.subplot(3, 4, 8)
plt.imshow(just_obstacles, cmap='gray')
plt.title("Only Obstacles Image")

plt.subplot(3, 4, 9)
plt.imshow(shifted_image, cmap='gray')
plt.title("Shifted Walls Image")

plt.subplot(3, 4, 10)
plt.imshow(shifted_with_obst, cmap='gray')
plt.title("Shifted with Obstacles added in Image")

plt.subplot(3, 4, 11)
plt.imshow(graph_image, cmap='gray')
plt.title("Final Image Cropped")

plt.subplot(3, 4, 12)
plt.imshow(pathed_image, cmap='gray')
plt.title("Graph Image with Path")

plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05, wspace=0.2, hspace=0.2)
# plt.show()

plt.figure()
plt.imshow(pathed_image, cmap='gray')
plt.title("Graph Image with Path")
plt.show()

