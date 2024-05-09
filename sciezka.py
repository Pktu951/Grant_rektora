import numpy as np
from numpy import inf
import math
import LidarMap

class Road:
    def __init__(self, map_class: LidarMap, starting_point: tuple, ending_point: tuple, resolution: int, car_length=1.5, car_width=0.5):
        # Initialize the Road class with map data and vehicle dimensions
        self.lidar_map = map_class.map  # Lidar map of the road
        self.resolution = resolution    # Resolution of the map
        self.car_length = car_length    # Length of the car
        self.car_width = car_width      # Width of the car
        self.starting_point = starting_point  # Starting point on the map
        self.ending_point = ending_point      # Ending point on the map
        self.path = []  # This will store the calculated path

    def is_path_clear(self, start, offsets, condition_check):
        # Helper function to check if the path is clear of obstacles
        x, y = start  # Starting coordinates
        for dx, dy in offsets:  # Iterate through the provided offsets
            nx, ny = x + dx, y + dy  # Calculate new coordinates based on offset
            # Check boundaries and condition
            if 0 <= nx < len(self.lidar_map) and 0 <= ny < len(self.lidar_map[0]):
                if condition_check(self.lidar_map[nx][ny]):
                    return False  # Path is not clear
        return True  # Path is clear

    def create_graph(self):
        # Create graph from Lidar map data for pathfinding
        keys = [(i, j) for i, row in enumerate(self.lidar_map) for j, val in enumerate(row) if not val]  # Map coordinates with no obstacles
        key_to_index = {key: idx for idx, key in enumerate(keys)}  # Mapping of coordinates to indices
        graph = {i: [] for i in range(len(keys))}  # Graph initialization

        # Define possible moves from each point (up, left, right, diagonal up-left, and diagonal up-right)
        for idx, (x, y) in enumerate(keys):
            neighbors = [
                (-1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1)
            ]
            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if (nx, ny) in key_to_index and self.is_path_clear((x, y), [(dx, dy)], lambda val: val == 0):
                    graph[idx].append(key_to_index[(nx, ny)])  # Add clear paths to the graph

        return graph, keys

    def bellman_ford(self, graph, keys):
        # Implementation of the Bellman-Ford algorithm to find the shortest path
        num_vertices = len(graph)
        start_index = keys.index(self.starting_point)
        end_index = keys.index(self.ending_point)
        distances = [inf] * num_vertices  # Start with infinite distances
        predecessor = [-1] * num_vertices  # Start with no predecessors

        distances[start_index] = 0  # Distance to the start is 0

        # Relax edges repeatedly
        for _ in range(num_vertices - 1):
            for u in range(num_vertices):
                for v in graph[u]:
                    if distances[v] > distances[u] + 1:
                        distances[v] = distances[u] + 1
                        predecessor[v] = u

        # Reconstruct the path
        path_indices = []
        current = end_index
        while current != start_index:
            path_indices.append(current)
            current = predecessor[current]
        path_indices.append(start_index)
        path_indices.reverse()

        self.path = [keys[idx] for idx in path_indices]

    def find_path(self):
        # Public method to find the path using internal methods
        graph, keys = self.create_graph()
        self.bellman_ford(graph, keys)

    def __repr__(self):
        # String representation of the class showing the calculated path
        return "[" + " -> ".join(map(str, self.path)) + "]"