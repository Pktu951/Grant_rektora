import numpy as np
from numpy import inf
import math
import LidarMap

class Road:
    def __init__(self, map_class: LidarMap, starting_point: tuple, ending_point: tuple, resolution: float, car_length=1.5, car_width=0.5):
        self.lidar_map = map_class.map
        self.resolution = resolution
        self.car_length = car_length
        self.car_width = car_width
        self.starting_point = starting_point
        self.ending_point = ending_point
        self.path = []

    def is_path_clear(self, start, offsets, condition_check):
        x, y = start
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(self.lidar_map) and 0 <= ny < len(self.lidar_map[0]):
                if condition_check(self.lidar_map[nx][ny]):
                    return False
        return True

    def create_graph(self):
        keys = [(i, j) for i, row in enumerate(self.lidar_map) for j, val in enumerate(row) if not val]
        key_to_index = {key: idx for idx, key in enumerate(keys)}
        graph = {i: [] for i in range(len(keys))}

        for idx, (x, y) in enumerate(keys):
            neighbors = [
                (-1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1)
            ]
            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if (nx, ny) in key_to_index and self.is_path_clear((x, y), [(dx, dy)], lambda val: val == 0):
                    graph[idx].append(key_to_index[(nx, ny)])

        return graph, keys

    def relax_edges(self, graph, distances, predecessor):
        num_vertices = len(graph)
        for _ in range(num_vertices - 1):
            for u in range(num_vertices):
                for v in graph[u]:
                    if distances[v] > distances[u] + 1:
                        distances[v] = distances[u] + 1
                        predecessor[v] = u

    def reconstruct_path(self, predecessor, start_index, end_index):
        path_indices = []
        current = end_index
        while current != start_index:
            path_indices.append(current)
            current = predecessor[current]
        path_indices.append(start_index)
        path_indices.reverse()
        return path_indices

    def bellman_ford(self, graph, keys):
        num_vertices = len(graph)
        start_index = keys.index(self.starting_point)
        end_index = keys.index(self.ending_point)
        distances = [inf] * num_vertices
        predecessor = [-1] * num_vertices

        distances[start_index] = 0

        self.relax_edges(graph, distances, predecessor)

        path_indices = self.reconstruct_path(predecessor, start_index, end_index)
        self.path = [keys[idx] for idx in path_indices]

    def find_path(self):
        graph, keys = self.create_graph()
        self.bellman_ford(graph, keys)

    def __repr__(self):
        return "[" + " -> ".join(map(str, self.path)) + "]"
