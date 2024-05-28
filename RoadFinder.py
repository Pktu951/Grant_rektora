import numpy as np
from numpy import inf
import math
import LidarMap

class RoadFinder:
    def __init__(self, map_class: LidarMap, starting_point: tuple, ending_point: tuple,car_length=1.5, car_width=0.5):
        self.lidar_map = map_class.map
        self.car_length = car_length
        self.car_width = car_width
        self.starting_point = starting_point
        self.ending_point = ending_point
        self.path = []

    def __is_path_clear(self, start, offsets, condition_check):
        x, y = start
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(self.lidar_map) and 0 <= ny < len(self.lidar_map[0]):
                if condition_check(self.lidar_map[ny, nx]):
                    return False
        return True

    def __create_graph(self):
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
                if (nx, ny) in key_to_index and self.__is_path_clear((x, y), [(dx, dy)], lambda val: val == 0):
                    graph[idx].append(key_to_index[(nx, ny)])

        return graph, keys

    def __relax_edges(self, graph, distances, predecessor):
        num_vertices = len(graph)
        for _ in range(num_vertices - 1):
            for u in range(num_vertices):
                for v in graph[u]:
                    if distances[v] > distances[u] + 1:
                        distances[v] = distances[u] + 1
                        predecessor[v] = u

    def __reconstruct_path(self, predecessor, start_index, end_index):
        path_indices = []
        current = end_index
        while current != start_index:
            path_indices.append(current)
            current = predecessor[current]
            if predecessor[current] == -1:
                raise ValueError("Predecessor is -1")
        path_indices.append(start_index)
        path_indices.reverse()
        return path_indices

    def __bellman_ford(self, graph, keys):
        num_vertices = len(graph)
        start_index = keys.index(self.starting_point)
        end_index = keys.index(self.ending_point)
        distances = [inf] * num_vertices
        predecessor = [-1] * num_vertices

        distances[start_index] = 0
        print("graph", graph)

        self.__relax_edges(graph, distances, predecessor)
        print("pred", predecessor)
        path_indices = self.__reconstruct_path(predecessor, start_index, end_index)
        print("reconstr")
        self.path = [keys[idx] for idx in path_indices]
        return self.path

    def find_path(self):
        graph, keys = self.__create_graph()
        print("create graph")
        return self.__bellman_ford(graph, keys)

    def __repr__(self):
        return "[" + " -> ".join(map(str, self.path)) + "]"
