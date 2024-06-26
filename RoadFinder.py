import numpy as np
<<<<<<< HEAD
=======
from numpy import inf
>>>>>>> ec69ec9d1a0e5ce4a2f75d727c7bed9cc0038f63
import math
import LidarMap

class RoadFinder:
<<<<<<< HEAD
    def __init__(self, map_class: LidarMap, starting_point: tuple, ending_point: tuple, car_length=1.5, car_width=0.5):
        if not hasattr(map_class, 'map'):
            raise ValueError("LidarMap class must have a 'map' attribute")
=======
    def __init__(self, map_class: LidarMap, starting_point: tuple, ending_point: tuple,car_length=1.5, car_width=0.5):
>>>>>>> ec69ec9d1a0e5ce4a2f75d727c7bed9cc0038f63
        self.lidar_map = map_class.map
        self.car_length = car_length
        self.car_width = car_width
        self.starting_point = starting_point
        self.ending_point = ending_point
        self.path = []

<<<<<<< HEAD
    def __is_path_clear(self, start, dx, dy):
        x, y = start
        nx, ny = x + dx, y + dy
        if not (0 <= nx < len(self.lidar_map) and 0 <= ny < len(self.lidar_map[0])):
            return False
        return self.lidar_map[nx][ny] == 0

    def __create_graph(self):
        keys = [(i, j) for i, row in enumerate(self.lidar_map) for j, val in enumerate(row) if not val]
        if not keys:
            raise ValueError("No valid path in the lidar map")
        key_to_index = {key: idx for idx, key in enumerate(keys)}
        graph = {i: [] for i in range(len(keys))}
        edge_weights = {}

        for idx, (x, y) in enumerate(keys):
            neighbors = [
                (-1, 0), (1, 0), (0, -1), (0, 1),  
                (-1, -1), (-1, 1), (1, -1), (1, 1)  
            ]
            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if (nx, ny) in key_to_index and self.__is_path_clear((x, y), dx, dy):
                    neighbor_index = key_to_index[(nx, ny)]
                    graph[idx].append(neighbor_index)
                    if abs(dx) + abs(dy) == 1:  
                        edge_weights[(idx, neighbor_index)] = 1
                    else:  # diagonal move
                        edge_weights[(idx, neighbor_index)] = math.sqrt(2)

        print("Graph:", graph)
        print("Keys:", keys)
        print("Edge Weights:", edge_weights)

        return graph, keys, edge_weights

    def __relax_edges(self, graph, edge_weights, distances, predecessor):
=======
    def __is_path_clear(self, start, offsets, condition_check):
        x, y = start
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(self.lidar_map) and 0 <= ny < len(self.lidar_map[0]):
                if condition_check(self.lidar_map[nx][ny]):
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
>>>>>>> ec69ec9d1a0e5ce4a2f75d727c7bed9cc0038f63
        num_vertices = len(graph)
        for _ in range(num_vertices - 1):
            for u in range(num_vertices):
                for v in graph[u]:
<<<<<<< HEAD
                    weight = edge_weights[(u, v)]
                    if distances[v] > distances[u] + weight:
                        distances[v] = distances[u] + weight
                        predecessor[v] = u

        print("Distances after relaxation:", distances)
        print("Predecessor after relaxation:", predecessor)

=======
                    if distances[v] > distances[u] + 1:
                        distances[v] = distances[u] + 1
                        predecessor[v] = u

>>>>>>> ec69ec9d1a0e5ce4a2f75d727c7bed9cc0038f63
    def __reconstruct_path(self, predecessor, start_index, end_index):
        path_indices = []
        current = end_index
        while current != start_index:
            path_indices.append(current)
            current = predecessor[current]
<<<<<<< HEAD
            if current == -1:  # path not found
                return []
=======
>>>>>>> ec69ec9d1a0e5ce4a2f75d727c7bed9cc0038f63
        path_indices.append(start_index)
        path_indices.reverse()
        return path_indices

<<<<<<< HEAD
    def __bellman_ford(self, graph, keys, edge_weights):
        if self.starting_point not in keys or self.ending_point not in keys:
            raise ValueError("Starting or ending point is not in the lidar map")
        num_vertices = len(graph)
        start_index = keys.index(self.starting_point)
        end_index = keys.index(self.ending_point)
        distances = [np.inf] * num_vertices
=======
    def __bellman_ford(self, graph, keys):
        num_vertices = len(graph)
        start_index = keys.index(self.starting_point)
        end_index = keys.index(self.ending_point)
        distances = [inf] * num_vertices
>>>>>>> ec69ec9d1a0e5ce4a2f75d727c7bed9cc0038f63
        predecessor = [-1] * num_vertices

        distances[start_index] = 0

<<<<<<< HEAD
        self.__relax_edges(graph, edge_weights, distances, predecessor)

        if distances[end_index] == np.inf:  # No path found
            return []
        
        path_indices = self.__reconstruct_path(predecessor, start_index, end_index)
        if not path_indices:
            return []
        self.path = [keys[idx] for idx in path_indices]

        print("Path indices:", path_indices)
        print("Final path:", self.path)

        return self.path

    def find_path(self):
        graph, keys, edge_weights = self.__create_graph()
        return self.__bellman_ford(graph, keys, edge_weights)

    def __repr__(self):
        return "[" + " -> ".join(map(str, self.path)) + "]"
=======
        self.__relax_edges(graph, distances, predecessor)

        path_indices = self.__reconstruct_path(predecessor, start_index, end_index)
        self.path = [keys[idx] for idx in path_indices]
        return self.path

    def find_path(self):
        graph, keys = self.__create_graph()
        return self.__bellman_ford(graph, keys)

    def __repr__(self):
        return "[" + " -> ".join(map(str, self.path)) + "]"
>>>>>>> ec69ec9d1a0e5ce4a2f75d727c7bed9cc0038f63
