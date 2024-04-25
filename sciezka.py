import numpy as np
from numpy import inf
import math
import LidarMap

class Road:
    def __init__(self, map_class : LidarMap, starting_point : tuple, ending_point : tuple, resolution : LidarMap, car_length = 1.5, car_width = 0.5):
        self.lidar_map = map_class.map
        self.path = []
        self.resolution = resolution.resolution
        self.car_length = car_length
        self.car_width = car_width

        self.starting_point = starting_point
        self.ending_point = ending_point
    def defining_keys(self):
        keys = []
        for i in range (len(self.lidar_map)):
            for j in range (len(self.lidar_map[0])):
                if not self.lidar_map[i][j]:
                    keys.append((i,j)) # dodaje współrzędne przyszłych wierzchołków grafu (wszystkie punkty które są zerami)
        return keys
    def creating_possible_paths(self, keys):
        graph = {}
        slant = []
        for k,v in enumerate (keys):
            values = []
            # wykonuje pięć sprawdzeń dla przejścia w przód, w lewo, w prawo, na skos w lewo i na skos w prawo
            if v[0]-1 >= 0:
                if not self.lidar_map[v[0]-1][v[1]]:
                    help = True
                    for x in range(1, math.ceil(self.car_width)):
                        if v[1] + x < len(self.lidar_map):
                            if self.lidar_map[v[0] - 1][v[1] + x]:
                                help = False
                    if help == True:
                        values.append(keys.index((v[0]-1,v[1])))

            if v[1]-1 >= 0:
                if not self.lidar_map[v[0]][v[1]-1]:
                    help = True
                    for x in range(1, math.ceil(self.car_width)):
                        if v[0] - x < len(self.lidar_map[0]):
                            if self.lidar_map[v[0] - x][v[1]]:
                                help = False
                    for x in range(1, math.ceil(self.car_length)):
                        if v[1] + x < len(self.lidar_map):
                            if self.lidar_map[v[0]][v[1] + x]:
                                help = False
                    if help == True:
                        values.append(keys.index((v[0],v[1]-1)))

            if v[1]+1 <= len(self.lidar_map[0]) - 1:
                if not self.lidar_map[v[0]][v[1]+1]:
                    help = True
                    for x in range(1, math.ceil(self.car_width)):
                        if v[0] - x < len(self.lidar_map[0]):
                            if self.lidar_map[v[0] + x][v[1]]:
                                help = False
                    for x in range(1, math.ceil(self.car_length)):
                        if v[1] + x < len(self.lidar_map):
                            if self.lidar_map[v[0]][v[1] - x]:
                                help = False
                    if help == True:
                        values.append(keys.index((v[0],v[1]+1)))

            if v[0]-1 >= 0 and v[1]-1 >= 0:
                if not self.lidar_map[v[0]-1][v[1]-1]:
                    help = True
                    for x in range(1, math.ceil(self.car_width)):
                        if v[1] + x < len(self.lidar_map):
                            if self.lidar_map[v[0] - 1][v[1] - 1 + x]:
                                help = False
                    for x in range(1, math.ceil(self.car_length)):
                        if v[0] + x < len(self.lidar_map[0]):
                            if self.lidar_map[v[0] + x][v[1] - 1]:
                                help = False
                    if help == True:
                        values.append(keys.index(v[0]-1,v[1]-1))
                        slant.append((k,keys.index(v[0]-1,v[1]-1))) # do slanta dodaje możliwe przejście na skos czyli index w keyach rozpatrywanego wierzchołka oraz indeks w keyach wierzchołka na skos

            if v[0]-1 >= 0 and v[1]+1 <= len(self.lidar_map[0]) - 1:
                if not self.lidar_map[v[0]-1][v[1]+1]:
                    help = True
                    for x in range(1, math.ceil(self.car_width)):
                        if v[1] + x < len(self.lidar_map):
                            if self.lidar_map[v[0] - 1][v[1] + 1 + x]:
                                help = False
                    for x in range(1, math.ceil(self.car_length)):
                        if v[0] + x < len(self.lidar_map[0]):
                            if self.lidar_map[v[0] + x][v[1] + 1]:
                                help = False
                    if help == True:
                        values.append(keys.index((v[0]-1,v[1]+1)))
                        slant.append((k,keys.index((v[0]-1,v[1]+1))))
            graph[k] = values # graf ma mieć jako klucze numery wierzchołków od 0 do len(keys) - 1, a jako wartości ma mieć indexy do których można przejść z danego wiezchołka
        return graph, slant
    def bellman_ford(self, graph, keys, wage_function):
        V = list(graph.keys()) # lista wierzchołków
        start_index = keys.index((self.starting_point))
        end_index = keys.index((self.ending_point))
        d = [inf for v in range (len(V))] # stworzenie listy kosztów dojścia z wierzchołka startowego do innych wierzchołków, początkowo ustawiam na inf
        p = [-1 for v in range (len(V))] # stworzenie listy poprzedników danego wierzchołka na najkrótszej ścieżce, początkowo ustawiam na -1
        d[start_index] = 0 # koszt dotarcia do wierzchołka startowego jest równy 0
        for i in range (1, len(V)-1): # poszukuje najtańszych ścieżek, w każdej iteracji uzupełniamy koszt dojścia do co najmniej jednego wierzchołka,
            # a posiadając już koszt dojścia do wierzchołka startowego, pozostaje nam n-1 wierzchołków
            for u in V:
                for v in graph[u]: # sprawdzenie każdej krawędzi
                    if d[v] > d[u] + wage_function[u][v]: # sprawdzenie czy obecny koszt dojścia do wierzchołka v jest większy niż suma obecnego
                        # kosztu dojścia do wierzchołka u oraz wagi krawędzi łączacej oba te wierzchołki
                        d[v] = d[u] + wage_function[u][v] # jeśli tak, to ustawiamy nowy koszt jako tą sumę
                        p[v] = u # oraz ustawiamy nowego poprzednika na najkrótszej ścieżce
        path = list() # stworzenie listy odwiedzonych wierzchołków na najkrótszej ścieżce

        path.append(start_index) # dodanie do listy wierzchołka początkowego
        x = end_index # utworzenie pomocniczej zmiennej, która początkowo przechowuje numer końcowego wierzchołka i przyjmuje numery poprzednich wierzchołków na ścieżce
        while (x != start_index): # do momentu, gdy zmienna pomocnicza będzie równa numerowi wierzchołka startowego
            path.insert(1,x) # dodaje do ścieżki wierzchołek, zaraz za wierzchołkiem startowym
            x = p[x] # ustawiam x na poprzednika dodanego wcześniej wierzchołka

        for v in path:
            self.path.append(keys[v]) # odtwarzam ścieżke na podstawie numerów wierzchołków z algorytmu bellman-forda
        self.path = np.array(self.path)
        
    def finding_path (self):
        keys = self.defining_keys()
        graph, slant = self.creating_possible_paths(keys)
        wage_function = np.full((len(keys), len(keys)), np.inf)
        for i in range(len(keys)):
            for j in range(len(keys)):
                if j in graph[i] : # jeśli j-ty wierzchołek sąsiaduje z i-tym
                    if (i,j) in slant: # jeśli przejście od niego na skos to waga pierwiastek z dwóch
                        wage_function[i][j] = np.sqrt(2)
                    else:
                        wage_function[i][j] = 1.0
        self.bellman_ford(graph, keys, wage_function)

    def __repr__ (self):
        string ='['
        for i in range(len(self.path)):
            string += f'{self.path[i]} -> '
        string += ']'
        new_string = string.replace(' -> ]',']')
        return new_string