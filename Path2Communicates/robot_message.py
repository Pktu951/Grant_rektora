import math
import time 
import numpy as np
from const import DISTANCE_1_MESSAGE, ANGLE_AFTER_1_MESSAGE, DY_AFTER_1_MESSAGE, DX_AFTER_1_MESSAGE, GO_RIGHT, GO_LEFT, GO_FORWARD

class RobotMessage:
    def __init__(self):
        self.robot_coordinates = [0.0, 0.0]
        self.trace = {}
        self.robot_angle = 0.0
    
    def __angle_robot_point(self, projected_coordinates: tuple) -> float:
        projected_x = projected_coordinates[0]
        projected_y = projected_coordinates[1]

        return math.degrees(math.atan2(projected_y, projected_x))
    
    def __get_rotation_matrix(self):
        rotation_matrix = np.zeros((2, 2))
        rotation_matrix[0][0] = math.cos(self.robot_angle)
        rotation_matrix[0][1] = -math.sin(self.robot_angle)
        rotation_matrix[1][0] = math.sin(self.robot_angle)
        rotation_matrix[1][1] = math.cos(self.robot_angle)
        return rotation_matrix

    def __project_point_local_axes(self, point_coordinates):
        local_position = np.array([[point_coordinates[0] - self.robot_coordinates[0]],
                                          [point_coordinates[1] - self.robot_coordinates[1]]])

        projected_point = self.__get_rotation_matrix() @ local_position
        projected_x, projected_y = projected_point[0][0], projected_point[1][0]

        return projected_x, projected_y
    
    def coordinates_after_message(self, command):
        cos_angle = math.cos(self.robot_angle)
        sin_angle = math.sin(self.robot_angle)
        
        if command == 'forward':
            self.robot_coordinates[1] = self.robot_coordinates[1] + DISTANCE_1_MESSAGE * sin_angle
            self.robot_coordinates[0] = self.robot_coordinates[0] + DISTANCE_1_MESSAGE * cos_angle
        else:
            self.robot_coordinates[1] = self.robot_coordinates[1] + DY_AFTER_1_MESSAGE*cos_angle - DX_AFTER_1_MESSAGE*sin_angle
            self.robot_coordinates[0] = self.robot_coordinates[0] + DY_AFTER_1_MESSAGE *sin_angle - DX_AFTER_1_MESSAGE *cos_angle

            if command == 'right': self.robot_angle = self.robot_angle + ANGLE_AFTER_1_MESSAGE
            else: self.robot_angle = self.robot_angle - ANGLE_AFTER_1_MESSAGE

    def is_within_bounds(x_, y_):
        if -0.2 < x_ < 0.2 and -0.2 < y_ < 0.2: return True
        return False

    def _message_generator(self, point_coordinates):
        messages = []
        projected_x, projected_y = self.__project_point_local_axes(point_coordinates)
        angle = self.__angle_robot_point((projected_x, projected_y))

        while not self.is_within_bounds(projected_x, projected_y):
            if angle < 90 - ANGLE_AFTER_1_MESSAGE:
                messages.append(GO_RIGHT)
                self.coordinates_after_message('right')
            elif angle > 90 - ANGLE_AFTER_1_MESSAGE:
                messages.append(GO_LEFT)
                self.coordinates_after_message('left')
            else:
                messages.append(GO_FORWARD)
                self.coordinates_after_message('forward')

            projected_x, projected_y = self.__project_point_local_axes(point_coordinates)
            angle = self.__angle_robot_point((projected_x, projected_y))
            #self.trace[self.robot_coordinates[0]] = self.robot_coordinates[1]      #do wizualizacji

        return messages
    def make_message_from_path(self, path: 'list[tuple]') -> 'list[str]':
        messages = []
        for coordinates in path:
            messages.extend(self._message_generator(coordinates))
        #print(messages)                                                            #do wizualizacji
        return messages