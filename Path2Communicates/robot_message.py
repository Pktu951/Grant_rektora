import math
import numpy as np
from typing import List, Tuple
from const import DISTANCE_1_MESSAGE, ANGLE_AFTER_1_MESSAGE, DY_AFTER_1_MESSAGE, DX_AFTER_1_MESSAGE
from interfaces.msg import CommandArray

class RobotMessage:
    def __init__(self):
        self.robot_coordinates = [0.0, 0.0]
        self.trace = {}
        self.robot_angle = 0.0
    
    def _angle_robot_point(self, projected_coordinates: tuple[float, float]) -> float:
        projected_x = projected_coordinates[0]
        projected_y = projected_coordinates[1]

        return math.degrees(math.atan2(projected_y, projected_x))
    
    def _get_rotation_matrix(self) -> float[2][2]:
        rotation_matrix = np.zeros((2, 2))
        rotation_matrix[0][0] = math.cos(self.robot_angle)
        rotation_matrix[0][1] = -math.sin(self.robot_angle)
        rotation_matrix[1][0] = math.sin(self.robot_angle)
        rotation_matrix[1][1] = math.cos(self.robot_angle)
        return rotation_matrix

    def _project_point_local_axes(self, point_coordinates : Tuple[float, float]) -> Tuple:
        local_position = np.array([[point_coordinates[0] - self.robot_coordinates[0]],
                                          [point_coordinates[1] - self.robot_coordinates[1]]])

        local_position = self.__get_rotation_matrix() @ local_position
        local_x, local_y = local_position[0][0], local_position[1][0]

        return local_x, local_y
    
    def _recalculate_coordinates_for_message(self, message : str):
        cos_angle = math.cos(self.robot_angle)
        sin_angle = math.sin(self.robot_angle)
        
        if message == 'forward':
            self.robot_coordinates[1] = self.robot_coordinates[1] + DISTANCE_1_MESSAGE * sin_angle
            self.robot_coordinates[0] = self.robot_coordinates[0] + DISTANCE_1_MESSAGE * cos_angle
        else:
            self.robot_coordinates[1] = self.robot_coordinates[1] + DY_AFTER_1_MESSAGE*cos_angle - DX_AFTER_1_MESSAGE*sin_angle
            self.robot_coordinates[0] = self.robot_coordinates[0] + DY_AFTER_1_MESSAGE *sin_angle - DX_AFTER_1_MESSAGE *cos_angle

            if message == 'right': self.robot_angle = self.robot_angle + ANGLE_AFTER_1_MESSAGE
            else: self.robot_angle = self.robot_angle - ANGLE_AFTER_1_MESSAGE

    def _is_within_bounds(x_ : float, y_ : float) -> bool:
        if -0.2 < x_ < 0.2 and -0.2 < y_ < 0.2: return True
        return False

    def _message_generator(self, point_coordinates : Tuple[float, float]) -> List[str]:
        messages = []
        projected_x, projected_y = self.__project_point_local_axes(point_coordinates)
        angle = self.__angle_robot_point((projected_x, projected_y))

        while not self.is_within_bounds(projected_x, projected_y):
            if angle < 90 - ANGLE_AFTER_1_MESSAGE:
                messages.append(CommandArray.GO_RIGHT)
                self.coordinates_after_message('right')
            elif angle > 90 + ANGLE_AFTER_1_MESSAGE:
                messages.append(CommandArray.GO_LEFT)
                self.coordinates_after_message('left')
            else:
                messages.append(CommandArray.GO_FORWARD)
                self.coordinates_after_message('forward')

            projected_x, projected_y = self.__project_point_local_axes(point_coordinates)
            angle = self.__angle_robot_point((projected_x, projected_y))
            #self.trace[self.robot_coordinates[0]] = self.robot_coordinates[1]      #do wizualizacji

        return messages
    
    def make_message_from_path(self, path: List[Tuple[float, float]]) -> CommandArray:
        commands = []
        for coordinates in path:
            commands.extend(self._message_generator(coordinates))
        #print(messages)                                                            #do wizualizacji
        command_array = CommandArray()
        command_array.commands = commands
        return command_array
    