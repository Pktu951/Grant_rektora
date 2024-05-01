import math
import time

import const
import numpy as np

class RobotMessage:
    def __init__(self):
        # const
        self.turn_radius = const.TURN_RADIUS
        self.velocity = const.VELOCITY
        self.message_frequency = const.MESSAGE_FREQUENCY
        self.distance_1_message = const.DISTANCE_1_MESSAGE
        self.robot_coordinates = (0.0, 0.0)
        self.trace = {}

        self.robot_info = {
            'x_coor': self.robot_coordinates[0],
            'y_coor': self.robot_coordinates[1],
            'angle': 0.0,
        }

        self.messages = []

        self.circle_trajectory = {'x': 0.0,
                                  'y': 0.0,
                                  'angle_curve': 0.0}


    def angle_robot_point(self, projected_coordinates: tuple) -> float:
        projected_x = projected_coordinates[0]
        projected_y = projected_coordinates[1]

        return math.degrees(math.atan2(projected_y, projected_x))

    def project_point_local_axes(self, x_point: float, y_point: float, x_robot: float, y_robot: float, angle_robot: float):
        roll = np.zeros((2, 2))

        position_of_local_axes = np.array([[x_point - x_robot],
                                          [y_point - y_robot]])

        roll[0][0] = math.cos(math.radians(angle_robot))
        roll[0][1] = -math.sin(math.radians(angle_robot))
        roll[1][0] = math.sin(math.radians(angle_robot))
        roll[1][1] = math.cos(math.radians(angle_robot))

        projected_point = np.dot(roll, position_of_local_axes)

        projected_x, projected_y = projected_point[0][0], projected_point[1][0]

        return projected_x, projected_y

    def _message_generator(self, point_coordinates):
        messages = []
        projected_x, projected_y = self.project_point_local_axes(point_coordinates[0],
                                                                 point_coordinates[1],
                                                                 self.robot_info['x_coor'],
                                                                 self.robot_info['y_coor'],
                                                                 self.robot_info['angle'])
        angle = self.angle_robot_point((projected_x, projected_y))

        while not(-0.2 < projected_x < 0.2 and -0.2 < projected_y < 0.2):
            if angle < 89:
                messages.append('right')
                self.robot_info['angle'] = self.robot_info['angle'] + const.ANGLE_AFTER_1_MESSAGE
                self.robot_info['y_coor'] = self.robot_info['y_coor'] + const.DY_AFTER_1_MESSAGE*math.cos(math.radians(self.robot_info['angle'])) - const.DX_AFTER_1_MESSAGE*math.sin(math.radians(self.robot_info['angle']))
                self.robot_info['x_coor'] = self.robot_info['x_coor'] + const.DY_AFTER_1_MESSAGE * math.sin(math.radians(self.robot_info['angle'])) - const.DX_AFTER_1_MESSAGE * math.cos(math.radians(self.robot_info['angle']))

            elif angle > 91:
                messages.append('left')
                self.robot_info['angle'] = self.robot_info['angle'] - const.ANGLE_AFTER_1_MESSAGE
                self.robot_info['y_coor'] = self.robot_info['y_coor'] + const.DY_AFTER_1_MESSAGE*math.cos(math.radians(self.robot_info['angle'])) - const.DX_AFTER_1_MESSAGE*math.sin(math.radians(self.robot_info['angle']))
                self.robot_info['x_coor'] = self.robot_info['x_coor'] + const.DY_AFTER_1_MESSAGE * math.sin(math.radians(self.robot_info['angle'])) - const.DX_AFTER_1_MESSAGE * math.cos(math.radians(self.robot_info['angle']))

            else:
                messages.append('forward')
                self.robot_info['y_coor'] = self.robot_info['y_coor'] + const.DISTANCE_1_MESSAGE * math.sin(math.radians(self.robot_info['angle']))
                self.robot_info['x_coor'] = self.robot_info['x_coor'] + const.DISTANCE_1_MESSAGE * math.cos(math.radians(self.robot_info['angle']))

            projected_x, projected_y = self.project_point_local_axes(point_coordinates[0],
                                                                     point_coordinates[1],
                                                                     self.robot_info['x_coor'],
                                                                     self.robot_info['y_coor'],
                                                                     self.robot_info['angle'])
            angle = self.angle_robot_point((projected_x, projected_y))
            self.trace[self.robot_info['x_coor']] = self.robot_info['y_coor']

        return messages
    def make_message_from_path(self, path: list[tuple]) -> list[str]:
        for coordinates in path:
            self.messages.extend(self._message_generator(coordinates))

        return self.messages

