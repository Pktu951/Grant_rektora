import numpy as np
import math

class RobotMessage:
    def __init__(self, turn_radius, velocity, message_frequency):
        self._turn_radius = turn_radius 
        self._velocity = velocity
        self._message_frequency = message_frequency
        self.start_robot_coordinates = (0.0, 0.0)
        self._robot_angle = 0.0
        self.delta_angle_forward = 4

        # Homogenous matrix local axis
        self._uniform_matrix = np.zeros((4, 4))
        self._uniform_matrix[3, 3] = 1

        # function to return translation vector
        self._translation_vector = lambda: self._uniform_matrix[:3, 3].reshape(-1, 1) \
            if len(self._uniform_matrix[:3, 3].shape) == 1 else self._uniform_matrix[:3, 3]

        # function to return rotation matrix
        self._rotation_matrix = lambda: self._uniform_matrix[:3, :3]

        # euclidean distance
        self._euclidean_distance = lambda x, y, x_local_point, y_local_point: math.sqrt(
            pow(x - x_local_point, 2) + pow(y - y_local_point, 2)
        )

        self._update_uniform_matrix(0, 0)

        # Calculation of the angle by which the robot will rotate after turning
        circumference = 2 * math.pi * self._turn_radius
        time_for_1_circumference = circumference / self._velocity
        number_of_messages_1_circumference = math.ceil(time_for_1_circumference / self._message_frequency)
        self._angle_after_1_message = 360 / number_of_messages_1_circumference

        # Calculation of dx and dy by which the robot will move in relation of the local axis
        # after turning
        self.dx_after_1_message = self._turn_radius * 2 * math.sin(
            math.radians(self._angle_after_1_message / 2)) * math.cos(
            math.radians((180 - self._angle_after_1_message) / 2))
        self.dy_after_1_message = self._turn_radius * 2 * math.sin(
            math.radians(self._angle_after_1_message / 2)) * math.sin(
            math.radians((180 - self._angle_after_1_message) / 2))

        # Forward distance after 1 message
        self.distance_1_message = self._velocity * self._message_frequency

        self.messages = []

    def _update_uniform_matrix(self, dx, dy, angle=0.0):
        """
        Update homogenous matrix (local axis)
        :param dx: difference between old x_local_point and new x_local_point
        :param dy: difference between old x_local_point and new x_local_point
        :param angle: the angle by which the robot rotated relative to the previous
        :return:
        """

        # Turn the robot by given angle in relation of the z axis
        self._robot_angle += angle

        # Calculation rotate matrix relative to the z axis
        z_rotation_matrix = np.array([
            [math.cos(math.radians(self._robot_angle)), -math.sin(math.radians(self._robot_angle)), 0],
            [math.sin(math.radians(self._robot_angle)), math.cos(math.radians(self._robot_angle)), 0],
            [0, 0, 1]
        ])

        # Project local point to global point
        dx_local = dx
        dy_local = dy
        from_local_to_global = self._projecting_point(dx_local, dy_local, to_local=False)
        new_x_global = from_local_to_global[0]
        new_y_global = from_local_to_global[1]

        # Move translation matrix by the drove distance by robot relative to global axes new_x_global, new_y_global
        new_translation_vector = np.array([new_x_global, new_y_global, [0]])

        # Update uniform matrix
        self._uniform_matrix = np.concatenate(
            (np.concatenate((z_rotation_matrix, new_translation_vector), axis=1),
             np.array([[0, 0, 0, 1]]))
        )

    def _projecting_point(self, x_point, y_point, to_local=True):
        """
        Project global vector to local vector
        :param x_point: global x point
        :param y_point: global y point
        :param to_local: (default True) if True -> global to local axis if False -> local to global
        :return: if to_local=True -> return coordinates of local point /
        if to_local=False -> return coordinates of global point
        """

        # Create point vector
        point_vector = np.array([[x_point], [y_point], [0], [1]])

        # If the point should be project to local axis
        if to_local:
            # Load rotate matrix from actual uniform matrix
            rotation_matrix = self._rotation_matrix()

            # Load translation vector from actual uniform matrix
            translation_vector = self._translation_vector()

            # Calculation of inverse rotation matrix
            inv_rotation_matrix = np.linalg.inv(rotation_matrix)

            # Create inverse uniform matrix which is need to project point to local axis
            inv_uniform_matrix = np.concatenate(
                (np.concatenate((inv_rotation_matrix, -(np.dot(inv_rotation_matrix, translation_vector))), axis=1),
                 np.array([[0, 0, 0, 1]])),
                axis=0
            )

            # Transform the point to local axis
            local_point = np.dot(inv_uniform_matrix, point_vector)

            return local_point[:3]

        # Transform the point to global axis
        global_point = np.dot(self._uniform_matrix, point_vector)

        return global_point

    def _is_point_inside_sphere(self, x_local_point, y_local_point):
        """
        Check if the goal point is in the circle area
        :param x_local_point: x coordinate on the local axis
        :param y_local_point:
        :return: True or False
        """
        return self._turn_radius > self._euclidean_distance(self._turn_radius, 0, x_local_point, y_local_point)

    def _angle_robot_point(self, x_local_point, y_local_point):
        """
        Count the angle between direction of the robot and the point.
        The point must be relative to the local axis
        :param x_local_point:
        :param y_local_point:
        :return: angle
        """
        angle = math.degrees(math.atan2(y_local_point, x_local_point))
        return angle - 90

    def make_message_from_path(self, path: list[tuple]):
        """
        From a list of tuples(x, y) that contain coordinates vertices, creates messages based on target points which
        enable the robot to move
        :param path: vertices of coordinates
        :return:
        """

        # global x and y coordinates from every goal point
        for x, y in path:
            # Project global x and y to local axis
            local_point = self._projecting_point(x, y)

            # if the point is inside the circle raise ValueError
            if not self._is_point_inside_sphere(x_local_point=local_point[0], y_local_point=local_point[1]):
                # while the distance between robot and goal point is greater than 0.1m execute the loop
                while self._euclidean_distance(0, 0, local_point[0], local_point[1]) > 0.1:
                    # Calculate the angle between robot and point in the local axis
                    angle_robot_point = self._angle_robot_point(local_point[0], local_point[1])

                    # if the robot is across the point (-angle error < angle robot point < angle error) go forward
                    if -self.delta_angle_forward < angle_robot_point < self.delta_angle_forward:
                        # Update uniform matrix with new x, y
                        self._update_uniform_matrix(0, self.distance_1_message)
                        self.messages.append('forward')

                    # if the point is on the right side of the robot then turn right
                    elif local_point[0] > 0:
                        # Update uniform matrix with new x, y and negative delta angle
                        self._update_uniform_matrix(self.dx_after_1_message,
                                                    self.dy_after_1_message,
                                                    angle=-self._angle_after_1_message)
                        self.messages.append('right')

                    # if the point is on the left side of the robot then turn left
                    else:
                        # Update uniform matrix with new x, y and positive delta angle
                        self._update_uniform_matrix(-self.dx_after_1_message,
                                                    self.dy_after_1_message,
                                                    angle=self._angle_after_1_message)
                        self.messages.append('left')

                    # Update goal point to local axis
                    local_point = self._projecting_point(x, y)

            else:
                raise ValueError('Incorrect goal point')
