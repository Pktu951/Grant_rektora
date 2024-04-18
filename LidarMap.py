import numpy as np
from sensor_msgs.msg import LaserScan

class LidarMap:
    def __init__(self, laser_scan: LaserScan, size : int = 11, resolution : float = 1):
        """
        Initialize the map using a ROS 2 LaserScan scan.
        :param lidar_scan: A LaserScan message object.
        :param size: The width and height of the square map in cells.
        :param resolution: The size of each cell in meters.
        """
        self.size = size
        self.resolution = resolution
        self.map = np.zeros((size, size), dtype=bool)  # initialize the map with boolean values
        self.position = (size-1, size // 2)  # point that the lidar is taking scan from
        
        self.process_scan(laser_scan)

    def process_scan(self, laser_scan: LaserScan) -> None:
        """
        Process the Laser scan and update the map accordingly.
        :param lidar_scan: A LaserScan message object.
        """
        angles = np.arange(laser_scan.angle_min, laser_scan.angle_max + laser_scan.angle_increment, laser_scan.angle_increment)
        angles = np.radians(angles) # convert degrees to radians
        
        for distance, angle in zip(laser_scan.ranges, angles):
            if not (laser_scan.range_min <= distance <= laser_scan.range_max):
                print(f"Invalid distance value: {distance} at angle {angle}")
                continue
            
            map_x, map_y = self._calculate_map_position(distance, angle)
            self._set_map_value(map_x, map_y)
            
    def _calculate_map_position(self, distance : float, angle : float) -> tuple:
        x = int(np.rint((distance * np.cos(angle)) / self.resolution))
        y = int(np.rint((distance * np.sin(angle)) / self.resolution))

        map_x = self.position[0] - x
        map_y = self.position[1] + y
        return map_x, map_y
            
    def _set_map_value(self, map_x : int, map_y : int) -> None:
        if 0 <= map_x < self.size and 0 <= map_y < self.size:
            self.map[map_x, map_y] = True

    def __repr__(self):
        map_str = ""
        for i, row in enumerate(self.map):
            if i == self.position[0]:
                map_str += " ".join(["." if j == self.position[1] else "█" if cell else " " for j, cell in enumerate(row)]) + "\n"
            else:
                map_str += " ".join(["█" if cell else " " for cell in row]) + "\n"
        return map_str