import numpy as np

class LidarMap:
    def __init__(self, lidar_scan, size=11, resolution=1):
        """
        Initialize the map using a Lidar scan.
        :param lidar_scan: An object with properties of the Lidar scan.
        :param size: The width and height of the square map in cells.
        :param resolution: The size of each cell in meters.
        """
        self.size = size  # size of the map in cells
        self.resolution = resolution  # resolution of a cell in meters
        self.map = np.zeros((size, size), dtype=bool)  # initialize the map with boolean values
        self.position = (size // 2, size-1)  # point that the lidar is taking scan from
        self.process_scan(lidar_scan)

    def process_scan(self, lidar_scan):
        """
        Process the Lidar scan and update the map accordingly.
        :param lidar_scan: An object with properties of the Lidar scan.
        """
        angle = lidar_scan.angle_min
        angles = np.arange(lidar_scan.angle_min, lidar_scan.angle_max + lidar_scan.angle_increment, lidar_scan.angle_increment)    # start, stop, step 
            
        for distance, angle in zip(lidar_scan.ranges, angles):
            if not (lidar_scan.range_min <= distance <= lidar_scan.range_max):
                print(f"Invalid distance value: {distance} at angle {angle}")
                continue    
            # Convert polar coordinates (angle, distance) to Cartesian coordinates (x, y)
            x = int((distance * np.cos(angle)) / self.resolution)
            y = int((distance * np.sin(angle)) / self.resolution)

            # Calculate map coordinates and check if they are within bounds
            map_x = self.position[0] + x
            map_y = self.position[1] - y  # y is inverted because map coordinates are top-down, while Cartesian coordinates are bottom-up
            coordinatesWithinBounds  = 0 <= map_x < self.size and 0 <= map_y < self.size
            if coordinatesWithinBounds:
                self.map[map_y, map_x] = True

            angle += lidar_scan.angle_increment
            
    def __repr__(self):
        map_str = ""
        for i, row in enumerate(self.map):
            if i == self.position[1]:
                map_str += " ".join(["." if j == self.position[0] else "█" if cell else " " for j, cell in enumerate(row)]) + "\n"
            else:
                map_str += " ".join(["█" if cell else " " for cell in row]) + "\n"
            
        return map_str