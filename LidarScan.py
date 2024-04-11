class LidarScan:
    def __init__(self, angle_min, angle_max, angle_increment, range_min, range_max, ranges):
        """
        Initialize the LidarScan structure.
        :param angle_min: start angle of the scan [rad]
        :param angle_max: end angle of the scan [rad]
        :param angle_increment: angular distance between measurements [rad]
        :param range_min: minimum range value [m]
        :param range_max: maximum range value [m]
        :param ranges: range data [m]
        """
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.range_min = range_min
        self.range_max = range_max
        self.ranges = ranges

    def __repr__(self):
        return (f"LidarScan(angle_min={self.angle_min}, angle_max={self.angle_max}, "
                f"angle_increment={self.angle_increment}, range_min={self.range_min}, "
                f"range_max={self.range_max}, ranges={self.ranges})")