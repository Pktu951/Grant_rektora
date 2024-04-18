from LidarMap import LidarMap
from sensor_msgs.msg import LaserScan

ourLaserScan = LaserScan()

ourLaserScan.angle_min = -90.0
ourLaserScan.angle_max = 90.0
ourLaserScan.angle_increment = 45.0
ourLaserScan.time_increment = 0.1
ourLaserScan.scan_time = 0.1
ourLaserScan.range_min = 0.5
ourLaserScan.range_max = 5.0
ourLaserScan.ranges = [3.0, 3.0, 3.0, 3.0, 3.0]

lidarMap = LidarMap(ourLaserScan)

print(lidarMap)