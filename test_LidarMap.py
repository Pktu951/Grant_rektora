import numpy as np
from LidarMap import LidarMap
from LidarScan import LidarScan

lidarScan = LidarScan(angle_min=0, angle_max=np.pi, angle_increment=np.pi/3, range_min=0.1, range_max=12, ranges=[2, 5, 12, 5])
print(lidarScan)
lidarMap = LidarMap(lidarScan)

print(lidarMap)