from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from LidarMap import LidarMap
from interfaces.msg import CommandArray
import interfaces.msg as msg
from Path2Communicates.robot_message import RobotMessageGenerator
from RoadFinder import RoadFinder
import time


class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.laser_scan_subscriber_ = self.create_subscription(
            LaserScan, 
            'laser_scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.publisher_ = self.create_publisher(CommandArray, 'command_array', 10)
        self._message_generator = RobotMessageGenerator()

    def scan_callback(self, msg: LaserScan):
        print("scan")
        target = (5, 5)
        lidar_map = LidarMap(msg, target, size = 20, cell_size=0.5)
        print("map", lidar_map)
        t = time.time()
        road_finder = RoadFinder(lidar_map, lidar_map.position, target)
        path = road_finder.find_path()
        print("path", path)
        relative_path = [( p[1] - lidar_map.position[1], -p[0] + lidar_map.position[0]) for p in path]
        print("relative path", relative_path)
        command_array = self._message_generator.make_message_from_path(relative_path)
        print(command_array)
        print("scan")
        self.publisher_.publish(command_array)
