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
        lidar_map = LidarMap(msg)
        print("map", lidar_map)
        t = time.time()
        road_finder = RoadFinder(lidar_map, (10, 5), (8, 5))
        path = road_finder.find_path()
        print("path", path)
        command_array = self._message_generator.make_message_from_path(path)
        print(command_array)
        print("scan")
        self.publisher_.publish(command_array)
