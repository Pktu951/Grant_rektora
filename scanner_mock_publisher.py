import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class ScannerMockPublisher(Node):
    def __init__(self, frequency:int):
        super().__init__('scanner_publisher')
        self.publisher = self.create_publisher(LaserScan, 'laser_scan', 10)
        self.frequency = frequency
        self.timer = self.create_timer(1/frequency, self.timer_callback)
    
    def timer_callback(self):
        msg = LaserScan()
        msg.header = Header() 
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'skaner_frame'
        
        msg.angle_min = float(-math.pi / 2)
        msg.angle_max = float(math.pi / 2)
        msg.angle_increment = float(math.pi / 180)/2
        
        msg.time_increment = float(1.0 / self.frequency)
        msg.scan_time = float(1.0 / self.frequency)

        msg.range_min = float(0.1)
        msg.range_max = float(10.0)
        
        msg.ranges = []
        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment)

        for i in range(num_ranges//2):
            msg.ranges.append(float(5))

        msg.intensities = []

        self.publisher.publish(msg)
