import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import LaserScan, Header  
from rclpy.qos import qos_profile_sensor_data
import numpy as np

class ScannerPublisher(Node):
    def __init__(self, frequency:int):
        super().__init__('scanner_publisher')
        self.publisher = self.create_publisher(LaserScan, 'scanner_topic', 10)  
        self.frequency = frequency
        self.timer = self.create_timer(1/frequency, self.timer_callback)
    
    def timer_callback(self):
        msg = LaserScan()
        msg.header = Header() 
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'skaner_frame'
        
        msg.angle_min = np.float32(-math.pi / 2)
        msg.angle_max = np.float32(math.pi / 2)
        msg.angle_increment = np.float32(math.pi / 180)
        
        msg.time_increment = np.float32(1.0 / self.frequency)
        msg.scan_time = np.float32(1.0 / self.frequency) 

        msg.range_min = np.float32(0.1)
        msg.range_max = np.float32(10.0)
        
        msg.ranges = []
        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment)

        for i in range(num_ranges):
            current_angle = msg.angle_min + i * msg.angle_increment
            msg.ranges.append(np.float32(current_angle))

        msg.intensities = np.array([], dtype="float32") 

        self.publisher.publish(msg)