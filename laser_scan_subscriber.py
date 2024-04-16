import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        self.laser_scan_subscriber_ = self.create_subscription(
            LaserScan, 
            'laser_scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
    
    def scan_callback(self, msg: LaserScan):
        header = msg.header

        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        time_increment = msg.time_increment

        scan_time = msg.scan_time

        range_max = msg.range_max
        range_min = msg.range_min

        self.get_logger().info(f'Header stamp: {header.stamp}\n'
                               f'Angle Min: {angle_min}\n'
                               f'Angle Max: {angle_max}\n'
                               f'Angle Increment: {angle_increment}\n'
                               f'Time Increment: {time_increment}\n'
                               f'Scan Time: {scan_time}\n'
                               f'Range Max: {range_max}\n'
                               f'Range Min: {range_min}\n')

        self.get_logger.info('Ranges:')
        for i, range_val in enumerate(msg.ranges):
            self.get_logger().info(f'   Range {i}: {range_val}')

        self.get_logger.info('Intesities:')
        for i, intensity_val in enumerate(msg.intensities):
            self.get_logger().info(f' Intensity {i}: {intensity_val}')
