import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import LaserScan, Header  
from rclpy.qos import qos_profile_sensor_data

class ScannerPublisher(Node):
    def __init__(self, frequency):
        super().__init__('scanner_publisher')
        self.publisher = self.create_publisher(LaserScan, 'scanner_topic', 10)  
        self.frequency = frequency
        self.timer = self.create_timer(1/frequency, self.timer_callback)
    
    def timer_callback(self):
        msg = LaserScan()
        msg.header = Header() 
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'skaner_frame'
        
        msg.angle_min = -math.pi / 2
        msg.angle_max = math.pi / 2
        msg.angle_increment = math.pi / 180
        
        msg.time_increment = 1.0 / self.frequency
        msg.scan_time = 1.0 / self.frequency  

        msg.range_min = 0.1
        msg.range_max = 10.0
        
        msg.ranges = [1.0] * int((msg.angle_max - msg.angle_min) / msg.angle_increment)  
        msg.intensities = [0.0] * int((msg.angle_max - msg.angle_min) / msg.angle_increment)  

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    skanner = ScannerPublisher(10)

    rclpy.spin(skanner)
    skanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
