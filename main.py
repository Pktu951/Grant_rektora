import rclpy
from laser_scan_subscriber import LaserScanSubscriber

def main(args=None):
    rclpy.init(args=args)

    node = LaserScanSubscriber()
    rclpy.spin(node)

    rclpy.shutdown()
