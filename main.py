def main(args=None):
    rclpy.init(args=args)

    node = LaserScanSubscriber()
    rclpy.spin(node)

    rclpy.shutdown()
