import rclpy
from laser_scan_subscriber import LaserScanSubscriber
from output_change_timer import OutputChangeTimer
from scanner_mock_publisher import ScannerMockPublisher

def main(args=None):
    rclpy.init(args=args)
    
    laser_scan_node = LaserScanSubscriber()
    output_change_timer = OutputChangeTimer()
    scanner_mock_pub = ScannerMockPublisher(10)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(laser_scan_node)
    executor.add_node(output_change_timer)
    executor.add_node(scanner_mock_pub)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("KeyboardInterrupt caught, shutting down...")
    except Exception as e:
        print(f"An exception occurred: {e}")
    finally:
        executor.shutdown()
        laser_scan_node.destroy_node()
        output_change_timer.destroy_node()
        scanner_mock_pub.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
