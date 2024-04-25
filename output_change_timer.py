from rclpy.node import Node

class OutputChangeTimer(Node):
    def __init__(self, interval=0.1):
        super().__init__('output_change_timer_node')
        self.timer = self.create_timer(interval, self.timer_callback)
        self.interval = interval

    def timer_callback(self):
        self.get_logger().info(f'Timer callback executing logic at {1/self.interval} Hz')