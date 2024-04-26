from rclpy.node import Node

class OutputChangeTimer(Node):
    def __init__(self, interval: float = 0.1):
        """Initialize the OutputChangeTimer and start a timer with the given interval."""
        
        super().__init__('output_change_timer_node')
        self.interval = interval
        
        self.timer = self.create_timer(interval, self.timer_callback)
        
    def timer_callback(self):
        """Timer callback function."""
        
        self.get_logger().info(f'Timer callback executing logic at {1/self.interval} Hz.')