from rclpy.node import Node
from interfaces.msg import CommandArray
from collections import deque
from threading import Lock
from update_output_handler import UpdateOutputHandler

class OutputChangeTimer(Node):
    def __init__(self, interval: float = 0.1):
        """Initialize the OutputChangeTimer, start a timer with the given interval
        and create a subscription to the CommandArray topic."""
        
        super().__init__('output_change_timer_node')
        
        self._timer = self.create_timer(interval, self._timer_callback)
        
        self._commands = deque()
        self._lock = Lock()
        self._interval = interval
        
        self._subscription = self.create_subscription(
            CommandArray,
            'command_array',
            self._listener_callback,
            10)
        
        self.update_output_handler = UpdateOutputHandler()
        
    def _listener_callback(self, msg: CommandArray) -> None:
        """Callback function for the CommandArray subscription. Save the commands"""
        
        with self._lock:
            self._commands = deque(msg.commands)
            self.get_logger().info(f'Listener commands: {self._commands}')
        
    def _timer_callback(self) -> None:
        """Timer callback function."""
        
        with self._lock:
            self.get_logger().info(f'Timer tick, commands: {self._commands}')
            command = self._commands.popleft() if self._commands else CommandArray.STOP
            self.update_output_handler.update_output(command)