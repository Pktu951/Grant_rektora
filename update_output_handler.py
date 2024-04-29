try:
    import RPi.GPIO as GPIO
except ImportError:
    print("RPi.GPIO not found, using simulated version.")
    try:
        import SimulRPi.GPIO as GPIO
    except ImportError:
        print("SimulRPi not found, exiting.")
        exit(1)
from interfaces.msg import CommandArray

class UpdateOutputHandler:
    def __init__(self):
        self.pins = [CommandArray.GO_FORWARD, CommandArray.GO_LEFT, CommandArray.GO_RIGHT, CommandArray.GO_BACKWARD]
        GPIO.setmode(GPIO.BOARD)
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
        
        GPIO.output(self.pins, [GPIO.LOW] * len(self.pins))
        
    def update_output(self, command: int) -> None:
        """Update the output based on the given command. If the command is not in the list of pins, set all pins to LOW."""
        pin_states = {pin: GPIO.LOW for pin in self.pins}
        
        if command in self.pins:
            pin_states[command] = GPIO.HIGH
        elif command != CommandArray.STOP:
            print(f"Invalid command: {command}")
            
        GPIO.output(list(pin_states.keys()), list(pin_states.values()))
        
    def __del__(self):
        GPIO.cleanup()