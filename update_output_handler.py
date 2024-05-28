import gpiod
from interfaces.msg import CommandArray

class UpdateOutputHandler:
    def __init__(self):
        self._chip = gpiod.Chip('gpiochip4')
        self._pins = [CommandArray.GO_FORWARD, CommandArray.GO_LEFT, CommandArray.GO_RIGHT, CommandArray.GO_BACKWARD]
        self._pins = [self._chip.get_line(pin) for pin in self._pins]

        for pin in self._pins:
            pin.request(consumer="output", type=gpiod.LINE_REQ_DIR_OUT)
            pin.set_value(0)


    def update_output(self, command: int) -> None:
        """Update the output based on the given command. If the command is not in the list of pins, set all pins to LOW."""

        for pin in self._pins:
            if command == pin.offset():
                pin.set_value(1)
            else:
                pin.set_value(0)

    def __del__(self):
        for pin in self._pins:
            pin.release()