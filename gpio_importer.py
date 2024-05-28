from os import getenv
from sys import exit

def import_gpio():
    """
    Attempt to import the RPi.GPIO library.
    Falls back to a simulated GPIO library if the real one is not available.
    Exits the application if neither real nor simulated GPIO libraries are available.
    """
    try:
        import gpiod as GPIO
        print("Using real RPi.GPIO library.")
    except ImportError:
        try:
            import SimulRPi.GPIO as GPIO
            print("RPi.GPIO not found, using SimulRPi.GPIO library.")
        except ImportError:
            print("RPi.GPIO and SimulRPi.GPIO libraries not found. Exiting.")
            exit(1)
    return GPIO

# Check if the USE_SIMULATED_GPIO environment variable is set to 1
if getenv('USE_SIMULATED_GPIO', '0') == '1':
    import SimulRPi.GPIO as GPIO
else:
    GPIO = import_gpio()