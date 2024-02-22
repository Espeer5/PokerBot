'''
This module contains utilities for interacting with the suction vacuum pump 
via PWM. One should import this fil rather than defining this function elsewhere
so that a single Serial object is created a single time and may be used across
files.
'''

from serial import Serial

PUMP = Serial("/dev/ttyACM0", 9600)

def send_pwm(value: int, device: Serial = PUMP) -> None:
    """
    Send a pwm value across the serial port to the Arduino on the specified
    port. Defaults to /dev/ttyACM0, but can choose any port.
    """
    if value < 0 or value > 255:
        raise ValueError("PWM value must be between 0 and 255")
    device.write(f"{value}".encode("utf-8"))
