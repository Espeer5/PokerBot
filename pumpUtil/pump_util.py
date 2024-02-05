'''
This module contains utilities for interacting with the suction vacuum pump 
via PWM.
'''

from serial import Serial

PUMP = Serial("/dev/ttyACM0", 9600)

def send_pwm(value: int, device: Serial = PUMP) -> None:
    """
    Send a pwm value across the serial port to the Arduino on the specified
    port. Defaults to /dev/ttyACM0, but can choose any port. (IF WE MOVE THE 
    PUMP USB, CHANGE PORT)
    """
    if value < 0 or value > 255:
        raise ValueError("PWM value must be between 0 and 255")
    device.write(f"{value}".encode("utf-8"))
