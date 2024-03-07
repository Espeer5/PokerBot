"""
A simple launch file used for testing the raw outputs of the camera nodes.
"""

from launch import LaunchDescription
from utils.constants import NODE_USBCAM, NODE_BOXCAM

def generate_launch_description():
    """
    Publish the nodes for the camera outputs
    """
    return LaunchDescription([NODE_BOXCAM])