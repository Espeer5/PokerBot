"""
The launch file for running a card flipping demo in order to tune the card 
flipping trajectory.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from utils.constants import ACTUAL_RSP, NODE_HEBI, NODE_RVIZ, NODE_USBCAM

def generate_launch_description():
    """
    Generates a list of nodes which will be run when the launch file is
    executed. Each node is configured to run a specific executable where the 
    executables need to be listed in the package's setup.py file.
    """

    # Configure the node which publishes the torques to the robot.
    flipper = Node(
        name       = 'flipper', 
        package    = 'brain',
        executable = 'flip_tune',
        output     = 'screen')
    
    # Configure the node which publishes the torques to the robot.
    control = Node(
        name       = 'control', 
        package    = 'trajectory',
        executable = 'obey',
        output     = 'screen')

    # Prepare the list of elements to launch
    
    # Return the description, built as a python list.
    return LaunchDescription([
        NODE_USBCAM,
        # BackCardDetectorNode,
        ACTUAL_RSP,
        # NODE_RVIZ,
        NODE_HEBI,
        flipper,
        control,
    ])