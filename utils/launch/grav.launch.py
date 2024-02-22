"""
Launch file for a gravity compensation tuning demo for the poker bot. Sends only
torque commands with no position or velocity controls to the robot. If the
gravity model constants are tuned correctly, the robot should hold the position
it is posed in by hand for all positions, and neither sag nor rise.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from utils.constants import ACTUAL_RSP, NODE_HEBI, NODE_RVIZ


def generate_launch_description():
    """
    Generates a list of nodes which will be run when the launch file is
    executed. Each node is configured to run a specific executable where the 
    executables need to be listed in the package's setup.py file.
    """

    # Configure the node which publishes the torques to the robot.
    grav_model = Node(
        name       = 'grav_model', 
        package    = 'utils',
        executable = 'grav_tune',
        output     = 'screen')
    
    # Prepare the list of elements to launch
    
    # Return the description, built as a python list.
    return LaunchDescription([
        ACTUAL_RSP,
        NODE_HEBI,
        grav_model,
    ])
