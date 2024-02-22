"""
Launch file for a basic trajectory test for the poker bot. This launch file
runs a test node which uses the trajectory class to dictate the joint positions
and velocities to the poker bot.
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
    traj_model = Node(
        name       = 'traj_model', 
        package    = 'utils',
        executable = 'traj_test',
        output     = 'screen')
    
    # Prepare the list of elements to launch
    
    # Return the description, built as a python list.
    return LaunchDescription([
        ACTUAL_RSP,
        # NODE_RVIZ,
        NODE_HEBI,
        traj_model,
    ])