"""
The main launch file for the PokerBot. This file launches all the nodes required
to run the PokerBot, being primarily the brain and the control node which 
collaborate to plan and execute the robot's actions.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from utils.constants import ACTUAL_RSP, NODE_HEBI, NODE_RVIZ, NODE_USBCAM, NODE_BOXCAM

def generate_launch_description():
    """
    Generates a list of nodes which will be run when the launch file is
    executed. Each node is configured to run a specific executable where the 
    executables need to be listed in the package's setup.py file.
    """

    # Configure the node which publishes the torques to the robot.
    brain = Node(
        name       = 'brain', 
        package    = 'brain',
        # executable = 'collect', # Or 'brain'
        executable = 'brain',
        output     = 'screen')

    # Configure the node which publishes the torques to the robot.
    control = Node(
        name       = 'control', 
        package    = 'trajectory',
        executable = 'obey',
        output     = 'screen')
    
    ChipDetectorNode = Node(
        name       = 'ChipDetector', 
        package    = 'detectors',
        executable = 'ChipDetector',
        output     = 'screen',
        remappings = [('/image_raw', '/usb_cam/image_raw')]
    )

    ButtonDetectorNode = Node(
        name       = 'ButtonDetector', 
        package    = 'detectors',
        executable = 'ButtonDetector',
        output     = 'screen',
        remappings = [('/image_raw', '/usb_cam/image_raw')]
    )
    
    # Configure the BackCardDetector
    BackCardDetectorNode = Node(
        name       = 'BackCardDetector', 
        package    = 'detectors',
        executable = 'BackCardDetector',
        output     = 'screen',
        remappings = [('/image_raw', '/usb_cam/image_raw')])
    
    CardDetectorNode = Node(
        name       = 'CardDetector', 
        package    = 'detectors',
        executable = 'CardDetector',
        output     = 'screen',
        remappings = [('/image_raw', '/usb_cam/image_raw')])

    # Prepare the list of elements to launch
    
    # Return the description, built as a python list.
    return LaunchDescription([
        NODE_USBCAM,
        BackCardDetectorNode,
        ChipDetectorNode,
        CardDetectorNode,
        ButtonDetectorNode,
        NODE_BOXCAM,
        ACTUAL_RSP,
        # NODE_RVIZ,
        NODE_HEBI,
        brain,
        control,
    ])