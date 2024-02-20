"""Launch the USB camera node and puck detector.

This launch file is intended show how the pieces come together.
Please copy the relevant pieces.

"""

from launch                            import LaunchDescription
from launch_ros.actions                import Node
import os
import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import Shutdown
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():
    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure the USB camera node
    USBCamNode = Node(
        name       = 'usb_cam', 
        package    = 'usb_cam',
        executable = 'usb_cam_node_exe',
        namespace  = 'usb_cam',
        output     = 'screen',
        parameters = [{'camera_name':         'logitech'},
                      {'video_device':        '/dev/video0'},
                      {'pixel_format':        'yuyv2rgb'},
                      {'image_width':         640},
                      {'image_height':        480},
                      {'framerate':           15.0},
                      {'brightness':          -1},
                      {'contrast':            -1},
                      {'saturation':          -1},
                      {'sharpness':           -1},
                      {'gain':                30},
                      {'auto_white_balance':  False},
                      {'white_balance':       3200},
                      {'autoexposure':        False},
                      {'exposure':            250},
                      {'autofocus':           True},
                      {'focus':               -1}])

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

    # ChipDetectorNode = Node(
    #     name       = 'ChipDetector', 
    #     package    = 'detectors',
    #     executable = 'ChipDetector',
    #     output     = 'screen',
    #     remappings = [('/image_raw', '/usb_cam/image_raw')])
 

    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([
        # Start the nodes.
        USBCamNode,
        BackCardDetectorNode,
        CardDetectorNode,
        # ChipDetectorNode
    ])
