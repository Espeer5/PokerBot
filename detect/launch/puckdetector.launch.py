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

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('basic134'), 'rviz/viewurdf.rviz')

    # Locate/load the robot's URDF file (XML).
    urdf = os.path.join(pkgdir('basic134'), 'urdf/threedofexample.urdf')
    with open(urdf, 'r') as file:
        robot_description = file.read()

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure the USB camera node
    node_usbcam = Node(
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

    # Configure the ball detector node
    node_puckdetector = Node(
        name       = 'puckdetector', 
        package    = 'detect',
        executable = 'puckdetector',
        output     = 'screen',
        remappings = [('/image_raw', '/usb_cam/image_raw')])
    
    # Configure the arm controller node
    node_demo = Node(
        name       = 'demo',
        package    = 'detect',
        executable = 'point',
        output     = 'screen',
    )

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher_ACTUAL = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())

    # Configure a node for the hebi interface.
    node_hebi = Node(
        name       = 'hebi', 
        package    = 'hebiros',
        executable = 'hebinode',
        output     = 'screen',
        parameters = [{'family': 'robotlab'},
                      {'motors': ['8.3', '8.5', '8.4']},
                      {'joints': ['base', 'shoulder', 'elbow']}])

    # Configure a node for the simple demo.
    node_demo = Node(
        name       = 'demo', 
        package    = 'detect',
        executable = 'point',
        output     = 'screen')


    node_robot_state_publisher_COMMAND = Node(
        name       = 'robot_state_publisher', 
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}],
        remappings = [('/joint_states', '/joint_commands')])


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([
        # Start the nodes.
        node_usbcam,
        node_robot_state_publisher_ACTUAL,
        #node_robot_state_publisher_COMMAND,
        #node_rviz,
        node_hebi,
        node_demo,
        node_puckdetector,
    ])
