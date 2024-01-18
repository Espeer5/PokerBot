"""Point the 3DOF arm for the first demo in ME/CS 134

"""

from launch                            import LaunchDescription
from launch_ros.actions                import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory as pkgdir
from launch.actions                    import Shutdown


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
        package    = 'point',
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
        # Start the hebi and demo nodes.
        node_robot_state_publisher_ACTUAL,
        #node_robot_state_publisher_COMMAND,
        node_rviz,
        node_hebi,
        node_demo,
    ])
