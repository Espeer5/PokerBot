"""Point the 3DOF arm for the first demo in ME/CS 134

"""

from launch                            import LaunchDescription
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the hebi interface.
    node_hebi = Node(
        name       = 'hebi', 
        package    = 'hebiros',
        executable = 'hebinode',
        output     = 'screen',
        parameters = [{'family': 'robotlab'},
                      {'motors': ['8.3', '8.5', '8.4']},
                      {'joints': ['one', 'two', 'three']}])

    # Configure a node for the simple demo.
    node_demo = Node(
        name       = 'demo', 
        package    = 'point',
        executable = 'point',
        output     = 'screen')


    ######################################################################
    # COMBINE THE ELEMENTS INTO ONE LIST
    
    # Return the description, built as a python list.
    return LaunchDescription([
        # Start the hebi and demo nodes.
        node_hebi,
        node_demo,
    ])
