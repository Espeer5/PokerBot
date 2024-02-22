"""
Contains the code for the Brain node of the Poker Robot. The Brain node is 
responsible for planning the robot's actions and sending the appropriate goal
commands to the Control node to execute the plan. Commands sent from the brain 
node must consist of a target joint position at which to be moving at a certain
velocity, as well as a total time duration for the movement to the goal position
from the previous goal position.
"""

import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import JointState
from utils.find_joints import find_joints
from utils.constants import GET_CHAIN


class BrainNode(Node):
    """
    The brain of the poker robot, telling the robot where it needs to go and
    what action it needs to take at that location.
    """
    def __init__(self):
        super().__init__('brain')

        # Create a message and publisher to send goals to the control node.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/goal', 10)

        self.chain = GET_CHAIN(self)
        self.goal1 = find_joints(self.chain, np.array([-0.3, 0.2, 0.3]).reshape(3, 1), 0.0)
        self.goal2 = find_joints(self.chain, np.array([0.3, 0.2, 0.02]).reshape(3, 1), 0.0)

        # Wait for the control node to subscribe to the goal topic.
        while(not self.cmdpub.get_subscription_count()):
            pass

    def shutdown(self):
        """
        Shutdown the brain node at program exit.
        """
        self.destroy_node()

    def send_goal(self, q, qdot, T, type_str):
        """
        Send a goal to the control node to move to the given joint position q at
        the given velocity qdot over the given time duration T.
        """
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name = (type_str,)
        self.cmdmsg.position = q.flatten().tolist()
        self.cmdmsg.velocity = qdot
        self.cmdmsg.effort = (T,)
        self.cmdpub.publish(self.cmdmsg)

def main(args=None):
    """
    Run the brain node.
    """
    # Initialize ROS.
    rclpy.init(args=args)

    # Create the brain node.
    node = BrainNode()

    # Send a goal to the control node.
    node.send_goal(node.goal1, (-0.2, -0.2, 0.0, 0.0, 0.0), 10.0, 'GRAB')
    node.send_goal(node.goal2, (0.0, 0.0, 0.0, 0.0, 0.0), 8.0, 'GRAB')

    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Clean up the node when it is destroyed.
    node.destroy_node()
    rclpy.shutdown()
