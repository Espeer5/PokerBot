"""
This brain node receives back of card locations from the back of card detector 
and then proceeds to collect and stack the cards in a different location.
"""

import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg  import Point

from utils.find_joints import find_joints
from utils.constants import GET_CHAIN
from brain.brain import BrainNode
from detectors.message_types.BackOfCardMessage import BackOfCardMessage


class CollectBrain(BrainNode):
    """
    The brain of the poker robot, telling the robot where it needs to go and
    what action it needs to take at that location.
    """
    def __init__(self, name):
        super().__init__(name)

        # Create a subscriber to the back card detector
        self.sub = self.create_subscription(
            String, '/BackOfCard', self.collect, 1)
        
        self.drop_loc = np.array([-0.2, 0.3, 0.0]).reshape(3, 1)
        
    def collect(self, msg):
        """
        Upon receiving a back of card location, the robot will proceed to collect
        the card and stack it in a different location.
        """
        message = BackOfCardMessage.from_string(msg.data)
        for pose in message.poses:
            # Collect the card
            x, y, z = pose.coords
            goal_pos = np.array([x, y + 0.025, z]).reshape(3, 1)
            goal_th = pose.theta
            self.act_at(goal_pos, goal_th, "GB_CARD")
            # Stack the card
            self.act_at(self.drop_loc, 0.0, "DROP")


def main(args=None):
    """
    Run the brain node.
    """
    # Initialize ROS.
    rclpy.init(args=args)

    # Create the brain node.
    node = CollectBrain('collector')

    node.get_logger().info("collector main")

    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Clean up the node when it is destroyed.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
