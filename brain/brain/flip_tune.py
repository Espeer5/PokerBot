"""
This module contains a PokerBot brain that runs a simple card flipping demo,
allowing the tuning of the card flipping trajectory.
"""

import rclpy
import numpy as np
from time import sleep

from utils.find_joints import find_joints
from utils.constants import GET_CHAIN
from brain.brain import BrainNode


class FlipBrain(BrainNode):
    """
    The brain of the poker robot, telling the robot where it needs to go in
    order to run a simple card flipping demo.
    """
    def __init__(self, name):
        super().__init__(name)

        self.chain = GET_CHAIN(self)
        self.goal1 = np.array([-0.3, 0.3, -0.01]).reshape(3, 1)
        self.goal2 = np.array([0.02, 0.4, 0.05]).reshape(3, 1)

    def flip(self):
        """
        Run the card flipping demo.
        """
        # Move to the first goal position.
        self.act_at(self.goal1, 0.0, "GB_CARD")
        # Move to the second goal position.
        self.act_at(self.goal2, 0.0, "FLIP")


def main(args=None):
    """
    Run the brain node.
    """
    # Initialize ROS.
    rclpy.init(args=args)

    # Create the brain node.
    node = FlipBrain('flipper')

    # Wait 4 seconds and begin the demo
    node.get_logger().info("waiting 4 seconds")
    sleep(4)

    for _ in range(4):
        node.flip()

    node.get_logger().info("flipper main")

    # Run the brain node.
    rclpy.spin(node)

    # Clean up the node.
    node.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()