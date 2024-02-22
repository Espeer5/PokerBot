"""
Code used to run simple gravity compensation model in order to tune the poker
bot gravity compensation parameters. A Gravity compensating node is the simplest
case of a robot control node defined in the constants file. This node is used to
send torques to the robot in order to counteract the effects of gravity on the
robot's joints and provide no position or velocity control.
"""

import rclpy

from utils.constants import CONTROL_NODE


def main(args=None):
    """
    Send gravity compensating torques to the robot.
    """
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the DEMO node.
    node = CONTROL_NODE('grav_demo')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
