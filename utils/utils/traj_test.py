"""
Tests the ability of the pokerbot to accurately achieve a desired position and 
orientation in task space.
"""

import numpy as np
import rclpy

from utils.constants import GET_CHAIN, CONTROL_NODE
from utils.find_joints import find_joints
from utils.TrajectoryUtils import goto5
from utils.pump_util import send_pwm

# Define the desired position and orientation for the robot to achieve
TEST_POINT = np.array([-0.1, 0.4, 0.2]).reshape(3, 1)
TEST_ANGLE = 0.0


class Trajectory():
    """
    A trajectory class which is added to a control node inheriting from the base 
    control node in order to dictate joint positions and velocities to the poker
    bot. Based on the time, will order the robot to perform a joint spline to 
    the positions computed to achieve the targets defined above.
    """
    def __init__(self, node, q0):
        self.node = node
        self.q0 = np.array(q0).reshape(5, 1)
        self.chain = GET_CHAIN(node)
        self.goal_joints = find_joints(self.chain, TEST_POINT, TEST_ANGLE)
        self.q = q0
        self.suction = True
        send_pwm(180)
    
    def evaluate(self, t, dt):
        if t < 2:
            return self.q0.flatten().tolist(), np.zeros((5, 1)).flatten().tolist()
        elif t < 10:
            q, qdot = goto5(t-2, 8, self.q0, self.goal_joints)
            self.q = q
            return q.flatten().tolist(), qdot.flatten().tolist()
        else:
            if self.suction:
                send_pwm(0)
                self.suction = False
            return self.q.flatten().tolist(), np.zeros((5, 1)).flatten().tolist()


class TestNode(CONTROL_NODE):
    """
    A test node which inherits from the base control node and uses the trajectory
    class to dictate the joint positions and velocities to the poker bot.
    """
    def __init__(self):
        super().__init__("test_node")
        self.traj = Trajectory(self, self.actpos)


def main(args=None):
    """
    Run the test node.
    """
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the test node.
    node = TestNode()

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
