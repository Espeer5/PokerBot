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
    def __init__(self, name):
        super().__init__(name)

        # Create a message and publisher to send goals to the control node.
        self.cmdmsg = JointState()
        self.cmdpub = self.create_publisher(JointState, '/goal', 10)

        self.chain = GET_CHAIN(self)
        self.goal1 = np.array([-0.3, 0.2, 0.01]).reshape(3, 1)
        self.goal2 = np.array([0.3, 0.2, 0.0]).reshape(3, 1)

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

    def act_at(self, goalpos, goal_th, type_str):
        """
        Send a sequence of commands to the control node to grab a card at the
        goal position with the given goal orientation of the end affector.
        """
        q_raised = find_joints(self.chain,
                               goalpos + np.array([0.0, 0.0, 0.05]).reshape(3, 1),
                               goal_th)
        q_goal = find_joints(self.chain, goalpos, goal_th)

        # find rotation from 
        self.send_goal(q_raised,
                       [0.0, -0.1, 0.0, 0.12, 0.0],
                       6.0,
                       type_str)
        self.send_goal(q_goal,
                       [0.0, 0.0, 0.0, -0.2, 0.0],
                       2.0, 'NONE')
        self.send_goal(q_raised, [0.0, 0.1, 0.0, -0.12, 0.0], 4.0, 'NONE')

def main(args=None):
    """
    Run the brain node.
    """
    # Initialize ROS.
    rclpy.init(args=args)

    # Create the brain node.
    node = BrainNode('brain')

    # Send a goal to the control node.
    for _ in range(8):
        node.act_at(node.goal1, 0.0, 'GB_CARD')
        node.act_at(node.goal2, 0.0, 'DROP')

    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Clean up the node when it is destroyed.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
