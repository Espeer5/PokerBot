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
from math import cos, sin
from time import sleep

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from std_msgs.msg import String
from utils.find_joints import find_joints
from detectors.message_types.ChipMessage import ChipMessage
from detectors.message_types.BackOfCardMessage import BackOfCardMessage
from detectors.message_types.CardPose import CardPose
from detectors.message_types.CardMessage import CardMessage
from utils.constants import GET_CHAIN, FLIP_PHI
from brain.game.Game import Game
import cv2

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

        # Create a listener for when the control node completes an action sequence
        self.action_sub = self.create_subscription(String, '/act_ID', self.act_ID_cb, 10)

        # Create a service for interacting with the BC detector node
        self.bc_cli = self.create_client(Trigger, '/bc_detector')

        # Create a service for interacting with the chip detector node
        self.ch_cli = self.create_client(Trigger, '/ch_detector')

        # Create a service for interacting with the FOC detector node
        self.foc_cli = self.create_client(Trigger, '/foc_detector')

        # Create a service for interacting with the FOC detector node
        self.bot_foc_cli = self.create_client(Trigger, '/bot_foc_detector')

        # Create a service for interacting with the BTN detector node
        self.btn_cli = self.create_client(Trigger, '/btn_detector')

        self.chain = GET_CHAIN(self)
        self.goal1 = np.array([-0.3, 0.2, 0.01]).reshape(3, 1)
        self.goal2 = np.array([0.3, 0.2, 0.0]).reshape(3, 1)

        # Wait for the control node to subscribe to the goal topic.
        while(not self.cmdpub.get_subscription_count()):
            pass

        self.PREV_ID = 0

        self.prev_complete = 0

    def shutdown(self):
        """
        Shutdown the brain node at program exit.
        """
        self.destroy_node()

    def act_ID_cb(self, msg):
        """
        Action taken when the control node completes an action sequence.
        """
        self.get_logger().info(f"Action sequence {msg.data} completed")
        self.prev_complete = int(msg.data)

    def send_goal(self, q, qdot, type_str, act_ID, T=None):
        """
        Send a goal to the control node to move to the given joint position q at
        the given velocity qdot over the given time duration T.
        """
        # self.get_logger().info(f"send_goal({q}, {qdot}, {T}, {type_str})")
        # self.get_logger().info("message sent")
        self.cmdmsg.header.stamp = self.get_clock().now().to_msg()
        self.cmdmsg.name = (type_str, str(act_ID))
        self.cmdmsg.position = q.flatten().tolist()
        self.cmdmsg.velocity = qdot
        self.cmdmsg.effort = (T,) if T is not None else (-1.0,)
        self.cmdpub.publish(self.cmdmsg)

    def act_at(self, goalpos, goal_th, type_str):
        """
        Send a sequence of commands to the control node to grab/drop a card at the
        goal position with the given goal orientation of the end affector.
        """
        self.get_logger().info(f"act_at({goalpos}, {goal_th}, {type_str})")
        self.PREV_ID += 1
        seq_ID = self.PREV_ID
        q_raised = find_joints(self.chain,
                               goalpos + np.array([0.0, 0.0, 0.04]).reshape(3, 1),
                               goal_th)
        if type_str == 'FLIP':
            q_goal = find_joints(self.chain, goalpos, goal_th, FLIP_PHI)
        elif type_str == "GB_CHIP":
            q_goal = find_joints(self.chain, goalpos, goal_th, -0.075)
        else:
            q_goal = find_joints(self.chain, goalpos, goal_th)

        self.send_goal(q_raised,
                       [0.0, -0.1, 0.0, 0.12, 0.0], type_str, seq_ID)
        swivel_velo = 0.0 if type_str in ['DROP', 'GB_CHIP'] else -0.2
        self.get_logger().info(f"{type_str}, {swivel_velo}")
        d_time = 1.0 if type_str == 'DROP' else 2.0
        self.send_goal(q_goal,
                       [0.0, 0.0, 0.0, swivel_velo, 0.0], 'NONE', seq_ID, d_time)
        if type_str == 'FLIP':
            theta = np.arctan2(goalpos[1], goalpos[0])
            d_pos = np.array([0.2 * cos(theta), 0.2 * sin(theta), -0.01]).reshape(3, 1)
            q_retract = find_joints(self.chain, goalpos - d_pos, goal_th, FLIP_PHI + 0.35)
            self.send_goal(q_retract, [0.0, 0.0, 0.0, 0.0, 0.0], 'NONE', seq_ID, 2.0)
        else:
            phi_velo = 0.0 if type_str in ['DROP', 'GB_CHIP'] else 0.1
            swivel_velo = 0.0 if type_str in ['DROP', 'GB_CHIP'] else -0.12
            self.get_logger().info(f"{type_str}, {swivel_velo}")
            self.send_goal(q_raised, [0.0, phi_velo, 0.0, swivel_velo, 0.0], 'NONE', seq_ID, 1.5)
        return seq_ID

    def get_bc(self):
        """
        Get the list of all the back of cards and their locations in the workspace.
        """
        req = Trigger.Request()
        future = self.bc_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result().message
        if msg not in ["No cards found", "No image available"]:
            return BackOfCardMessage.from_string(msg)
        return msg
    
    def get_ch(self):
        """
        Get the list of all the chips and their locations in the workspace.
        """
        req = Trigger.Request()
        future = self.ch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result().message
        if msg is not None:
            return ChipMessage.from_string(msg)
        return None

    
    def get_foc(self):
        """
        Get the list of all the fronts of cards and their locations in the 
        workspace.
        """
        req = Trigger.Request()
        future = self.foc_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result().message
        if msg not in ["No cards found", "No image available"]:
            return CardMessage.from_string(msg)
        return None
    

    def get_btn(self):
        """
        Get the button location in the workspace.
        """
        req = Trigger.Request()
        future = self.btn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result().message
        if msg is not None and len(msg) > 0:
            return [float(coord) for coord in msg.split(", ")]
        return None
    
    def get_bot_foc(self):
        """
        Get the robot's cards from the secondary camera.
        """
        req = Trigger.Request()
        future = self.bot_foc_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        msg = future.result().message
        if msg not in ["No cards found", "No image available"]:
            return CardMessage.from_string(msg)
        return None


def main(args=None):
    """
    Run the brain node.
    """
    # Initialize ROS.
    rclpy.init(args=args)

    # Create the brain node.
    node = BrainNode('brain')

    # Wait for the control node to be ready
    sleep(4)

    # Send a goal to the control node.
    # for _ in range(8):
    #     node.act_at(node.goal1, 0.0, 'GB_CARD')
    #     node.act_at(node.goal2, 0.0, 'DROP')
    # while True:
    #     node.get_logger().info("RAN")
    #     sleep(1)
    #     node.get_logger().info(f"{node.get_foc()}")
    game = Game(node)
    game.run()
    # while True:
    #     node.get_logger().info("RAN")
    #     chip_message = node.get_ch()
    #     if chip_message is not None:
    #         for chip in chip_message.chips:
    #             node.act_at(np.array(chip.coords).reshape(3, 1), 0, "GB_CHIP")
    #             node.act_at(np.array([0.0, 0.5, 0.0]).reshape(3, 1), 0, "DROP")
    #             sleep(10)
            # cv2.waitKey(0)
            # node.get_logger().info("Got your input")
            
        # sleep(1)
        # node.get_logger().info(f"{node.get_ch()}")
    # Spin the node so the callback function is called.
    rclpy.spin(node)

    # Clean up the node when it is destroyed.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
