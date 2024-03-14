from enum import Enum
import numpy as np
from time import sleep
import rclpy
# from brain.brain import act_at

from brain.game.constants import find_card
from detectors.message_types.CardPose import CardPose
from brain.game.constants import (DECK_LOCATION, FLOP_LOCATIONS, TURN_LOCATION,
                                  RIVER_LOCATION, FLIP_LOC)

class State(Enum):
        FLOP = 1
        TURN = 2
        RIVER = 3


class CommunityCardsDealer():

    def __init__(self, node):
        self.curr_state = State.FLOP
        self.node = node
        self.grab_loc = DECK_LOCATION + np.array([0.02, 0.0, 0.0]).reshape(3, 1)

    def check_and_fix(self, expected):
        expected_poses = set()
        for loc in expected:
            x, y, z = loc
            expected_poses.add(CardPose((x[0], y[0], z[0]), 0.0))

        missing = []
        card_info = []

        iters = 0
        while len(missing) > 0 or iters == 0:
            iters += 1

            message = None
            while message is None:
                message = self.node.get_foc()
 
            actual_poses = set([card.pose for card in message.cards]) if type(message) != str else set()
            missing = list(expected_poses - actual_poses)
            self.node.get_logger().info(f"missing={missing}")

            ID = None
            for pose in missing:
                self.node.act_at(self.grab_loc, np.pi/2, "GB_CARD")
                wait_ID = self.node.act_at(FLIP_LOC, 0.0, "FLIP")
                while self.node.prev_complete != wait_ID:
                    rclpy.spin_once(self.node)

                loc, theta, (rank, suit) = find_card(self.node)
                card_info.append((rank, suit))
                self.node.get_logger().info(f"{rank} of {suit}")
                self.node.act_at(loc, theta, "GB_CARD")

                wait = pose == missing[-1]
                ID = self.node.act_at(np.array(pose.coords).reshape(3, 1), pose.theta, "DROP", wait=wait)

            while ID is not None and self.node.prev_complete != ID:
                rclpy.spin_once(self.node)

        return card_info

    def run(self):
        if self.curr_state == State.FLOP:
            card_info = []
            for i in range(3):
                self.node.act_at(self.grab_loc, np.pi / 2, "GB_CARD")
                wait_ID = self.node.act_at(FLIP_LOC, 0.0, "FLIP")
                while self.node.prev_complete != wait_ID:
                    rclpy.spin_once(self.node)

                loc, theta, (rank, suit) = find_card(self.node)
                card_info.append((rank, suit))
                self.node.get_logger().info(f"{rank} of {suit}")
                self.node.act_at(loc, theta, "GB_CARD")
                wait = True if i == 2 else False
                wait_ID = self.node.act_at(FLOP_LOCATIONS[i], 0.0, "DROP", wait=wait)
                while self.node.prev_complete != wait_ID:
                    rclpy.spin_once(self.node)
    
            self.curr_state = State.TURN
            card_info += self.check_and_fix(FLOP_LOCATIONS)
            card_info = card_info[-3:]
            return card_info

        elif self.curr_state == State.TURN:
            self.node.act_at(self.grab_loc, np.pi / 2, "GB_CARD")
            wait_ID = self.node.act_at(FLIP_LOC, 0.0, "FLIP")
            while self.node.prev_complete != wait_ID:
                rclpy.spin_once(self.node)

            loc, theta, (rank, suit) = find_card(self.node)
            self.node.get_logger().info(f"{rank} of {suit}")
            self.node.act_at(loc, theta, "GB_CARD")
            wait_ID = self.node.act_at(TURN_LOCATION, 0.0, "DROP", wait=True)
            while self.node.prev_complete != wait_ID:
                rclpy.spin_once(self.node)
            self.curr_state = State.RIVER
            card_info = [(rank, suit)]
            card_info += self.check_and_fix([TURN_LOCATION])
            card_info = card_info[-1:]
            return card_info

        elif self.curr_state == State.RIVER:
            self.node.act_at(self.grab_loc, np.pi / 2, "GB_CARD")
            wait_ID = self.node.act_at(FLIP_LOC, 0.0, "FLIP")
            while self.node.prev_complete != wait_ID:
                rclpy.spin_once(self.node)

            loc, theta, (rank, suit) = find_card(self.node)
            self.node.get_logger().info(f"{rank} of {suit}")
            self.node.act_at(loc, theta, "GB_CARD")
            wait_ID = self.node.act_at(RIVER_LOCATION, 0.0, "DROP", wait=True)
            while self.node.prev_complete != wait_ID:
                rclpy.spin_once(self.node)
            self.curr_state == State.FLOP
            card_info = [(rank, suit)]
            card_info += self.check_and_fix([RIVER_LOCATION])
            card_info = card_info[-1:]
            return card_info
