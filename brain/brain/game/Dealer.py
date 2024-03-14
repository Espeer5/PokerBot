import numpy as np
from enum import Enum
import rclpy
from detectors.message_types.Card import Card
from detectors.message_types.CardPose import CardPose
from brain.game.constants import DECK_LOCATION, get_card_locations_from_card_box


class Dealer():

    def __init__(self, node, active_players):
        self.node = node
        self.players = active_players
        self.dealer_index = 0

    def check_and_fix(self, card_locations):
        card_locations = set(card_locations)

        extra = []
        missing = []

        iters = 0
        while len(extra) > 0 or len(missing) > 0 or iters == 0:
            iters += 1

            message = None
            while message is None:
                message = self.node.get_bc()

            back_of_cards = set(message.poses) if type(message) != str else set()
            
            extra = list(back_of_cards - card_locations)
            missing = list(card_locations - back_of_cards)

            ID = None
            for pose in missing:
                self.node.act_at(DECK_LOCATION, np.pi/2, "GB_CARD")
                self.node.get_logger().info(f"{pose.coords}")
                wait = pose == missing[-1]
                ID = self.node.act_at(np.array(pose.coords).reshape(3, 1), pose.theta, "DROP", wait=wait)

            for pose in extra:
                self.node.get_logger().info(f"{pose.coords}")
                self.node.act_at(np.array(pose.coords).reshape(3, 1), pose.theta, "GB_CARD")
                wait = pose == extra[-1]
                ID = self.node.act_at(DECK_LOCATION, np.pi/2, "DROP", wait=wait)
                    
            while ID is not None and self.node.prev_complete != ID:
                rclpy.spin_once(self.node)

    def run(self):
        # card_locations = {self.get_card_locations_from_card_box(curr_player.card_box) for curr_player in}
        card_locations = []

        assert len(self.players) > 0
        for i in range(len(self.players)):
            raised = self.players[i].player_id == "robot"
            curr_player = self.players[(self.dealer_index + i) % len(self.players)]
            
            coords1, coords2, theta = get_card_locations_from_card_box(curr_player.card_box, raised)


            card_locations.append(CardPose(coords1, theta))
            card_locations.append(CardPose(coords2, theta))


        for i in range(0, len(card_locations), 2):
            self.node.act_at(DECK_LOCATION, np.pi/2, "GB_CARD")
            self.node.act_at(np.array(card_locations[i].coords).reshape(3, 1), card_locations[i].theta, "DROP")

        for i in range(1, len(card_locations), 2):
            wait = i == len(card_locations) - 1

            self.node.act_at(DECK_LOCATION, np.pi/2, "GB_CARD")
            ID = self.node.act_at(np.array(card_locations[i].coords).reshape(3, 1), card_locations[i].theta, "DROP", wait=wait)

        while self.node.prev_complete != ID:
            rclpy.spin_once(self.node)

        self.check_and_fix(card_locations[:-2])