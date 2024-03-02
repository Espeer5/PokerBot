import numpy as np
from enum import Enum

from brain.game.constants import DECK_LOCATION, get_card_locations_from_card_box


class Dealer():

    def __init__(self, node, active_players):
        self.node = node
        self.players = active_players
        self.dealer_index = 0

    def run(self):
        # card_locations = {self.get_card_locations_from_card_box(curr_player.card_box) for curr_player in}
        for i in range(len(self.players)):
            raised = False
            if i == len(self.players) - 1:
                raised = True
            curr_player = self.players[(self.dealer_index + i) % len(self.players)]
            
            card1coords, card2coords, theta = get_card_locations_from_card_box(curr_player.card_box, raised)

            self.node.act_at(DECK_LOCATION, np.pi/2, "GB_CARD")
            self.node.act_at(card1coords, theta, "DROP")

            self.node.act_at(DECK_LOCATION, np.pi / 2, "GB_CARD")
            self.node.act_at(card2coords, theta, "DROP")