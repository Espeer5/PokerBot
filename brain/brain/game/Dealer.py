import numpy as np


class Dealer():
    DECK_LOCATION = np.array([-0.64, 0.045, 0.0]).reshape(3, 1)
    CARD_SIZE = (0.06, 0.09)

    def __init__(self, node, active_players, dealer_idx):
        self.node = node
        self.players = active_players
        self.dealer_index = dealer_idx

    def get_card_locations_from_card_box(self, card_box, raised=False):
        (x0, y0), (x1, y1) = card_box
        w = x1 - x0
        h = y1 - y0

        if w < h:
            theta = np.radians(90)

            total_vert_gap_size = h - 2 * self.CARD_SIZE[0]
            vert_gap_size = total_vert_gap_size / 3

            total_horizontal_gap_size = w - self.CARD_SIZE[1]
            horizontal_gap_size = total_horizontal_gap_size / 2

            card1x = x0 + horizontal_gap_size + (self.CARD_SIZE[1] / 2)
            card1y = y0 + vert_gap_size + (self.CARD_SIZE[0] / 2)

            card2x = card1x
            card2y = y1 - vert_gap_size - (self.CARD_SIZE[0] / 2)

            z = 0.225 if raised else 0.0

            coords1 = np.array([card1x, card1y, z]).reshape(3, 1)
            coords2 = np.array([card2x, card2y, z]).reshape(3, 1)
            return coords1, coords2, theta

        else:
            theta = 0

            total_vert_gap_size = h - self.CARD_SIZE[1]
            vert_gap_size = total_vert_gap_size / 2

            total_horizontal_gap_size = w - 2 * self.CARD_SIZE[0]
            horizontal_gap_size = total_horizontal_gap_size / 3

            card1x = x0 + horizontal_gap_size + (self.CARD_SIZE[0] / 2)
            card1y = y0 + vert_gap_size + (self.CARD_SIZE[1] / 2)

            card2x = x1 - horizontal_gap_size - (self.CARD_SIZE[0] / 2)
            card2y = card1y

            z = 0.225 if raised else 0.0

            coords1 = np.array([card1x, card1y, z]).reshape(3, 1)
            coords2 = np.array([card2x, card2y, z]).reshape(3, 1)
            return coords1, coords2, theta


    def run(self):
        # card_locations = {self.get_card_locations_from_card_box(curr_player.card_box) for curr_player in}
        for i in range(len(self.players)):
            raised = False
            if i == len(self.players) - 1:
                raised = True
            curr_player = self.players[(self.dealer_index + i) % len(self.players)]
            
            card1coords, card2coords, theta = self.get_card_locations_from_card_box(curr_player.card_box, raised)

            self.node.act_at(self.DECK_LOCATION, np.pi/2, "GB_CARD")
            self.node.act_at(card1coords, theta, "DROP")

            self.node.act_at(self.DECK_LOCATION, np.pi / 2, "GB_CARD")
            self.node.act_at(card2coords, theta, "DROP")