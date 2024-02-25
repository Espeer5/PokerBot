import numpy as np


class Dealer():
    DECK_LOCATION = (0.28, 0.30, 0.0)
    CARD_SIZE = (0.06, 0.09)

    def __init__(self, active_players, dealer_idx):
        self.players = active_players
        self.dealer_index = dealer_idx

    def get_card_locations_from_card_box(self, card_box):
        (x0, y0), (x1, y1) = card_box
        w = x1 - x0
        h = y1 - y0

        if w < h:
            theta = 90

            total_vert_gap_size = h - 2 * self.CARD_SIZE[0]
            vert_gap_size = total_vert_gap_size / 3

            total_horizontal_gap_size = w - self.CARD_SIZE[1]
            horizontal_gap_size = total_horizontal_gap_size / 2

            card1x = x0 + horizontal_gap_size + (self.CARD_SIZE[1] / 2)
            card1y = y0 + vert_gap_size + (self.CARD_SIZE[0] / 2)

            card2x = card1x
            card2y = y1 - vert_gap_size - (self.CARD_SIZE[0] / 2)

            return np.array([card1x, card1y, 0.0]), np.array([card2x, card2y, 0.0]), theta

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

            return np.array([card1x, card1y, 0.0]), np.array([card2x, card2y, 0.0]), theta


    def run(self):
        # card_locations = {self.get_card_locations_from_card_box(curr_player.card_box) for curr_player in}

        for i in range(len(self.players)):
            curr_player = self.players[(self.dealer_index + i) % len(self.players)]
            
            card1coords, card2coords, theta = self.get_card_locations_from_card_box(curr_player.card_box)

            self.act_at(self.DECK_LOCATION, 0, "GB_CARD")
            self.act_at(card1coords, theta, "DROP")

            self.act_at(self.DECK_LOCATION, 0, "GB_CARD")
            self.act_at(card2coords, theta, "DROP")