"""
Constants used by the poker game agent.
"""

from time import sleep
import numpy as np

from brain.game.Player import Player

# The player id is mapped to the locations of each players' cards 
CARD_LOC = {1:((-0.4725,0.415, 0.0),(-0.4725,0.49, 0.0), np.pi / 2), 2: ((-0.17,0.67, 0.0),(-0.09,0.67, 0.0), 0.0),
            3: ((0.227,0.66, 0.0),(0.307,0.66, 0.0), 0.0)}

# The locations of players in the game (x1, y1), (x2, y2)
PLAYERS = [Player(1, [(-0.56, 0.26), (-0.38, 0.55)], [(-0.535, 0.367), (-0.40, 0.517)]),
           Player(2, [(-0.31, 0.605), (-0.17, 0.755)], [(-0.202, 0.635), (-0.045, 0.73)]),
           Player(3, [(0.065, 0.61), (0.375, 0.76)], [(0.1875, 0.58), (0.3575, 0.74)]),
           Player("robot", [(0.30, 0.22), (0.44, 0.52)], [(0.4725, 0.3), (0.5925, 0.48)])]


# The constant location of the deck
DECK_LOCATION = np.array([-0.34, 0.05, 0.0]).reshape(3, 1)

# The size of a card
CARD_SIZE = (0.06, 0.09)

# Community card locations
# FLOP_LOCATIONS = [
#                     np.array([-0.52, 0.27, 0.0]).reshape(3, 1),
#                     np.array([-0.44, 0.27, 0.0]).reshape(3, 1),
#                     np.array([-0.36, 0.27, 0.0]).reshape(3, 1)
#                 ]
# TURN_LOCATION = np.array([-0.28, 0.27, 0.0]).reshape(3, 1)
# RIVER_LOCATION = np.array([-0.20, 0.27, 0.0]).reshape(3, 1)

FLOP_LOCATIONS = [
                    np.array([-0.16, 0.25, 0.0]).reshape(3, 1),
                    np.array([-0.08, 0.25, 0.0]).reshape(3, 1),
                    np.array([0, 0.25, 0.0]).reshape(3, 1)
                ]
TURN_LOCATION = np.array([0.08, 0.25, 0.0]).reshape(3, 1)
RIVER_LOCATION = np.array([0.16, 0.25, 0.0]).reshape(3, 1)

# Constant location where cards are flipped in the workspace
FLIP_LOC = np.array([0.02, 0.55, 0.05]).reshape(3, 1)

# Bounding box on the card flip location in which to find flipped card
FLIP_BOUND_BOX = [(-0.01, 0.45), (0.05, 0.65)]

# Utility functions

def find_card(node):
        """
        Finds a card close to the flip location using the card detector
        """
        loc = None
        theta = None
        rank = None
        suit = None
        while loc is None:
            cards = node.get_foc()
            if cards is not None and type(cards) is not str:
                cards = cards.cards
                for card in cards:
                    pot_loc = card.pose.coords
                    node.get_logger().info(f"Potential card location: {pot_loc}")
                    # The card is the right card if it is within tolerance of the
                    # flipping location
                    if (FLIP_BOUND_BOX[0][0] <= pot_loc[0] <= FLIP_BOUND_BOX[1][0] and
                        FLIP_BOUND_BOX[0][1] <= pot_loc[1] <= FLIP_BOUND_BOX[1][1]):
                        rank = card.rank
                        suit = card.suit
                        loc = np.array(pot_loc).reshape(3, 1) + np.array([0.0, 0.03, 0.0]).reshape(3, 1)
                        theta = card.pose.theta
                        break
        return loc, theta, (rank, suit)


def get_card_locations_from_card_box(card_box, raised=False):
    """
    Find the locations to put a plaer's two cards based on the bounds of their 
    card box. 
    """
    (x0, y0), (x1, y1) = card_box
    w = x1 - x0
    h = y1 - y0

    if w < h:
        theta = np.radians(90)

        total_vert_gap_size = h - 2 * CARD_SIZE[0]
        vert_gap_size = total_vert_gap_size / 3

        total_horizontal_gap_size = w - CARD_SIZE[1]
        horizontal_gap_size = total_horizontal_gap_size / 2

        card1x = x0 + horizontal_gap_size + (CARD_SIZE[1] / 2)
        card1y = y0 + vert_gap_size + (CARD_SIZE[0] / 2)

        card2x = card1x
        card2y = y1 - vert_gap_size - (CARD_SIZE[0] / 2)

        z = 0.21 if raised else -0.01

        coords1 = card1x, card1y, z
        coords2 = card2x, card2y, z
        return coords1, coords2, theta

    else:
        theta = 0

        total_vert_gap_size = h - CARD_SIZE[1]
        vert_gap_size = total_vert_gap_size / 2

        total_horizontal_gap_size = w - 2 * CARD_SIZE[0]
        horizontal_gap_size = total_horizontal_gap_size / 3

        card1x = x0 + horizontal_gap_size + (CARD_SIZE[0] / 2)
        card1y = y0 + vert_gap_size + (CARD_SIZE[1] / 2)

        card2x = x1 - horizontal_gap_size - (CARD_SIZE[0] / 2)
        card2y = card1y

        z = 0.21 if raised else 0.0

        coords1 = card1x, card1y, z
        coords2 = card2x, card2y, z
        return coords1, coords2, theta
