"""
This module contains the logic for the poker game state and how it may evolve. 
A game state in poker consists of the active players and their stacks, the
community cards, the pot, the dealer index, the state, and the phase of that
state. For example, the state may be "DEALING" and the phase may be something
like "PLAYER1 CARD 2". The game state is used by the game node to keep track of 
the game and to make decisions based on the current state of the game.
"""

class PokerState():
    """
    A class representing the state of a poker game.
    """
    def __init__(self, players, dealer_idx):
        self.players = players
        self.dealer_idx = dealer_idx
        self.community_cards = []
        self.pot = 0
        self.state = "DEALING"
        self.phase = "PLAYER1 CARD 1"


def read_state(self):
    """
    Read the state of the game from the workspace.
    """
    chips = self.node.get_ch()
    boc = self.node.get_boc()
    foc = self.node.get_foc()