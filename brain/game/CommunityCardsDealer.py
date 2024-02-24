from enum import Enum
# from brain.brain import act_at

class State(Enum):
        FLOP = 1
        TURN = 2
        RIVER = 3

class CommunityCardsDealer():
    DECK_LOCATION = None
    FLOP_LOCATIONS = [None, None, None]
    TURN_LOCATION = None
    RIVER_LOCATION = None

    def __init__(self):
        self.curr_state = State.FLOP

    def run(self):
        if self.curr_state == State.FLOP:
            for i in range(3):
                pass
                # pick up card from DECK_LOCATION
                # turn over card at FLOP_LOCATION[i]
            self.curr_state = State.TURN

        elif self.curr_state == State.TURN:
            # pick up card from DECK_LOCATION
            # turn over card at TURN_LOCATION
            self.curr_state = State.RIVER

        elif self.curr_state == State.RIVER:
            # pick up card from DECK_LOCATION
            # turn over card at RIVER_LOCATION
            self.curr_state == State.FLOP