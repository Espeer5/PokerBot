class Dealer():
    DECK_LOCATION = None

    def __init__(self, active_players, dealer_idx):
        self.active_players = active_players
        self.dealer_index = dealer_idx

    def run(self):
        for i in range(2 * len(self.active_players)):
            curr_player = self.active_players[(self.dealer_index + i) % len(self.active_players)]
            
            # pick up card at DECK_LOCATION
            # place card face down at curr_player.card_location

        

