class Betting():
    POT_LOCATION = [(None, None), (None, None)]
    SMALL_BLIND = 1
    BIG_BLIND = 2

    def __init__(self, prev_state, active_players, dealer_idx):
        self.prev_state = prev_state
        self.players = active_players
        self.start_idx = dealer_idx + 1
        self.chips_in_pot = self.detect_chips_in_pot()
    
    def in_pot(self, chip_location):
        x, y = chip_location
        (x0, y0), (x1, y1) = self.POT_LOCATION
        return x0 <= x and x <= x1 and y0 <= y and y <= y1

    def detect_chips_in_pot(self):
        chips = None
        return sum([chip.value for chip in chips if self.in_pot(chips)])
        
    def in_players_card_box(self, player, card_location):
        x, y = card_location
        (x0, y0), (x1, y1) = player.card_box
        return x0 <= x and x <= x1 and y0 <= y and y <= y1

    def detect_fold(self, player):
        back_of_cards = None
        for card in back_of_cards:
            if self.in_players_card_box(player, card):
                return True
        return False
    
    def detect_curr_bettor(self):
        betting_token_location = None
        # find closest player
        pass

    
    def run(self):
        pass
        # player_bet_amounts = {}
        # num_bets_made = 0

        # if self.prev_state == "Dealing":
        #     curr_chips_in_pot = 0
        #     while curr_chips_in_pot - self.chips_in_pot < self.SMALL_BLIND:
        #         # prompt user
        #         curr_chips_in_pot = self.detect_chips_in_pot()
        #         pass
        #     self.chips_in_pot = curr_chips_in_pot
        #     curr_player = self.players[(self.start_idx + num_bets_made) % len(self.players)]
        #     player_bet_amounts[curr_player] = self.SMALL_BLIND
        #     num_bets_made += 1

        #     while curr_chips_in_pot - self.chips_in_pot < self.BIG_BLIND:
        #         # prompt user
        #         curr_chips_in_pot = self.detect_chips_in_pot()
        #         pass
        #     self.chips_in_pot = curr_chips_in_pot
        #     curr_player = self.players[(self.start_idx + num_bets_made) % len(self.players)]
        #     player_bet_amounts[curr_player] = self.BIG_BLIND
        #     num_bets_made += 1


        # while num_bets_made < len(self.players):
        #     curr_player = self.players[(self.start_idx + num_bets_made) % len(self.players)]

        #     while not self.detect_fold(curr_player) and 
        # while loop through users
            # while user has not called or folded
                # prompt user
            
            # if user raised
            # reset loop


        
            

        

