from Dealer import Dealer
from CommunityCardsDealer import CommunityCardsDealer
from Betting import Betting
from Showdown import Showdown
from Payout import Payout


class Game():

    def __init__(self):
        self.players = self.detect_active_players()
        self.dealer_idx = 0
        # self.curr_state = 
        

    def detect_active_players(self):
        # get chips
        # get player locations
        # see which player boxes are active
        pass

    
    def play_game(self):
        dealer_idx = 0

        while True:
            active_players = self.detect_active_players()
            # move dealer token

            dealer = Dealer(active_players, dealer_idx)
            dealer.run()

            while True:
                ccards_dealer = CommunityCardsDealer()
                ccards_dealer.run()

                betting = Betting()
                is_showdown = betting.run()

                if is_showdown:
                    showdown = Showdown(players=None, community_cards=None)
                    winning_player = showdown.run()

                    payout = Payout(winning_player)
                    payout.run()
                    break








