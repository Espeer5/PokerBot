from Dealer import Dealer
from CommunityCardsDealer import CommunityCardsDealer
from Betting import Betting
from Showdown import Showdown
from Payout import Payout
from Player import Player

# for testing purposes
PLAYERS = [Player(None, [(-55.5, 38), (44, 53)])]


class Game():

    def __init__(self):
        # self.players = self.detect_active_players()
        # self.dealer_idx = 0
        # self.curr_state = 
        pass
        

    # def detect_active_players(self):
    #     # get chips
    #     # get player locations
    #     # see which player boxes are active
    #     pass

    
    def run(self):
        dealer_idx = 0

        while True:
            active_players = PLAYERS
            # move dealer token

            dealer = Dealer(active_players, dealer_idx)
            dealer.run()
            exit()

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








