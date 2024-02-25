from brain.game.Dealer import Dealer
from brain.game.CommunityCardsDealer import CommunityCardsDealer
from brain.game.Betting import Betting
from brain.game.Showdown import Showdown
from brain.game.Payout import Payout
from brain.game.Player import Player


# for testing purposes
PLAYERS = [Player(None, [(-0.555, 0.38), (-0.44, 0.53)]),
           Player(None, [(-0.22, 0.63), (-0.05, 0.74)]),
           Player(None, [(0.19, 0.63), (0.35, 0.74)]),
           Player(None, [(0.44, 0.21), (0.56, 0.37)]),
           Player(None, [(0.235, 0.045), (0.395, 0.16)])]


class Game():

    def __init__(self, node):
        self.node = node
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
            dealer = Dealer(self.node, active_players, dealer_idx)
            dealer.run()
            break
            

            # while True:
            #     ccards_dealer = CommunityCardsDealer()
            #     ccards_dealer.run()

            #     betting = Betting()
            #     is_showdown = betting.run()

            #     if is_showdown:
            #         showdown = Showdown(players=None, community_cards=None)
            #         winning_player = showdown.run()

            #         payout = Payout(winning_player)
            #         payout.run()
            #         break








