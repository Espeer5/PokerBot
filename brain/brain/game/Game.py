from brain.game.Dealer import Dealer
from brain.game.CommunityCardsDealer import CommunityCardsDealer
from brain.game.Betting import Betting
from brain.game.Showdown import Showdown
from brain.game.Payout import Payout
from brain.game.Player import Player
from brain.game.constants import PLAYERS, show


class Game():
    """
    This class gives PokerBot's game playing logic.
    """

    def __init__(self, node):
        self.node = node
        self.node.get_logger().info("Game initialized")
        self.players = PLAYERS
        #self.players = self.detect_active_players()
        #self.node.get_logger().info(f"Active players: {[p.chip_box for p in self.players]}")
        # self.dealer_idx = 0
        # self.curr_state = 
        

    def detect_active_players(self):
        """
        Detects the active players in the current game by bucketing red chips
        into the player's chip box that the chips belong to.
        """
        chip_loc_set = self.node.get_ch().chips
        player_set = []
        # Bucket every chip into a certain player's chip box based on location
        for chip in chip_loc_set:
            self.node.get_logger().info(f"chip: {chip.coords}")
            for player in PLAYERS:
                if player not in player_set:
                    if (player.chip_box[0][0] <= chip.coords[0] <= player.chip_box[1][0]
                        and player.chip_box[0][1] <= chip.coords[1] <= player.chip_box[1][1]):
                        player_set.append(player)
            # sort the players in order of x-coordinates, least to greates
            player_set.sort(key=lambda x: x.chip_box[0][0])
        return player_set

    
    def run(self):
        dealer_idx = 0

        while True:
            # Initialize dealing states
            dealer = Dealer(self.node, self.players, dealer_idx)

            # Deal player hands
            dealer.run()

            # Initialize community card dealing states
            CC_dealer = CommunityCardsDealer(self.node)

            # Deal the flop
            CC_dealer.run()

            # Deal the turn
            CC_dealer.run()

            # Deal the river
            CC_dealer.run()

            show(self.node)
            break
            

            # while True:
            #     ccards_dealer = CommunityCardsDealer()
            #     ccards_dealer.run()

            #     betting = Betting()
            #     is_showdown, active_players = betting.run()

            #     if is_showdown:
            #         showdown = Showdown(players=None, community_cards=None)
            #         winning_player = showdown.run()

            #         payout = Payout(winning_player)
            #         payout.run()
            #         break








