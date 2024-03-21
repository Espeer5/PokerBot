import rclpy

from brain.game.Dealer import Dealer
from brain.game.CommunityCardsDealer import CommunityCardsDealer
from brain.game.Betting import Betting
from brain.game.Showdown import Showdown
from brain.game.Payout import Payout
from brain.game.Player import Player
from brain.game.constants import PLAYERS

from utils.text_to_speech import text_to_speech


class Game():
    """
    This class gives PokerBot's game playing logic.
    """

    def __init__(self, node):
        self.node = node
        self.players = self.detect_active_players()
        # self.players = PLAYERS
        self.ccards = []
        # self.node.get_logger().info(f"Active players: {[p.chip_box for p in self.players]}")
        # self.dealer_idx = 0
        # self.curr_state = 
        self.node.get_logger().info("Game initialized")
        text_to_speech("starting game","start")
        

    def detect_active_players(self):
        """
        Detects the active players in the current game by bucketing red chips
        into the player's chip box that the chips belong to.
        """
        chip_loc_set = self.node.get_ch().chips
        self.node.get_logger().info(f"chips={chip_loc_set}")
        player_set = []
        # Bucket every chip into a certain player's chip box based on location
        for chip in chip_loc_set:
            for player in PLAYERS:
                if player not in player_set:
                    if (player.chip_box[0][0] <= chip.coords[0] <= player.chip_box[1][0]
                        and player.chip_box[0][1] <= chip.coords[1] <= player.chip_box[1][1]):
                        player_set.append(player)
                        self.node.get_logger().info(f"chip: {chip.coords}, player: {player.player_id}")
            # sort the players in order of x-coordinates, least to greatest
            player_set.sort()
        return player_set

    def run(self):
        # showdown = Showdown(self.node, PLAYERS[:-1], [("Ace", "Spades"), ("Five", "Hearts"), ("Ten", "Hearts"), ("Ace", "Diamonds"), ("Ten", "Spades")])
        # winning_player = showdown.run()
        # self.node.get_logger().info(f"Player {winning_player.player_id} has won the round!")
        # return


        while True:
            # Initialize dealing states
            dealer = Dealer(self.node, self.players)

            # Deal player hands
            dealer.run()

            ccards_dealer = CommunityCardsDealer(self.node)

            pot_size = 0
            round_number = 1
            while True: 
                betting = Betting(self.node, self.players, pot_size, round_number, self.ccards)
                is_showdown, self.players, pot_size = betting.run()
                round_number += 1
                    
                if len(self.players) == 1:
                    # payout = Payout(self.players[0])
                    # payout.run()
                    break

                if is_showdown:
                    showdown = Showdown(self.node, self.players[:-1], self.ccards)
                    winning_player = showdown.run()
                    self.node.get_logger().info(f"Player {winning_player.player_id} has won the round!")

                    # payout = Payout(winning_player)
                    # payout.run()
                    break

                self.ccards += ccards_dealer.run()








