import numpy as np


class Betting():
    POT_LOCATION = [(-0.25, 0.20), (0.15, 0.50)]
    SMALL_BLIND = 1
    BIG_BLIND = 2

    def __init__(self, node, active_players):
        self.node = node
        self.prev_state = "dealing"
        self.players = active_players
        self.chips_in_pot = self.detect_chips_in_pot()
        self.num_rounds_of_betting = 0
    
    def in_pot(self, chip_location):
        x, y, _ = chip_location
        (x0, y0), (x1, y1) = self.POT_LOCATION
        return x0 <= x and x <= x1 and y0 <= y and y <= y1
    
    def get_chip_value(self, chip):
        if chip.color == "blue":
            return 1
        elif chip.color == "black":
            return 2
        else:
            return 0

    def detect_chips_in_pot(self):
        chips = self.node.get_ch().chips
        return sum([self.get_chip_value(chip) for chip in chips if self.in_pot(chip.coords)])
        
    def in_players_card_box(self, player, card_location):
        x, y, _ = card_location
        (x0, y0), (x1, y1) = player.card_box
        return x0 <= x and x <= x1 and y0 <= y and y <= y1

    def detect_fold(self, player):
        message = None
        while message is None:
            message = self.node.get_bc()
        if type(message) != str:
            back_of_cards = message.poses
            for card in back_of_cards:
                if self.in_players_card_box(player, card.coords):
                    return True
        return False
    
    def detect_curr_bettor(self):
        button = None
        closest_player = None

        while button is None:
            self.node.get_logger().info(f"looking")
            chip_message = self.node.get_ch()
            for chip in chip_message.chips:
                if chip.color == "red":
                    button = chip
                    break

        x, y, _ = button.coords
        button_point = np.array([x, y]).reshape(2, 1)

        closest_distance = np.inf
        for player in self.players:
            (x0, y0), (x1, y1) = player.chip_box
            point1 = np.array([x0, y0]).reshape(2, 1)
            point2 = np.array([x1, y1]).reshape(2, 1)

            curr_distance = min(np.linalg.norm(point1 - button_point),
                                np.linalg.norm(point2 - button_point))
            
            if curr_distance < closest_distance:
                closest_distance = curr_distance
                closest_player = player

        self.node.get_logger().info(f"player {closest_player.player_id} is betting...")
        return button, closest_player

    def run(self):
        num_bets = 0
        curr_button, curr_bettor = self.detect_curr_bettor()
        self.node.get_logger().info(f"player {curr_bettor.player_id} is betting...")
        curr_index = self.players.index(curr_bettor)

        curr_pot_size = self.detect_chips_in_pot()
        curr_min_bid = self.SMALL_BLIND if self.prev_state is "dealing" else 0

        while num_bets < len(self.players):

            new_button, new_bettor = self.detect_curr_bettor()
            if curr_bettor != new_bettor:

                new_pot_size = self.detect_chips_in_pot()
                new_bettor_is_next = new_bettor == self.players[(curr_index + 1) % len(self.players)]
                new_bet = new_pot_size - curr_pot_size
                bet_is_large_enough = new_bet >= curr_min_bid
                curr_bettor_has_folded = self.detect_fold(curr_bettor)
                if new_bettor_is_next and (bet_is_large_enough or curr_bettor_has_folded):
                    if curr_bettor_has_folded:
                        self.node.get_logger().info(f"player {curr_bettor.player_id} has folded")
                        self.players.remove(curr_bettor)
                    else:
                        self.node.get_logger().info(f"player {curr_bettor.player_id} bet {new_bet}")
                        if self.prev_state == "dealing" and num_bets == 0:
                            curr_min_bid = self.BIG_BLIND
                        else:
                            new_bet_is_raise = new_bet > curr_min_bid
                            if new_bet_is_raise:
                                self.node.get_logger().info(f"player {curr_bettor.player_id} raised to {new_bet} from {curr_min_bid}")
                                curr_min_bid = new_bet
                                num_bets = 0

                    
                    curr_bettor = new_bettor
                    curr_index = (curr_index + 1) % len(self.players)
                    num_bets += 1
                else:
                    self.node.act_at(np.array(new_button.coords).reshape(3, 1), 0, "GB_CHIP")
                    self.node.act_at(np.array(curr_button.coords).reshape(3, 1), 0, "DROP")


        if self.prev_state == "dealing":
            self.prev_state = "not dealing"

        self.num_rounds_of_betting += 1
        is_showdown = self.num_rounds_of_betting == 4
        return is_showdown, self.players
