import numpy as np
import rclpy
import random


class Betting():
    LEFT_POT_LOCATION = [(-0.25, 0.20), (0.25, 0.50)]
    RIGHT_POT_LOCATION = [(None, None), (None, None)]
    SMALL_BLIND = 1
    BIG_BLIND = 2

    def __init__(self, node, active_players, chips_in_pot, is_first_round):
        node.get_logger().info("Betting initialized")
        self.node = node
        self.is_first_round = is_first_round
        self.players = active_players
        self.chips_in_pot = chips_in_pot
        self.num_rounds_of_betting = 0
    
    def in_pot(self, chip_location):
        x, y, _ = chip_location
        (x0, y0), (x1, y1) = self.POT_LOCATION
        return x0 <= x <= x1 and y0 <= y <= y1
    
    def get_chip_value(self, chip):
        if chip.color == "blue":
            return 1
        elif chip.color == "black":
            return 2
        else:
            return 0

    def detect_pot_size(self):
        chips = self.node.get_ch().chips
        return sum([self.get_chip_value(chip) for chip in chips if self.in_pot(chip.coords)])
        
    def in_players_card_box(self, player, card_location):
        x, y, _ = card_location
        (x0, y0), (x1, y1) = player.card_box
        return x0 <= x <= x1 and y0 <= y  <= y1
    
    def in_players_chip_box(self, player, chip_location):
        x, y, _ = chip_location
        (x0, y0), (x1, y1) = player.chip_box
        return x0 <= x <= x1 and y0 <= y  <= y1

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
            button_msg = self.node.get_btn()
            button = button_msg

        x, y, _ = button
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

        return button, closest_player
    
    def is_betting_over(self, player_bet_amounts, num_bets):
        if len(self.players) == 1:
            return True

        if num_bets < len(player_bet_amounts.keys()):
            return False

        curr_amount = player_bet_amounts[self.players[0]]
        for player, bet_amount in player_bet_amounts.items():
            if player in self.players:
                if bet_amount == 0 or bet_amount != curr_amount:
                    return False
        return True

    def find_space_in_pot(self, pot):
        (x0, y0), (x1, y1) = pot
        x = (x0 + x1) / 2
        y = (y0 + y1) / 2

        chips = self.node.get_ch().chips

        while True:
            curr_space = np.array([x, y]).reshape(2, 1)

            overlapping = False
            for chip in chips:
                cx, cy, _ = chip.coords
                chip_coords = np.array([cx, cy]).reshape(2, 1)

                if np.linalg.norm(curr_space - chip_coords) < 0.02:
                    overlapping = True
                    break

            if overlapping:
                curr_space = np.array([x + (random.random() - 1) * 0.08, y + (random.random() - 1) * 0.08]).reshape(3, 1)
            else:
                return curr_space
            
    def tidy_pot(self):
        chips = self.node.get_ch().chips
        messy_chips = []
        messy = True
        for chip in chips:
            if not self.in_pot(chip):
                for player in self.players:
                    if self.in_players_chip_box(player, chip.coords):
                        messy = False
                        break
                if messy:
                    messy_chips.append(chip)

        for chip in messy_chips:
            x, y, z = chip.coords
            if x < 0:
                tidy_location = self.find_space_in_pot(self.LEFT_POT_LOCATION)
            else:
                tidy_location = self.find_space_in_pot(self.RIGHT_POT_LOCATION)
            
            self.node.act_at(np.array([x, y, z]).reshape(3, 1), 0, "GB_CHIP")
            wait_ID = self.node.act_at(tidy_location, 0, "DROP")

            while self.node.prev_complete != wait_ID:
                rclpy.spin_once(self.node)


    def run(self):
        num_bets = 0
        curr_button, curr_bettor = self.detect_curr_bettor()
        self.node.get_logger().info(f"player {curr_bettor.player_id} is betting...")
        curr_index = self.players.index(curr_bettor)

        curr_pot_size = self.chips_in_pot
        curr_min_bet = self.SMALL_BLIND if self.is_first_round else 0

        player_bet_amounts = {player: 0 for player in self.players}

        while not self.is_betting_over(player_bet_amounts, num_bets):

            new_button, new_bettor = self.detect_curr_bettor()
            if curr_bettor != new_bettor:
                self.node.get_logger().info(f"player {new_bettor.player_id} is betting...")

                self.tidy_pot()
                new_pot_size = self.detect_pot_size()
                new_bettor_is_next = new_bettor == self.players[(curr_index + 1) % len(self.players)]
                new_bet = new_pot_size - curr_pot_size

                bet_is_large_enough = player_bet_amounts[curr_bettor] + new_bet >= curr_min_bet
                curr_bettor_has_folded = self.detect_fold(curr_bettor)
                if new_bettor_is_next and (bet_is_large_enough or curr_bettor_has_folded):
                    if not bet_is_large_enough and curr_bettor_has_folded:
                        self.node.get_logger().info(f"player {curr_bettor.player_id} has folded")
                        self.players.remove(curr_bettor)
                    else:
                        player_bet_amounts[curr_bettor] += new_bet
                        self.node.get_logger().info(f"player {curr_bettor.player_id} bet {new_bet}, total this round: {player_bet_amounts[curr_bettor]}")
                        if self.is_first_round and num_bets == 0:
                            curr_min_bet = self.BIG_BLIND
                        else:
                            new_bet_is_raise = new_bet > curr_min_bet
                            if new_bet_is_raise:
                                self.node.get_logger().info(f"player {curr_bettor.player_id} raised to {new_bet} from {curr_min_bet}")
                                curr_min_bet = new_bet

                    curr_pot_size = new_pot_size
                    curr_bettor = new_bettor
                    curr_button = new_button
                    curr_index = self.players.index(curr_bettor)
                    num_bets += 1
                else:
                    if not new_bettor_is_next:
                        self.node.get_logger().info(f"{new_bettor.player_id} is not next! {self.players[(curr_index + 1) % len(self.players)].player_id} is next!")

                    if not bet_is_large_enough:
                        self.node.get_logger().info(f"{new_bet} is smaller than minimum bet size {curr_min_bet}")
    
                    self.node.act_at(np.array(new_button).reshape(3, 1), 0, "GB_CHIP")
                    wait_ID = self.node.act_at(np.array(curr_button).reshape(3, 1), 0, "DROP")

                    while self.node.prev_complete != wait_ID:
                        rclpy.spin_once(self.node)
                        # pass

        self.node.get_logger().info(f"{player_bet_amounts}")

        self.num_rounds_of_betting += 1
        is_showdown = self.num_rounds_of_betting == 4
        return is_showdown, self.players, curr_pot_size
