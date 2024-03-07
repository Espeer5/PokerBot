import numpy as np


class Betting():
    POT_LOCATION = [(-0.25, 0.20), (0.25, 0.50)]
    SMALL_BLIND = 1
    BIG_BLIND = 2

    def __init__(self, node, active_players):
        node.get_logger().info("Betting initialized")
        self.node = node
        self.prev_state = "dealing"
        self.players = active_players
        self.chips_in_pot = self.detect_chips_in_pot()
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

    def detect_chips_in_pot(self):
        chips = self.node.get_ch().chips
        return sum([self.get_chip_value(chip) for chip in chips if self.in_pot(chip.coords)])
        
    def in_players_card_box(self, player, card_location):
        x, y, _ = card_location
        (x0, y0), (x1, y1) = player.card_box
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
    
    def is_betting_over(self, player_bet_amounts):
        if len(self.players) == 1:
            return True

        curr_amount = player_bet_amounts[self.players[0]]
        for player, bet_amount in player_bet_amounts.items():
            if player in self.players:
                if bet_amount == 0 or bet_amount != curr_amount:
                    return False
        return True

    def run(self):
        num_bets = 0
        curr_button, curr_bettor = self.detect_curr_bettor()
        self.node.get_logger().info(f"player {curr_bettor.player_id} is betting...")
        curr_index = self.players.index(curr_bettor)

        curr_pot_size = self.detect_chips_in_pot()
        curr_min_bet = self.SMALL_BLIND if self.prev_state is "dealing" else 0

        player_bet_amounts = {player: 0 for player in self.players}

        while not self.is_betting_over(player_bet_amounts):

            new_button, new_bettor = self.detect_curr_bettor()
            if curr_bettor != new_bettor:
                self.node.get_logger().info(f"player {new_bettor.player_id} is betting...")

                new_pot_size = self.detect_chips_in_pot()
                new_bettor_is_next = new_bettor == self.players[(curr_index + 1) % len(self.players)]
                new_bet = new_pot_size - curr_pot_size

                bet_is_large_enough = player_bet_amounts[curr_bettor] + new_bet >= curr_min_bet
                curr_bettor_has_folded = self.detect_fold(curr_bettor)
                if new_bettor_is_next and (bet_is_large_enough or curr_bettor_has_folded):
                    if curr_bettor_has_folded:
                        self.node.get_logger().info(f"player {curr_bettor.player_id} has folded")
                        self.players.remove(curr_bettor)
                    else:
                        player_bet_amounts[curr_bettor] += new_bet
                        self.node.get_logger().info(f"player {curr_bettor.player_id} bet {new_bet}, total this round: {player_bet_amounts[curr_bettor]}")
                        if self.prev_state == "dealing" and num_bets == 0:
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
    
                    # self.node.act_at(np.array(new_button).reshape(3, 1), 0, "GB_CHIP")
                    # self.node.act_at(np.array(curr_button).reshape(3, 1), 0, "DROP")

        self.node.get_logger().info(f"{player_bet_amounts}")

        if self.prev_state == "dealing":
            self.prev_state = "not dealing"

        self.num_rounds_of_betting += 1
        is_showdown = self.num_rounds_of_betting == 4
        return is_showdown, self.players
