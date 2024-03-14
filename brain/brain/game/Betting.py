import numpy as np
import rclpy
import random

from brain.game.constants import PLAYERS, get_card_locations_from_card_box
from brain.game.psolve.model.decision import Agent
from brain.game.psolve.model.representation.hand import Hand
from brain.game.psolve.model.representation.card_set import CardSet, Card
from detectors.message_types.Chip import Chip


class Betting():
    LEFT_POT_LOCATION = [(-0.40, 0.34), (-0.08, 0.58)]
    RIGHT_POT_LOCATION = [(0.08, 0.34), (0.32, 0.58)]
    SMALL_BLIND = 1
    BIG_BLIND = 2
    ROBOT_BLUE_CHIP_LOCATION = (0.40, 0.35, 0.0)
    ROBOT_RED_CHIP_LOCATION = (0.40, 0.42, 0.0)
    PLAYER_TO_BUTTON_LOCATION = {1: (-0.36, 0.42, 0.0), 2: (-0.15, 0.58, 0.0),
                                 3: (0.23, 0.55, 0.0), "robot": (0.34, 0.38, 0.0)}

    def __init__(self, node, active_players, chips_in_pot, round_number, ccards):
        node.get_logger().info("Betting initialized")
        self.node = node
        self.is_first_round = round_number == 1
        self.players = active_players
        self.chips_in_pot = chips_in_pot
        self.num_rounds_of_betting = round_number
        self.p_cards = self.node.get_bot_foc()
        while "back" in [card.rank for card in self.p_cards.cards]:
            self.p_cards = self.node.get_bot_foc()
        self.c_cards = ccards
    
    def in_pot(self, chip_location):
        x, y, _ = chip_location
        (x0l, y0l), (x1l, y1l) = self.LEFT_POT_LOCATION
        (x0r, y0r), (x1r, y1r) = self.RIGHT_POT_LOCATION
        return (x0l <= x <= x1l and y0l <= y <= y1l) or (x0r <= x <= x1r and y0r <= y <= y1r)
    
    def get_chip_value(self, chip):
        if chip.color == "blue":
            return 1
        elif chip.color == "red":
            return 2
        else:
            raise "invalid chip color"

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
        return x0 <= x <= x1 and y0 <= y <= y1

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
                if bet_amount != curr_amount:
                    return False
        return True

    def find_space_in_pot(self, pot, tidy_locations):
        (x0, y0), (x1, y1) = pot
        x = (x0 + x1) / 2
        y = (y0 + y1) / 2

        dx = (x1 - x0) / 2
        dy = (y1 - y0) / 2

        curr_space = np.array([x, y])
        chips = self.node.get_ch().chips + tidy_locations

        scale = 0.75
        while True:
            overlapping = False
            for chip in chips:
                cx, cy, _ = chip.coords
                chip_coords = np.array([cx, cy])

                if np.linalg.norm(curr_space - chip_coords) <= 0.042:
                    overlapping = True
                    break

            if overlapping:
                curr_space = np.array([x + (random.random() - 1) * dx * (1 - scale), y + (random.random() - 1) * dy * (1 - scale)])
                scale *= scale
                self.node.get_logger().info(f"new location to try = {curr_space}")
            else:
                return np.array([curr_space[0], curr_space[1], -0.02])
            
    def tidy_pot(self):
        messy_chips = []
        iters = 0

        while len(messy_chips) > 0 or iters == 0:
            chips = self.node.get_ch().chips
            messy_chips = []
            iters += 1
            for chip in chips:
                if not self.in_pot(chip.coords):
                    messy = True
                    for player in PLAYERS:
                        if self.in_players_chip_box(player, chip.coords):
                            messy = False
                            break
                    if messy:
                        messy_chips.append(chip)

            self.node.get_logger().info(f"messy_chips={[chip.coords for chip in messy_chips]}")
            wait_ID = None
            tidy_locations = []
            for chip in messy_chips:
                x, y, z = chip.coords
                if x < 0:
                    tidy_location = self.find_space_in_pot(self.LEFT_POT_LOCATION, tidy_locations)
                    x1, y1, z1 = tidy_location
                    tidy_locations.append(Chip("none", (x1, y1, z1)))
                else:
                    tidy_location = self.find_space_in_pot(self.RIGHT_POT_LOCATION, tidy_locations)
                    x1, y1, z1 = tidy_location
                    tidy_locations.append(Chip("none", (x1, y1, z1)))
                
                wait = chip == messy_chips[-1]
                self.node.act_at(np.array([x, y, z]).reshape(3, 1), 0, "GB_CHIP")
                wait_ID = self.node.act_at(tidy_location.reshape(3, 1), 0, "DROP", wait=wait)

            while wait_ID is not None and self.node.prev_complete != wait_ID:
                rclpy.spin_once(self.node)

    def choose_bet(self, min_bet):
        """
        Decide how much to bet
        """
        hole_ranks = [card.rank.lower() for card in self.p_cards.cards]
        self.node.get_logger().info(f"{hole_ranks}")
        hole_suits = [card.suit.lower() for card in self.p_cards.cards]
        hole_cards = CardSet([Card(hole_ranks[i], hole_suits[i]) for i in range(len(hole_ranks))])
        c_ranks = [card[0].lower() for card in self.c_cards]
        c_suits = [card[1].lower() for card in self.c_cards]
        comm_cards = CardSet([Card(c_ranks[i], c_suits[i]) for i in range(len(c_ranks))])
        agent = Agent(Hand(hole_cards, comm_cards), 10)
        act = agent.make_decision()
        if act == 'call':
            return min_bet
        if act == 'fold':
            return -1
        return min_bet + random.randint(1, 3)

    @staticmethod
    def chips_from_bet(bet_amnt):
        """
        Given a bet amount, determines how many of each color of chip the robot
        should play.
        """
        if bet_amnt == -1:
            return {"red": 0, "blue": 0, "fold": 1}
        else:
            num_red = bet_amnt // 2
            num_blue = bet_amnt % 2
            return {"red": num_red, "blue": num_blue, "fold": 0}
        
    def make_robot_bets(self, curr_pot_size, bet_dict):
        if bet_dict["fold"]:
            fold_location = (0.25, 0.08, 0.0)
            coords1, coords2, theta = get_card_locations_from_card_box(PLAYERS[-1].card_box, raised=True)
            
            self.node.act_at(np.array(coords1).reshape(3, 1), theta, "GB_CARD")
            self.node.act_at(np.array(fold_location).reshape(3, 1), 0, "DROP")

            self.node.act_at(np.array(coords2).reshape(3, 1), theta, "GB_CARD")
            wait_ID = self.node.act_at(np.array(fold_location).reshape(3, 1), 0, "DROP", wait=True)

            while self.node.prev_complete != wait_ID:
                rclpy.spin_once(self.node)
        else:
            new_pot_size = self.detect_pot_size()
            bet_amount = 2 * bet_dict["red"] + bet_dict["blue"]
            goal_pot_size = curr_pot_size + bet_amount
            while new_pot_size < goal_pot_size:
                tidy_locations = []
                for i in range(bet_dict["red"]):
                    pot = self.LEFT_POT_LOCATION if random.random() < 0.5 else self.RIGHT_POT_LOCATION
                    tidy_location = self.find_space_in_pot(pot, tidy_locations)
                    x1, y1, z1 = tidy_location
                    tidy_locations.append(Chip("none", (x1, y1, z1)))
                    self.node.act_at(np.array(self.ROBOT_RED_CHIP_LOCATION).reshape(3, 1), 0, "GB_CHIP")

                    wait = i == bet_dict["red"] - 1 and bet_dict["blue"] == 0
                    wait_ID = self.node.act_at(np.array(tidy_location).reshape(3, 1), 0, "DROP", wait=wait)

                for j in range(bet_dict["blue"]):
                    pot = self.LEFT_POT_LOCATION if random.random() < 0.5 else self.RIGHT_POT_LOCATION
                    tidy_location = self.find_space_in_pot(pot, tidy_locations)
                    x1, y1, z1 = tidy_location
                    tidy_locations.append(Chip("none", (x1, y1, z1)))
                    self.node.act_at(np.array(self.ROBOT_BLUE_CHIP_LOCATION).reshape(3, 1), 0, "GB_CHIP")

                    wait = j == bet_dict["blue"] - 1
                    wait_ID = self.node.act_at(np.array(tidy_location).reshape(3, 1), 0, "DROP", wait=wait)
            
                while self.node.prev_complete != wait_ID:
                    rclpy.spin_once(self.node)

                new_pot_size = self.detect_pot_size()
                bet_dict = self.chips_from_bet(goal_pot_size - curr_pot_size)



    def run(self):
        num_bets = 0
        curr_button, curr_bettor = self.detect_curr_bettor()
        first_bettor = curr_bettor
        new_button = None
        self.node.get_logger().info(f"player {curr_bettor.player_id} is betting...")
        curr_index = self.players.index(curr_bettor)

        curr_pot_size = self.chips_in_pot
        curr_min_bet = self.SMALL_BLIND if self.is_first_round else 0

        player_bet_amounts = {player: 0 for player in self.players}

        while not self.is_betting_over(player_bet_amounts, num_bets):

            if curr_bettor.player_id == "robot":
                bet_amount = self.choose_bet(curr_min_bet)
                chips_map = self.chips_from_bet(bet_amount)
                self.make_robot_bets(curr_pot_size, chips_map)

                correct_next = self.players[(curr_index + 1) % len(self.players)].player_id

                new_button_loc = self.PLAYER_TO_BUTTON_LOCATION[correct_next]
                old_button_loc = new_button if new_button is not None else curr_button
                self.node.act_at(np.array(old_button_loc).reshape(3, 1), 0, "GB_CHIP")
                wait_ID = self.node.act_at(np.array(new_button_loc).reshape(3, 1), 0, "DROP", wait=True)

                while self.node.prev_complete != wait_ID:
                    rclpy.spin_once(self.node)

            new_button, new_bettor = self.detect_curr_bettor()
            if curr_bettor != new_bettor:
                self.node.get_logger().info(f"player {new_bettor.player_id} is betting...")

                self.tidy_pot()
                new_pot_size = self.detect_pot_size()
                new_bettor_is_next = new_bettor == self.players[(curr_index + 1) % len(self.players)]
                new_bet = new_pot_size - curr_pot_size

                bet_is_large_enough = player_bet_amounts[curr_bettor] + new_bet >= curr_min_bet
                curr_bettor_has_folded = self.detect_fold(curr_bettor)
                if new_bettor_is_next and (bet_is_large_enough or (curr_bettor_has_folded and new_bet == 0)):
                    if not bet_is_large_enough and curr_bettor_has_folded:
                        self.node.get_logger().info(f"player {curr_bettor.player_id} has folded")
                        if curr_bettor is first_bettor:
                            first_bettor = new_bettor
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
                        correct_next = self.players[(curr_index + 1) % len(self.players)].player_id
                        self.node.get_logger().info(f"{new_bettor.player_id} is not next! {correct_next} is next!")

                        button_loc = self.PLAYER_TO_BUTTON_LOCATION[correct_next]
                        self.node.act_at(np.array(new_button).reshape(3, 1), 0, "GB_CHIP")
                        wait_ID = self.node.act_at(np.array(correct_next).reshape(3, 1), 0, "DROP", wait=True)

                        while self.node.prev_complete != wait_ID:
                            rclpy.spin_once(self.node)

                    if not bet_is_large_enough:
                        self.node.get_logger().info(f"{new_bet} is smaller than minimum bet size {curr_min_bet}")
    
                        self.node.act_at(np.array(new_button).reshape(3, 1), 0, "GB_CHIP")
                        wait_ID = self.node.act_at(np.array(curr_button).reshape(3, 1), 0, "DROP", wait=True)

                        while self.node.prev_complete != wait_ID:
                            rclpy.spin_once(self.node)

        self.node.get_logger().info(f"{player_bet_amounts}")

        button_loc = self.PLAYER_TO_BUTTON_LOCATION[first_bettor.player_id]
        self.node.act_at(np.array(new_button).reshape(3, 1), 0, "GB_CHIP")
        wait_ID = self.node.act_at(np.array(button_loc).reshape(3, 1), 0, "DROP", wait=True)

        while self.node.prev_complete != wait_ID:
            rclpy.spin_once(self.node)

        is_showdown = self.num_rounds_of_betting == 4
        return is_showdown, self.players, curr_pot_size
