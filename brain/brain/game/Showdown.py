from pokereval.hand_evaluator import HandEvaluator
from pokereval.card import Card
from brain.game.constants import PLAYERS, get_card_locations_from_card_box
from brain.game.constants import FLIP_LOC
import numpy as np
import rclpy


class Showdown():

    def __init__(self, node, players, community_cards):
        self.node = node
        self.players = players
        self.community_cards = community_cards

    def closest_player(self, card_location):
        card_location = np.array(card_location).reshape(2, 1)

        closest_distance = np.inf
        for player in self.players:
            (x0, y0), (x1, y1) = player.card_box
            point1 = np.array([x0, y0]).reshape(2, 1)
            point2 = np.array([x1, y1]).reshape(2, 1)

            curr_distance = min(np.linalg.norm(point1 - card_location),
                                np.linalg.norm(point2 - card_location))
            
            if curr_distance < closest_distance:
                closest_distance = curr_distance
                closest_player = player

        if closest_distance > 0.08:
            return None
        return closest_player
    
    def translate_to_pokereval_format(self, rank, suit):
        rank = rank.lower()
        suit = suit.lower()

        face_card_to_number = {"ace": 14, "king": 13, "queen": 12, "jack": 11,
                               "ten": 10, "nine": 9, "eight": 8, "seven": 7,
                               "six": 6, "five": 5, "four": 4, "three": 3, "two": 2}
        rank = face_card_to_number[rank]

        suit_to_number = {"spades": 1, "hearts": 2, "diamonds": 3, "clubs": 4}
        suit = suit_to_number[suit]

        return Card(rank, suit)
    
    def each_player_has_two_cards(self, players_to_cards):
        # self.node.get_logger().info(f"{players_to_cards}")
        
        for player, cards in players_to_cards.items():
            if len(cards) != 2:
                self.node.get_logger().info(f"Player {player.player_id} only has {len(cards)} cards")
                return False
            
        if len(players_to_cards.keys()) != len(self.players):
            self.node.get_logger().info(f"Not everyone has turned over their cards")
            return False
        
        return True

    def run(self):
        players_to_cards = {}

        robot_cards_msg = self.node.get_bot_foc()
        while robot_cards_msg is None or type(robot_cards_msg) is str or "Back" in [card.rank for card in robot_cards_msg.cards]:
            robot_cards_msg = self.node.get_bot_foc()
    
        players_to_cards[PLAYERS[-1]] = []
        self.node.get_logger().info("robot's cards")
        for card in robot_cards_msg.cards:
            self.node.get_logger().info(f"{card.rank}, {card.suit}")
            players_to_cards[PLAYERS[-1]].append(self.translate_to_pokereval_format(card.rank, card.suit))

        self.node.get_logger().info("players' cards")
        while not self.each_player_has_two_cards(players_to_cards):
            players_to_cards = {}
            card_msg = self.node.get_foc()
            if card_msg is not None:
                for card in card_msg.cards:
                    if card.rank != "Back" and card.suit != "Card":
                        x, y, _ = card.pose.coords
                        player = self.closest_player((x, y))
                        if player is not None:
                            if player not in players_to_cards:
                                players_to_cards[player] = []
                            self.node.get_logger().info(f"{card.rank}, {card.suit}")
                            players_to_cards[player].append(self.translate_to_pokereval_format(card.rank, card.suit))
            

        coords1, coords2, theta = get_card_locations_from_card_box(PLAYERS[-1].card_box, raised=True)
            
        self.node.act_at(np.array(coords1).reshape(3, 1), theta, "GB_CARD")
        self.node.act_at(FLIP_LOC, 0.0, "FLIP")

        self.node.act_at(np.array(coords2).reshape(3, 1), theta, "GB_CARD")
        wait_ID = self.node.act_at(FLIP_LOC, 0.0, "FLIP", wait=True)

        while self.node.prev_complete != wait_ID:
            rclpy.spin_once(self.node)

        winning_player = None
        winning_score = 0

        ccards = [self.translate_to_pokereval_format(rank, suit) for rank, suit in self.community_cards]
        self.node.get_logger().info(f"Community cards: {ccards}")

        self.node.get_logger().info(f"{players_to_cards}")

        for player, cards in players_to_cards.items():
            self.node.get_logger().info(f"Player {player.player_id}'s cards: {cards}")
            curr_score = HandEvaluator.evaluate_hand(cards, ccards)

            if curr_score > winning_score:
                winning_score = curr_score
                winning_player = player

        return winning_player
        