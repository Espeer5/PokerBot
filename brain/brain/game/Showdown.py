from pokereval.hand_evaluator import HandEvaluator


class Showdown():

    def __init__(self, node, players, community_cards):
        self.node = node
        self.players = players
        self.community_cards = community_cards

    def in_players_card_box(self, player, card_location):
        x, y = card_location
        (x0, y0), (x1, y1) = player.card_box
        return x0 <= x <= x1 and y0 <= y <= y1
    
    def translate_to_pokereval_format(self, rank, suit):
        rank = rank.lower()
        suit = suit.lower()

        face_card_to_number = {"ace": 14, "king": 13, "queen": 12, "jack": 11,
                               "ten": 10, "nine": 9, "eight": 8, "seven": 7,
                               "six": 6, "five": 5, "four": 4, "three": 3, "two": 2}
        rank = face_card_to_number[rank]

        suit_to_number = {"spades": 1, "hearts": 2, "diamonds": 3, "clubs": 4}
        suit = suit_to_number[suit]

        return rank, suit

    def run(self):
        players_to_cards = {}

        card_msg = self.node.get_foc()
        for card in card_msg.cards:
            for player in self.players:
                x, y, _ = card.pose.coords
                if self.in_players_card_box(player, (x, y)):
                    if player not in players_to_cards:
                        players_to_cards[player] = []
                    players_to_cards[player].append(self.translate_to_pokereval_format(card.rank, card.suit))

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
        