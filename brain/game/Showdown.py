from pokereval.hand_evaluator import HandEvaluator


class Showdown():

    def __init__(self, players, community_cards):
        self.players = players
        self.community_cards = community_cards

    def in_players_pot(self, player, card_location):
        x, y = card_location
        (x0, y0), (x1, y1) = player.card_box
        return x0 <= x and x <= x1 and y0 <= y and y <= y1

    def run(self):
        players_to_cards = {}

        cards = None
        for card in cards:
            for player in self.players:
                if self.in_players_pot(player, card.location):
                    if player not in players_to_cards:
                        players_to_cards[player] = []
                    players_to_cards[player].append(card)

        winning_player = None
        winning_score = 0

        for player in self.players:
            curr_score = HandEvaluator.evaluate_hand(players_to_cards[player], self.community_cards)

            if curr_score > winning_score:
                winning_score = curr_score
                winning_player = player

        return winning_player
        