from detectors.message_types.Card import Card
import json

class CardMessage():
    def __init__(self, cards):
        self.cards = [Card(pose, rank, suit) for pose, rank, suit in cards]

    def to_string(self):
        return json.dumps([card.to_string() for card in self.cards])
    
    @staticmethod
    def from_string(str):
        card_strings = json.loads(str)
        cards = [Card.from_string(card_str) for card_str in card_strings]
        card_tups = [(card.pose, card.rank, card.suit) for card in cards]
        return CardMessage(card_tups)

