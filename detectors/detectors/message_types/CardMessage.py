from detectors.message_types.CardPose import CardPose
import json

class CardMessage():
    
    class Card():

        def __init__(self, pose, rank, suit):
            self.pose = pose
            self.rank = rank
            self.suit = suit

        def to_string(self):
            fields_list = [self.pose.to_string(), self.rank, self.suit]
            return json.dumps(fields_list)
        
        @staticmethod
        def from_string(str):
            pose_str, rank, suit = json.loads(str)
            return CardMessage.Card(CardPose.from_string(pose_str), rank, suit)


    def __init__(self, cards):
        self.cards = [CardMessage.Card(pose, rank, suit) for pose, rank, suit in cards]

    def to_string(self):
        return json.dumps([card.to_string() for card in self.cards])
    
    @staticmethod
    def from_string(str):
        card_strings = json.loads(str)
        cards = [CardMessage.Card.from_string(card_str) for card_str in card_strings]
        card_tups = [(card.pose, card.rank, card.suit) for card in cards]
        return CardMessage(card_tups)
    
