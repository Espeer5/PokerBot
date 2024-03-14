import json
import numpy as np
from detectors.message_types.CardPose import CardPose

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
        return Card(CardPose.from_string(pose_str), rank, suit)
    
    def __hash__(self):
        return 0
    
    def __eq__(self, other):
        assert isinstance(other, Card)
        
        self_coords = np.array(self.pose.coords)
        other_coords = np.array(other.pose.coords)

        return np.linalg.norm(self_coords - other_coords) <= 0.03



        