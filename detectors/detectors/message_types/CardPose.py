import json
import numpy as np


class CardPose():

    def __init__(self, coords, theta):
        self.coords = coords
        self.theta = theta

    def to_string(self):
        x, y, z = self.coords
        return json.dumps([x, y, z, self.theta])
    
    @staticmethod
    def from_string(str):
        x, y, z, theta = json.loads(str)
        return CardPose((x, y, z), theta)
    
    def __hash__(self):
        # x, y, z = self.coords
        return 0
    
    def __eq__(self, other):
        assert isinstance(other, CardPose)
        
        self_coords = np.array(self.coords)
        other_coords = np.array(other.coords)

        return np.linalg.norm(self_coords - other_coords) <= 0.06
