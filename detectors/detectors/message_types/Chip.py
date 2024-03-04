import json
import numpy as np


class Chip():
    def __init__(self, color, coords):
        self.color = color
        self.coords = coords

    def to_string(self):
        fields_list = [self.color, self.coords]
        return json.dumps(fields_list)

    @staticmethod
    def from_string(str):
        color, coords = json.loads(str)
        return Chip(color, coords)
    
    def __eq__(self, other):
        assert isinstance(other, Chip)
        color_is_equal = self.color == other.color
        coord_distance = np.linalg.norm(np.array(self.coords) - np.array(other.coords))
        return color_is_equal and coord_distance < 0.02
    
    def __hash__(self):
        return hash(self.color)
