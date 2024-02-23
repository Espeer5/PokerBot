import json

class CardPose():

    def __init__(self, coords, theta):
        self.coords = coords
        self.theta = theta

    def to_string(self):
        x, y, z = self.coords
        return json.dumps([x, y, z, self.theta])
    
    @staticmethod
    def from_string(str):
        print(str)
        x, y, z, theta = json.loads(str)
        return CardPose((x, y, z), theta)