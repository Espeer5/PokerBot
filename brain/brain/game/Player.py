class Player():

    def __init__(self, player_id, chip_box, card_box):
        self.player_id = player_id
        self.chip_box = chip_box
        self.card_box = card_box
        self.cards = []
        self.stack = {"RED": 1, "BLACK": 1, "BLUE": 1}

    def __eq__(self, other):
        assert isinstance(other, Player)
        return self.player_id == other.player_id
    
    def __lt__(self, other):
        assert isinstance(other, Player)
        if self.player_id == "robot":
            return False
        elif other.player_id == "robot":
            return True
        return self.player_id < other.player_id
