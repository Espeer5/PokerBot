class Player():

    def __init__(self, chip_box, card_box):
        self.chip_box = chip_box
        self.card_box = card_box
        self.cards = []
        self.stack = {"RED": 1, "BLACK": 1, "BLUE": 1}
