class Payout():
    POT_LOCATION = [(None, None), (None, None)]

    def __init__(self, winner):
        self.winner = winner
        chips = None
        self.num_winners_chips = len([chip for chip in chips if self.in_winners_pot(chip)])
        self.num_chips_in_pot = len([chip for chip in chips if self.in_pot(chip)])

    def in_pot(self, chip_location):
        x, y = chip_location
        (x0, y0), (x1, y1) = self.POT_LOCATION
        return x0 <= x and x <= x1 and y0 <= y and y <= y1
    
    def in_winners_pot(self, chip_location):
        x, y = chip_location
        (x0, y0), (x1, y1) = self.winner.chip_box
        return x0 <= x and x <= x1 and y0 <= y and y <= y1

    def run(self):
        num_chips_in_pot = self.num_chips_in_pot
        while num_chips_in_pot > 0:
            chips = None
            num_chips_in_pot = len([chip for chip in chips if self.in_pot(chip)])
            # prompt user
            break

        num_winners_chips = self.num_winners_chips
        while num_winners_chips > 0:
            chips = None
            num_winners_chips = len([chip for chip in chips if self.in_winners_pot(chip)])
            # prompt user
            break
