import json
from detectors.message_types.Chip import Chip


class ChipMessage():
    def __init__(self, chips):
        self.chips = chips

    # @staticmethod
    # def from_chips_list(color_to_coords):
    #     chips = []
    #     for color, coords_list in color_to_coords.items():
    #         for coords in coords_list:
    #             chips.append(Chip(color, coords))

    #     return ChipMessage(chips)

    def to_string(self):
        string_chips = []
        for chip in self.chips:
            string_chips.append(chip.to_string())

        return json.dumps(string_chips)
    
    @staticmethod
    def from_string(str):
        chips = []
        string_chips = json.loads(str)
        for chip_string in string_chips:
            chips.append(Chip.from_string(chip_string))

        return ChipMessage(chips)



    