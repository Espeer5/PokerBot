import json

class ChipMessage():

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
            return ChipMessage.Chip(color, coords)

    def __init__(self, chips):
        self.chips = chips
        
    @staticmethod
    def from_color_to_coords_map(color_to_coords):
        chips = []
        for color, coords_list in color_to_coords.items():
            for coords in coords_list:
                chips.append(ChipMessage.Chip(color, coords))

        return ChipMessage(chips)

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
            chips.append(ChipMessage.Chip.from_string(chip_string))

        return ChipMessage(chips)



    