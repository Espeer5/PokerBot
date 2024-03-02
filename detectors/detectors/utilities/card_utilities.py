import numpy as np
import cv2
import json
from ament_index_python.packages import get_package_share_directory as pkgdir


# Adaptive threshold levels
BKG_THRESH = 60
CARD_THRESH = 30

# Card Area Bounds
CARD_MAX_AREA = 20000
CARD_MIN_AREA = 1000

# Style
FONT = cv2.FONT_HERSHEY_SIMPLEX
BACK_OF_CARD_DESCRIPTORS = np.array([])
CARD_DESCRIPTORS_MAP = np.array([])


# class QueryCard:
#     """Structure to store information about query cards in the camera image."""

#     def __init__(self):
#         # self.contour = [] # Contour of card
#         self.width, self.height = 0, 0 # Width and height of card
#         # self.corner_pts = [] # Corner points of card
#         self.center = [] # Center point of card
#         self.image = [] # 200x300, flattened, grayed, blurred image
#         self.rank = "Unknown"
#         self.suit = "Unknown"


class ReferenceFeatures:
    """Structure to store information about train rank images."""

    def __init__(self, name, features):
        self.name = name
        self.features = features

def preprocess_image(image):
    """Returns a grayed, blurred, and adaptively thresholded camera image."""

    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray,(5,5),0)

    # The best threshold level depends on the ambient lighting conditions.
    # For bright lighting, a high threshold must be used to isolate the cards
    # from the background. For dim lighting, a low threshold must be used.
    # To make the card detector independent of lighting conditions, the
    # following adaptive threshold method is used.
    #
    # A background pixel in the center top of the image is sampled to determine
    # its intensity. The adaptive threshold is set at 50 (THRESH_ADDER) higher
    # than that. This allows the threshold to adapt to the lighting conditions.
    # img_w, img_h = np.shape(image)[:2]
    # bkg_level = gray[img_w // 2][img_h // 2]
    bkg_level = 85
    thresh_level = bkg_level + BKG_THRESH

    retval, thresh = cv2.threshold(blur,thresh_level,255,cv2.THRESH_BINARY)
    
    return thresh


def find_cards(thresh_image):
    """Finds all card-sized contours in a thresholded camera image."""
    contours, hierarchies = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return np.array([])

    def is_contour_a_card(contour, hierarchy):
        size = cv2.contourArea(contour)
        peri = cv2.arcLength(contour, True)
        num_corners = len(cv2.approxPolyDP(contour, 0.01*peri, True))

        PARENT = 3
        NO_PARENT = -1
        return size < CARD_MAX_AREA and size > CARD_MIN_AREA and hierarchy[PARENT] == NO_PARENT and num_corners == 4

    card_contours = [contour for contour, hierarchy in zip(contours, hierarchies[0]) if is_contour_a_card(contour, hierarchy)]
    # card_contours.sort(key=cv2.contourArea, reverse=True)

    return card_contours


def extract_card_from_image(image, contour):
    """Flattens an image of a card into a top-down 200x300 perspective.
    Returns the flattened, re-sized, grayed image.
    See www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/"""
    rect = cv2.minAreaRect(contour)
    (x, y), (w, h), theta = rect
    if h/w < 1:
        theta += 90
        old_w = w
        w = h
        h = old_w

    shape = image.shape[1], image.shape[0] # cv2.warpAffine expects shape in (length, height)

    matrix = cv2.getRotationMatrix2D(center=(x, y), angle=theta, scale=1)
    image = cv2.warpAffine(src=image, M=matrix, dsize=shape)

    x = int(x - w/2)
    y = int(y - h/2)
    h = round(h)
    w = round(w)

    image = image[y:y+h, x:x+w]

    if len(image) > 0:
        image = cv2.resize(image, (200, 300),
                interpolation = cv2.INTER_LINEAR)

    return image


# def preprocess_card(contour, image):
#     """Uses contour to find information about the query card."""

#     qCard = QueryCard()

#     rect = cv2.minAreaRect(contour)
#     # box_points = cv2.boxPoints(rect)
#     (x, y), (w, h), _ = rect

#     qCard.width, qCard.height = w, h
#     qCard.center = [int(x), int(y)]

#     qCard.image = cv2.cvtColor(extract_card_from_image(image, rect), cv2.COLOR_BGR2GRAY)

#     return qCard


def identify_card(card_image):
    """Finds best rank and suit matches for the query card."""

    orb = cv2.ORB_create(fastThreshold=0, edgeThreshold=0)
    bf = cv2.BFMatcher_create(cv2.NORM_HAMMING,crossCheck=True)
    _, descriptors1 = orb.detectAndCompute(card_image, None)

    best_num_matches = 0
    for name, descriptors2 in CARD_DESCRIPTORS_MAP.items():
        matches = bf.match(descriptors1, descriptors2)

        if len(matches) > best_num_matches:
            best_num_matches = len(matches)
            best_name = name

    best_rank, best_suit = best_name.split("_of_")
    # print(best_num_matches)
    if best_num_matches < 50:
        return None, None
    return best_rank, best_suit


def is_back_of_card(card_image):
    ORB = cv2.ORB_create(fastThreshold=0, edgeThreshold=0)
    BF = cv2.BFMatcher_create(cv2.NORM_HAMMING,crossCheck=True)

    _, descriptors1 = ORB.detectAndCompute(card_image, None)

    matches = BF.match(descriptors1, BACK_OF_CARD_DESCRIPTORS)
    return len(matches) >= 140


def load_back_of_card_descriptors_from_json():
    global BACK_OF_CARD_DESCRIPTORS
    json_file = open(f"{pkgdir('detectors')}/card_features/BackOfCardDescriptors.json", "r")
    BACK_OF_CARD_DESCRIPTORS = np.array(json.load(json_file), dtype=object).astype('uint8')
    json_file.close()


def load_card_descriptors_map_from_json():
    global CARD_DESCRIPTORS_MAP
    json_file = open(f"{pkgdir('detectors')}/card_features/CardDescriptors.json", "r")
    descriptors_dict = json.load(json_file)
    for key in descriptors_dict:
        descriptors_dict[key] = np.array(descriptors_dict[key], dtype=object).astype('uint8')
    CARD_DESCRIPTORS_MAP = descriptors_dict
    json_file.close()


# def load_cards():
#     """Loads rank images from directory specified by filepath. Stores
#     them in a list of Train_ranks objects."""
    
#     ranks = ['Ace','Two','Three','Four','Five','Six','Seven','Eight',
#              'Nine','Ten','Jack','Queen','King']
#     suits = ['Spades','Diamonds',
#              'Clubs','Hearts']
    


#     orb = cv2.ORB_create(fastThreshold=0, edgeThreshold=0)

#     name_to_descriptors = {}

#     # train_cards = []
#     for rank in ranks:
#         for suit in suits:

#     #         # card_names.append(rank + "_of_" + suit)
#     #         train_cards.append(Train_ranks())
#             name = rank + "_of_" + suit

#             filename = rank + "_of_" + suit + '.jpg'
#             img = cv2.imread("references/card_images/"+filename, cv2.IMREAD_GRAYSCALE)
#             _, descriptors1 = orb.detectAndCompute(img, None)
#             name_to_descriptors[name] = descriptors1.tolist()

#     json_descriptors = json.dumps(name_to_descriptors)
#     file = open("references/CardDescriptors.json", "w")
#     file.write(json_descriptors)

#     # return train_cards


def draw_results(image, rank, suit, coords):
    """Draw the card name, center point, and contour on the camera image."""

    x, y = coords
    cv2.circle(image, (x,y),5,(255,0,0),-1)

    # Draw card name twice, so letters have black outline
    cv2.putText(image, (rank + ' of'), (x-60,y-10), FONT, 1, (0,0,0), 3, cv2.LINE_AA)
    cv2.putText(image, (rank + ' of'), (x-60,y-10), FONT, 1, (50,200,200), 2, cv2.LINE_AA)

    cv2.putText(image, suit, (x-60,y+25), FONT, 1, (0,0,0), 3, cv2.LINE_AA)
    cv2.putText(image, suit, (x-60,y+25), FONT, 1, (50,200,200), 2, cv2.LINE_AA)

    return image
