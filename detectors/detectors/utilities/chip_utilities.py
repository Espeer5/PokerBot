import cv2
import json
import numpy as np
from detectors.utilities.card_utilities import extract_card_from_image
from ament_index_python.packages import get_package_share_directory as pkgdir


CHIP_MIN_SIZE = 50
CHIP_MAX_SIZE = 400
CHIP_DESCRIPTORS_MAP = {}

BKG_THRESH = 60


def load_chip_descriptors_from_json():
    global CHIP_DESCRIPTORS_MAP
    json_file = open(f"{pkgdir('detectors')}/card_features/ChipDescriptors.json", "r")
    descriptors_dict = json.load(json_file)
    for color, descriptors in descriptors_dict.items():
        descriptors_dict[color] = np.array(descriptors, dtype=object).astype('uint8')
    CHIP_DESCRIPTORS_MAP = descriptors_dict
    json_file.close()


def preprocess_image(image):
    """Returns a grayed, blurred, and adaptively thresholded camera image."""
    image = cv2.GaussianBlur(image,(5,5),0)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    red_range = np.array([[0, 15], [65, 200], [90, 210]])
    white_range = np.array([[98, 120], [5, 60], [150, 255]])
    blue_range = np.array([[100, 125], [60, 185], [100, 200]])
    black_range = np.array([[100, 120], [0, 255], [0, 150]])

    def threshold_and_process(image, color_range, erode=0, dilate=0):
        color = cv2.inRange(image, color_range[:,0], color_range[:,1])

        morph_ellipse = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))
        if dilate > 0:
            color = cv2.dilate(color, morph_ellipse, iterations=dilate)

        if erode > 0:
            color = cv2.erode(color, morph_ellipse, iterations=erode)
        return color

    red = threshold_and_process(image, red_range, erode=5)
    white = threshold_and_process(image, white_range, erode=5)
    blue = threshold_and_process(image, blue_range, erode=5)
    black = threshold_and_process(image, black_range, erode=5)

    return red, white, blue, black


def find_chips(image, thresh_image, color):
    """Finds all card-sized contours in a thresholded camera image."""
    contours, _ = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return []

    def is_chip(contour):
        _, r = cv2.minEnclosingCircle(contour)
        circle_size = np.pi * r**2
        size = cv2.contourArea(contour)

        if size <= CHIP_MAX_SIZE:
            chip_image = extract_chip_from_image(image, contour)
            if len(chip_image) > 0:
                # reference = cv2.imread(f"{pkgdir('detectors')}/card_images/BlackChip.jpg")
                # reference = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
                # reference = cv2.resize(reference, (300, 300), interpolation=cv2.INTER_LINEAR)
                # cv2.imshow("reference", reference)
                # cv2.waitKey(0)
                # reference = cv2.GaussianBlur(reference,(9,9),0)

                # ORB = cv2.ORB_create(fastThreshold=0, edgeThreshold=0)
                ORB = cv2.ORB_create(fastThreshold=0)
                BF = cv2.BFMatcher_create(cv2.NORM_HAMMING,crossCheck=True)

                keypoints1, descriptors1 = ORB.detectAndCompute(chip_image, None)
                # keypoints2, descriptors2 = ORB.detectAndCompute(reference, None)
                reference_descriptors = CHIP_DESCRIPTORS_MAP[color]
                # keypoints2, CHIP_DESCRIPTORS = ORB.detectAndCompute(reference_image, None)

                matches = BF.match(descriptors1, reference_descriptors)
            
                # matching_result = cv2.drawMatches(chip_image, keypoints1, reference, keypoints2, matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                # cv2.imshow("matches", matching_result)
                # cv2.waitKey(0)
                # print(len(matches))
                print(len(matches))
                # if len(matches) >= 50:
                    # cv2.imshow("chip", chip_image)
                    # cv2.waitKey(0)
                #     cv2.imwrite(f"{pkgdir('detectors')}/card_images/RedChip.jpg", chip_image)
                if color == "black":
                    return len(matches) >= 120
                else:
                    return len(matches) >= 100

    return [contour for contour in contours if is_chip(contour)]


def extract_chip_from_image(image, contour):
    """Flattens an image of a card into a top-down 200x300 perspective.
    Returns the flattened, re-sized, grayed image.
    See www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/"""
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    (x, y), _ = cv2.minEnclosingCircle(contour)
    r = 18

    image = image[round(y - r): round(y + r), round(x - r): round(x + r)]

    if len(image) > 0 and len(image[0]) > 0:
        image = cv2.resize(image, (300, 300), interpolation=cv2.INTER_LINEAR)

    return image


def apply_watershed_algorithm(og_image, image, dx, dy):
    if len(image) > 0 and len(image[0]) > 0:
        kernel = np.ones((3,3),np.uint8)
        morph_ellipse = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))

        image = cv2.erode(image, morph_ellipse, iterations=3)

        # Finding sure background area
        sure_bg = cv2.dilate(image, kernel, iterations=3)

        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(image, cv2.DIST_L2,5)
        ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)

        # Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg,sure_fg)

        # Marker labelling
        ret, markers = cv2.connectedComponents(sure_fg)
        # Add one to all labels so that sure background is not 0, but 1
        markers = markers+1
        # Now, mark the region of unknown with zero
        markers[unknown==255] = 0

        markers = cv2.watershed(og_image, markers)
        og_image[markers == -1] = [255,0,0]

        labels = np.unique(markers)

        chips = []
        for label in labels[2:]:  
        
        # Create a binary image in which only the area of the label is in the foreground 
        #and the rest of the image is in the background   
            target = np.where(markers == label, 255, 0).astype(np.uint8)
        
        # Perform contour extraction on the created binary image
            contours, hierarchy = cv2.findContours(
                target, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            chips.append((contours[0], hierarchy[0][0]))

        # print(type(chips))
        for contour in chips:
            for coords in contour[0]:
                coords[0] += [dx, dy]
                
        return chips
    else:
        return []


# color_to_descriptors = {}
# for color in ["Red", "White", "Blue", "Black"]:
#     filename = color + "Chip.jpg"
#     image = cv2.imread("references/card_images/" + filename)
#     image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     image = cv2.resize(image, (300, 300), interpolation=cv2.INTER_LINEAR)

#     ORB = cv2.ORB_create(fastThreshold=0)

#     keypoints, descriptors = ORB.detectAndCompute(image, None)

#     color_to_descriptors[color.lower()] = descriptors.tolist()

# file = open("references/ChipDescriptors.json", "w")
# file.write(json.dumps(color_to_descriptors))
# file.close()


    