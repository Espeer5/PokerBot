import cv2
import numpy as np


CHIP_MIN_RADIUS = 10
CHIP_MAX_RADIUS = 16

BKG_THRESH = 60


def preprocess_image(image):
    """Returns a grayed, blurred, and adaptively thresholded camera image."""

    # gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    image = cv2.GaussianBlur(image,(5,5),0)

    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    # red_range = np.array([[2, 8], [98, 110], [138, 152]])
    red_range = np.array([[0, 15], [25, 170], [120, 180]])
    # white_range = np.array([[100, 110], [12, 20], [200, 215]])
    white_range = np.array([[98, 120], [5, 60], [180, 255]])
    # blue_range = np.array([[108, 117], [140, 160], [140, 155]])
    blue_range = np.array([[100, 125], [22, 165], [125, 200]])
    # black_range = np.array([[100, 120], [40, 60], [68, 82]])
    black_range = np.array([[100, 120], [8, 65], [50, 180]])
    
    def threshold_and_process(image, color_range):
        color = cv2.inRange(image, color_range[:,0], color_range[:,1])
        color = cv2.erode(color, None, iterations=3)
        color = cv2.dilate(color, None, iterations=2)
        return color
    
    red = threshold_and_process(hsv, red_range)
    white = threshold_and_process(hsv, white_range)
    blue = threshold_and_process(hsv, blue_range)
    black = threshold_and_process(hsv, black_range)

    return red, white, blue, black


def find_chips(thresh_image):
    """Finds all card-sized contours in a thresholded camera image."""
    contours, hierarchies = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return np.array([])

    def is_contour_a_chip(contour, hierarchy):
        size = cv2.contourArea(contour)
        _, r = cv2.minEnclosingCircle(contour)
        circle_size = np.pi * r**2

        # peri = cv2.arcLength(contour, True)
        # num_corners = len(cv2.approxPolyDP(contour, 0.01*peri, True))

        PARENT = 3
        NO_PARENT = -1
        AREA_THRESHOLD = 0.6
        return r > CHIP_MIN_RADIUS and r < CHIP_MAX_RADIUS and hierarchy[PARENT] == NO_PARENT and size / circle_size > AREA_THRESHOLD

    chip_contours = [contour for contour, hierarchy in zip(contours, hierarchies[0]) if is_contour_a_chip(contour, hierarchy)]
    # _contours.sort(key=cv2.contourArea, reverse=True)
    return chip_contours


# def extract_chip_from_image(image, contour):
#     """Flattens an image of a card into a top-down 200x300 perspective.
#     Returns the flattened, re-sized, grayed image.
#     See www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/"""
#     (x, y), r = cv2.minEnclosingCircle(contour)

#     image = image[round(y - r): round(y + r), round(x - r): round(x + r)]

#     if len(image) > 0:
#         image = cv2.resize(image, (100, 100),
#                 interpolation = cv2.INTER_LINEAR)

#     return image