import cv2
import numpy as np
from detectors.utilities.card_utilities import extract_card_from_image


CHIP_MIN_RADIUS = 10
CHIP_MAX_RADIUS = 25

BKG_THRESH = 60


def preprocess_image(image):
    """Returns a grayed, blurred, and adaptively thresholded camera image."""
    # gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    image = cv2.GaussianBlur(image,(5,5),0)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    red_range = np.array([[0, 15], [0, 255], [120, 200]])
    white_range = np.array([[100, 120], [0, 255], [180, 255]])
    blue_range = np.array([[100, 125], [0, 165], [125, 170]])
    black_range = np.array([[100, 120], [8, 65], [50, 150]])



    def threshold_and_process(image, color_range, erode=0, dilate=0):
        color = cv2.inRange(image, color_range[:,0], color_range[:,1])
        # gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        # cv2.imshow("gray", gray)
        # ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        # cv2.imshow("thresh", thresh)
        # cv2.imshow("color", color)
        # cv2.waitKey(0)

        # # noise removal
        if dilate > 0:
            morph_ellipse = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(3, 3))

            # kernel = np.ones((3,3), np.uint8)
            # color = cv2.morphologyEx(color, cv2.MORPH_OPEN, kernel, iterations=2)
            color = cv2.dilate(color, morph_ellipse, iterations=dilate)
            # cv2.imshow("color", color)
            # cv2.waitKey(0)

        # cv2.imshow("opening", opening)
        # cv2.waitKey(0)
        # # sure background area
        # sure_bg = cv2.dilate(opening,kernel,iterations=3)
        # cv2.imshow("sure_bg", sure_bg)
        # # Finding sure foreground area
        # dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        # cv2.imshow("dist_transform", dist_transform)
        # ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
        # cv2.imshow("sure_fg", sure_fg)
        # # Finding unknown region
        # sure_fg = np.uint8(sure_fg)
        # unknown = cv2.subtract(sure_bg,sure_fg)
        # cv2.imshow("unknown", unknown)

        # # Marker labelling
        # ret, markers = cv2.connectedComponents(sure_fg)
        # # Add one to all labels so that sure background is not 0, but 1
        # markers = markers+1
        # # Now, mark the region of unknown with zero
        # markers[unknown==255] = 0

        # markers = cv2.watershed(image, markers)
        # image[markers == -1] = [255,0,0]

        # labels = np.unique(markers)
 
        # coins = []
        # for label in labels[2:]:  
        
        # # Create a binary image in which only the area of the label is in the foreground 
        # #and the rest of the image is in the background   
        #     target = np.where(markers == label, 255, 0).astype(np.uint8)
        
        # # Perform contour extraction on the created binary image
        #     contours, hierarchy = cv2.findContours(
        #         target, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        #     )
        #     coins.append(contours[0])
        
        # # Draw the outline
        # image = cv2.drawContours(image, coins, -1, color=(0, 0, 0), thickness=5)
        # cv2.imshow("image", image)
        # cv2.waitKey(0)
        # color = cv2.dilate(color, None, iterations=4)
        # color = cv2.erode(color, None, iterations=2)
        # cv2.imshow("markers", markers)
        # print("markers=", markers)
        # exit()
        # cv2.imshow("image", color)
        # cv2.waitKey(0)
        return color

    red = threshold_and_process(image, red_range, dilate=2)
    white = threshold_and_process(image, white_range, dilate=2)
    blue = threshold_and_process(image, blue_range, dilate=2)
    black = threshold_and_process(image, black_range, dilate=2)

    return red, white, blue, black


def find_chips(og_image, thresh_image):
    """Finds all card-sized contours in a thresholded camera image."""
    contours, hierarchies = cv2.findContours(thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        return []

    def get_chip_contours(contour, hierarchy, first_call=True):
        size = cv2.contourArea(contour)
        _, r = cv2.minEnclosingCircle(contour)
        circle_size = np.pi * r**2

        # peri = cv2.arcLength(contour, True)
        # num_corners = len(cv2.approxPolyDP(contour, 0.01*peri, True))

        PARENT = 3
        NO_PARENT = -1
        AREA_THRESHOLD = 0.7
        # cv2.circle(thresh_image, (round(u), round(v)), round(r), (255,0,0), 3)
        # cv2.imshow("thresh", thresh_image)
        # cv2.waitKey(0)
        # print("data=", r > CHIP_MIN_RADIUS, r < CHIP_MAX_RADIUS, hierarchy[PARENT] == NO_PARENT, size / circle_size > AREA_THRESHOLD)

        if r > CHIP_MIN_RADIUS and r < CHIP_MAX_RADIUS and hierarchy[PARENT] == NO_PARENT and size / circle_size > AREA_THRESHOLD:
            return [contour]
        elif r > 2 * CHIP_MIN_RADIUS and first_call:
            image, _, _ = extract_chip_from_image(thresh_image, contour)
            og_image2, dx, dy = extract_chip_from_image(og_image, contour)

            cv2.erode
            watershed_results = apply_watershed_algorithm(og_image2, image, dx, dy)
            # print("wsr=", len(watershed_results))
            chip_results = [get_chip_contours(contour, hierarchy, first_call=False) for contour, hierarchy in watershed_results]
            # print("cpr=", len([chip for chip in chip_results if len(chip) > 0]))
            return_list = [contour for contour_list in chip_results for contour in contour_list if len(contour) > 0]
            return return_list
        else:
            return []

    chip_contours = [get_chip_contours(contour, hierarchy) for contour, hierarchy in zip(contours, hierarchies[0])]
    return_list = [contour for contour_list in chip_contours for contour in contour_list if len(contour) > 0]
    return return_list


def extract_chip_from_image(image, contour):
    """Flattens an image of a card into a top-down 200x300 perspective.
    Returns the flattened, re-sized, grayed image.
    See www.pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/"""
    (x, y), r = cv2.minEnclosingCircle(contour)

    image = image[round(y - r): round(y + r), round(x - r): round(x + r)]

    if len(image) > 0 and len(image[0]) > 0:
        image = cv2.resize(image, (100, 100), interpolation=cv2.INTER_LINEAR)

    return image, round(x), round(y)


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
        