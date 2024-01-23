#!/usr/bin/env python3
#
#   writearuco.py
#
#   Generate an ArUco marker (from a given dictionary).
#

# ROS Imports
import cv2


#
#   Dictionary Options
#
#      DICT_4X4_50
#      DICT_4X4_100
#      DICT_4X4_250
#      DICT_4X4_1000
#
#      DICT_5X5_50
#      DICT_5X5_100
#      DICT_5X5_250
#      DICT_5X5_1000
#
#      DICT_6X6_50
#      DICT_6X6_100
#      DICT_6X6_250
#      DICT_6X6_1000
#
#      DICT_7X7_50
#      DICT_7X7_100
#      DICT_7X7_250
#      DICT_7X7_1000
#
#   a) Choose size: 4x4, 5x5, 6x6, 7x7
#
#   b) The number of elements in a dictionary doesn't change the
#      markers.  It simply caps which ones will be detected.  I
#      presume smallers ones are thus more robust.
#
#   Also pick the image size.  If you want to be super picky, choose a
#   pixel size which is a multiple of the tag size plus the black
#   border, i.e.
#                  size = (N+2) * something   for an NxN marker
#

# Set the ArUco dictionary.  Here we choose 6X6.
dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)

# Create a marker, ID#07, 200x200 pixels.
markerimage = cv2.aruco.drawMarker(dict,  7, 200);
cv2.imwrite("marker07.png", markerimage);

# Create a marker, ID#18, 200x200 pixels.
markerimage = cv2.aruco.drawMarker(dict, 18, 200);
cv2.imwrite("marker18.png", markerimage);

# Create a marker, ID#23, 200x200 pixels.
markerimage = cv2.aruco.drawMarker(dict, 23, 200);
cv2.imwrite("marker23.png", markerimage);

# Create a marker, ID#41, 200x200 pixels.
markerimage = cv2.aruco.drawMarker(dict, 41, 200);
cv2.imwrite("marker41.png", markerimage);
