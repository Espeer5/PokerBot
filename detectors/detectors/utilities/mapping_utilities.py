import cv2
import numpy as np

import pickle
from utils.TransformHelpers import Rotz


# Pixel Conversion
def pixelToWorld(image, u, v, x0, y0, annotateImage=False):
    # DX = 50
    # DY = 63

    '''
    Convert the (u,v) pixel position into (x,y) world coordinates
    Inputs:
        image: The image as seen by the camera
        u:     The horizontal (column) pixel coordinate
        v:     The vertical (row) pixel coordinate
        x0:    The x world coordinate in the center of the marker paper
        y0:    The y world coordinate in the center of the marker paper
        annotateImage: Annotate the image with the marker information

    Outputs:
        point: The (x,y) world coordinates matching (u,v), or None

    Return None for the point if not all the Aruco markers are detected
    '''

    # Detect the Aruco markers (using the 4X4 dictionary).
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(
        image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    if annotateImage:
        # if markerIds is None:
        cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)
        cv2.imshow("image", image)
        cv2.waitKey(0)

    # Abort if not all markers are detected.

    # if (markerIds is None or len(markerIds) != 4 or
    #     set(markerIds.flatten()) != set([1,2,3,4])):
    if markerIds is None or not set([1,2,3,4]).issubset(set(markerIds.flatten())):
        print(markerIds)
        print("OBSCURED MARKERS")
        return None
    

    newMarkerCorners = []
    newMarkerIds = []
    for corner, id in zip(markerCorners, markerIds):
        if id in [1, 2, 3, 4]:
            newMarkerCorners.append(corner)
            newMarkerIds.append(id)

    markerCorners = newMarkerCorners
    markerIds = newMarkerIds


    # Determine the center of the marker pixel coordinates.
    uvMarkers = np.zeros((4,2), dtype='float32')
    for i in range(4):
        uvMarkers[markerIds[i]-1,:] = np.mean(markerCorners[i], axis=1)

    # Calculate the matching World coordinates of the 4 Aruco markers.
    DX = 0.469
    DY = 0.2975
    xyMarkers = np.float32([[x0+dx, y0+dy] for (dx, dy) in
                            [(-DX, DY), (DX, DY), (-DX, -DY), (DX, -DY)]])


    # Create the perspective transform.
    M = cv2.getPerspectiveTransform(uvMarkers, xyMarkers)

    # Save the perspectiveTransform
    with open("p_trans.pkl", "wb") as f:
        pickle.dump(M, f)

    # Map the object in question.
    uvObj = np.float32([u, v])
    xyObj = cv2.perspectiveTransform(uvObj.reshape(1,1,2), M).reshape(2)
    xyObj = xyObj + np.float32([-0.015, -0.02])
    assert xyObj is not None


    # Mark the detected coordinates.
    if annotateImage:
        # cv2.circle(image, (u, v), 5, (0, 0, 0), -1)
        s = "(%7.4f, %7.4f)" % (xyObj[0], xyObj[1])
        cv2.putText(image, s, (u-80, v-8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 0), 2, cv2.LINE_AA)

    return xyObj


def pixel_to_world_2(image, u, v):
    """
    Convert the u,v image pixel coordinates to x,y world coordinates using 
    a single Aruco marker with a saved mapping from an earlier transformation 
    saved as a pickle file.
    """
    with open("p_trans.pkl", "rb") as f:
        transform = pickle.load(f)

    # Detect the single Aruco marker on the table
    markerCorners, markerIds, _ = cv2.aruco.detectMarkers(
        image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    
    # Abort if the marker is not detected
    if markerIds is None or 4 not in markerIds:
        print("MARKER OBSCURED")
        return None
    
    
    # Determine the center of the marker pixel coordinates
    uvMarker = np.mean(markerCorners[0], axis=1)

    x = np.array([1, 0])
    y = np.array([0, 1])
    x_p = markerCorners[0][0][1] - markerCorners[0][0][0]
    y_p = markerCorners[0][0][2] - markerCorners[0][0][1]
    alpha_x = np.arccos(np.dot(x_p, x) / np.linalg.norm(x_p) / np.linalg.norm(x))
    alpha_y = np.arccos(np.dot(y_p, y) / np.linalg.norm(y_p) / np.linalg.norm(y))
    alpha = np.mean([alpha_x, alpha_y])
    # print(f"FuCkIn {alpha}")

    # This marker is at a known location in the world
    xyMarker = np.array([0.469, 0.08])

    # Calculate the offset from the marker to the perspective transformed position
    # of the marker
    trans_marker = cv2.perspectiveTransform(uvMarker.reshape(1,1,2), transform).reshape(2)

    # Now use this offset to get the world coordinates of the object
    uvObj = np.float32([u, v])
    xyObj = cv2.perspectiveTransform(uvObj.reshape(1,1,2), transform).reshape(2)
    mag_xy = np.linalg.norm(xyObj) * (-1 if xyObj[0] < 0 else 1)
    alpha = alpha / mag_xy / 30
    R = np.array([[np.cos(alpha), -np.sin(alpha)], [np.sin(alpha), np.cos(alpha)]])
    R_t = np.transpose(R)
    xyObj = (R_t @ (xyObj - trans_marker)) + xyMarker
    x_offs = 0.0
    y_offs = 0.0
    if mag_xy > 0 and xyObj[1] > 0.3:
        x_offs = -0.01
        y_offs = 0.005
    elif mag_xy < 0 and xyObj[1] > 0.3:
        x_offs = -0.019
    if mag_xy < 0:
        y_offs = -0.007
    xyObj = xyObj + np.float32([x_offs, y_offs])
    assert xyObj is not None

    return xyObj