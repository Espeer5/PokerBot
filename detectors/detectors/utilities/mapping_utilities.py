import cv2
import numpy as np


# Pixel Conversion
def pixelToWorld(image, u, v, x0, y0, annotateImage=True):
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
        cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)

    # Abort if not all markers are detected.
    if (markerIds is None or len(markerIds) != 4 or
        set(markerIds.flatten()) != set([1,2,3,4])):
        return None


    # Determine the center of the marker pixel coordinates.
    uvMarkers = np.zeros((4,2), dtype='float32')
    for i in range(4):
        uvMarkers[markerIds[i]-1,:] = np.mean(markerCorners[i], axis=1)

    # Calculate the matching World coordinates of the 4 Aruco markers.
    DX = 0.5
    DY = 0.315
    xyMarkers = np.float32([[x0+dx, y0+dy] for (dx, dy) in
                            [(-DX, DY), (DX, DY), (-DX, -DY), (DX, -DY)]])


    # Create the perspective transform.
    M = cv2.getPerspectiveTransform(uvMarkers, xyMarkers)

    # Map the object in question.
    uvObj = np.float32([u, v])
    xyObj = cv2.perspectiveTransform(uvObj.reshape(1,1,2), M).reshape(2)


    # Mark the detected coordinates.
    if annotateImage:
        # cv2.circle(image, (u, v), 5, (0, 0, 0), -1)
        s = "(%7.4f, %7.4f)" % (xyObj[0], xyObj[1])
        cv2.putText(image, s, (u-80, v-8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 0), 2, cv2.LINE_AA)

    return xyObj