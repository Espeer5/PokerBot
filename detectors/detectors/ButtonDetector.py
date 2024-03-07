#!/usr/bin/env python3
#
#   balldetector.py
#
#   Detect the tennis balls with OpenCV.
#
#   Node:           /balldetector
#   Subscribers:    /usb_cam/image_raw          Source image
#   Publishers:     /balldetector/binary        Intermediate binary image
#                   /balldetector/image_raw     Debug (marked up) image
#
import cv2
import numpy as np

# ROS Imports
import rclpy

from std_srvs.srv       import Trigger
from detectors.utilities.base_node import Detector
from detectors.utilities.chip_utilities import *
from detectors.utilities.mapping_utilities import pixelToWorld


#
#  Detector Node Class
#
class ButtonDetectorNode(Detector):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)
    black = (0, 0, 0)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        load_chip_descriptors_from_json()
        self.hsvlimits = np.array([[85, 95], [120, 190], [150, 215]])

        # Provice the /btn_detector service for the brain node to request the 
        # locations of all chips showing
        self.btn_service = self.create_service(Trigger, '/btn_detector', self.btn_callback)

        # Report.
        self.get_logger().info("ButtonDetector running...")

    # Process the image (detect the ball).
    def btn_callback(self, _, response):
        if self.prev_images is None:
            response.message = "No image available"
            response.success = False
            return response
        response.success = True

        image = self.prev_images[-1]
        # Ensure the previous image is able to be processed
        assert(image.encoding == "rgb8")  # lies

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        binary = cv2.inRange(hsv, self.hsvlimits[:,0], self.hsvlimits[:,1])

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # assert len(contours) == 1
        if len(contours) > 0:
            (u, v), _ = cv2.minEnclosingCircle(contours[0])

            world_coords = pixelToWorld(frame, round(u), round(v), 0.0, 0.34, annotateImage=False)

            if world_coords is not None:
                x, y = world_coords
                point = f"{x}, {y}, {0.0}"
                response.message = point
    
        return response


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = ButtonDetectorNode('ChipDetectorNode')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
