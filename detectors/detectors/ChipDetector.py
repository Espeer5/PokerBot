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
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image
from std_srvs.srv       import Trigger
from detectors.utilities.base_node import Detector
from detectors.utilities.chip_utilities import *
from detectors.utilities.mapping_utilities import pixelToWorld
from detectors.message_types.ChipMessage import ChipMessage


#
#  Detector Node Class
#
class ChipDetectorNode(Detector):
    # Pick some colors, assuming RGB8 encoding.
    red    = (255,   0,   0)
    green  = (  0, 255,   0)
    blue   = (  0,   0, 255)
    yellow = (255, 255,   0)
    white  = (255, 255, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        load_chip_descriptors_from_json()

        # Provice the /ch_detector service for the brain node to request the 
        # locations of all chips showing
        self.ch_service = self.create_service(Trigger, '/ch_detector', self.ch_callback)

        # Report.
        self.get_logger().info("ChipDetector running...")

    # Process the image (detect the ball).
    def ch_callback(self, _, response):
        if self.prev_img is None:
            response.message = "No image available"
            response.success = False
            return response
        response.success = True

        # Ensure the previous image is able to be processed
        assert(self.prev_img.encoding == "rgb8")  # lies

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(self.prev_img, "bgr8")

        red, white, blue, black = preprocess_image(frame)

        red_contours = find_chips(frame, red, "red")
        white_contours = find_chips(frame, white, "white")
        blue_contours = find_chips(frame, blue, "blue")
        black_contours = find_chips(frame, black, "black")

        self.get_logger().info(f"{len(red_contours)}, {len(white_contours)}, {len(blue_contours)}, {len(black_contours)}")
        contours = red_contours + white_contours + blue_contours + black_contours
        cv2.drawContours(frame, contours, -1, (0, 0, 255), 3)
        cv2.imshow("contours", frame)
        cv2.waitKey(0)

        def get_coords_from_contours(contours):
            coords = []
            for contour in contours:
                (u, v), _ = cv2.minEnclosingCircle(contour)
                world_coords = pixelToWorld(frame, round(u), round(v), 0.0, 0.34, annotateImage=False)
                if world_coords is not None:
                    coords.append((float(world_coords[0]), float(world_coords[1]), float(0)))
            return coords
        
        color_to_coords_map = {}
        for color, contours in [("red", red_contours), ("white", white_contours),
                                ("blue", blue_contours), ("black", black_contours)]:
            color_to_coords_map[color] = get_coords_from_contours(contours)

        # self.get_logger().info(f"color_to_coords_map= {color_to_coords_map}")
        response.message = ChipMessage.from_color_to_coords_map(color_to_coords_map).to_string()
        # send answer
        return response


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = ChipDetectorNode('ChipDetectorNode')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
