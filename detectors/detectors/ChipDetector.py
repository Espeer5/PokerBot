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
from detectors.message_types.Chip import Chip


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
    black = (0, 0, 0)

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
        if self.prev_images is None:
            response.message = "No image available"
            response.success = False
            return response
        response.success = True



        chip_to_coords_map = {}
        chip_to_contour_map = {}

        for image in self.prev_images:
            # Ensure the previous image is able to be processed
            assert(image.encoding == "rgb8")  # lies

            # Convert into OpenCV image, using RGB 8-bit (pass-through).
            frame = self.bridge.imgmsg_to_cv2(image, "bgr8")

            red, white, blue, black = preprocess_image(frame)

            red_contours = find_chips(frame, red, "red")
            white_contours = find_chips(frame, white, "white")
            blue_contours = find_chips(frame, blue, "blue")
            black_contours = find_chips(frame, black, "black")

            # self.get_logger().info(f"{len(red_contours)}, {len(white_contours)}, {len(blue_contours)}, {len(black_contours)}")
            # contours = red_contours + white_contours + blue_contours + black_contours
            # cv2.drawContours(frame, contours, -1, (0, 0, 255), 3)
            # cv2.imshow("contours", frame)
            # cv2.waitKey(0)

            def get_chips_from_contours(contours, color):
                chips = []
                for contour in contours:
                    (u, v), _ = cv2.minEnclosingCircle(contour)
                    world_coords = pixelToWorld(frame, round(u), round(v), 0.0, 0.34, annotateImage=False)
                    if world_coords is not None:
                        chip = Chip(color, (float(world_coords[0]), float(world_coords[1]), float(0)))
                        chips.append(chip)
                        if chip not in chip_to_contour_map:
                            chip_to_contour_map[chip] = []
                        chip_to_contour_map[chip].append(contour)
                return chips
            
            for color, contours in [("red", red_contours), ("white", white_contours),
                                    ("blue", blue_contours), ("black", black_contours)]:
                for chip in get_chips_from_contours(contours, color):
                    if chip not in chip_to_coords_map:
                        chip_to_coords_map[chip] = [[], []]
                    chip_to_coords_map[chip][0].append(chip.coords[0])
                    chip_to_coords_map[chip][1].append(chip.coords[1])
                    

        debugging_frame = frame = self.bridge.imgmsg_to_cv2(self.prev_images[-1], "bgr8")

        chips = []
        for chip, coords in chip_to_coords_map.items():
            if len(coords[0]) > 0.8 * len(self.prev_images):
                average_chip = Chip(chip.color, (np.average(coords[0]), np.average(coords[1]), 0.0))
                chips.append(average_chip)
                self.get_logger().info(average_chip.to_string())

                x_values = []
                y_values = []
                for contour in chip_to_contour_map[chip]:
                    (x, y), _ = cv2.minEnclosingCircle(contour)
                    x_values.append(x)
                    y_values.append(y)
                    # print("center=", center)
                if chip.color == "red":
                    color = self.red
                elif chip.color == "blue":
                    color = self.blue
                elif chip.color == "white":
                    color = self.white
                elif chip.color == "black":
                    color = self.black
                cv2.circle(debugging_frame, (round(np.average(x_values)), round(np.average(y_values))), 15, color, 1)

        self.get_logger().info(f"{len(chips)}")
        cv2.imshow("debug", debugging_frame)
        cv2.waitKey(0)

        response.message = ChipMessage(chips).to_string()
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
