#!/usr/bin/env python3
#
#   backCard.py
#
#   Detect the back of a playing card with OpenCV.
#
#   Node:           /backCard
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
from detectors.utilities.mapping_utilities import pixel_to_world_2
from detectors.utilities.card_utilities import *
from detectors.message_types.BackOfCardMessage import BackOfCardMessage
from detectors.message_types.CardPose import CardPose
from utils.TransformHelpers import *


MINAREA = 1000

#
#  Detector Node Class
#
class BackCardDetectorNode(Detector):
    BLUE = (0, 0, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)
        
        load_back_of_card_descriptors_from_json()

        # Provide the /bc_detector service for the brain node to request the 
        # locations of all backs of cards showing
        self.bc_service = self.create_service(Trigger, '/bc_detector', self.bc_callback)

        self.debugpub = self.create_publisher(Image, name+'/debug', 3)

        # Report.
        self.get_logger().info("BackCardDetector running...")

    def bc_callback(self, _, response):
        """
        Callback for the /bc_detector service. This service is called by the
        brain node to request the locations of all backs of cards showing.
        """
        if self.prev_images is None:
            response.message = "No image available"
            response.success = False
            return response
        response.success = True
        # Ensure the previous image is able to be processed
        all_card_poses = set()
        all_contours = []
        for image in self.prev_images:
            assert image.encoding == "rgb8"

            # Convert into OpenCV image, using RGB 8-bit (pass-through).
            frame = self.bridge.imgmsg_to_cv2(image, "passthrough")
            debug_frame = frame.copy()

            processed_image = preprocess_image(frame)
            card_contours = find_cards(processed_image)
            all_contours += card_contours
            card_poses = set()
            for contour in card_contours:
                card_image = extract_card_from_image(frame, contour)
                if card_image is not None:
                    card_image = cv2.cvtColor(card_image, cv2.COLOR_BGR2GRAY)

                    is_back, _ = is_back_of_card(card_image)
                    if is_back:
                        (x, y), (w, h), alpha = cv2.minAreaRect(contour)

                        if h/w < 1:
                            alpha += 90

                        alpha = np.radians(alpha)

                        world_loc = pixel_to_world_2(frame, round(x), round(y))

                        if world_loc is not None:
                            pose = CardPose((float(world_loc[0]), float(world_loc[1]), float(-0.01)), alpha)
                            card_poses.add(pose)
            all_card_poses = all_card_poses.union(card_poses)

        if len(all_card_poses) > 0:
            cv2.drawContours(debug_frame, all_contours, -1, (255, 0, 0), 3)
            self.debugpub.publish(self.bridge.cv2_to_imgmsg(debug_frame, "rgb8"))
            msg_object = BackOfCardMessage(all_card_poses)
            response_str = msg_object.to_string()
        else:
            response_str = "No cards found"
        response.message = response_str
        return response


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = BackCardDetectorNode('BackCardDetectorNode')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
