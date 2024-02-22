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
from geometry_msgs.msg  import Point, Pose
from detectors.utilities.mapping_utilities import pixelToWorld
from detectors.utilities.card_utilities import *


MINAREA = 1000

#
#  Detector Node Class
#
class BackCardDetectorNode(Node):
    BLUE = (0, 0, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Create a er for the processed (debugging) images.
        # Store up to three images, just in case.
        self.pubrgb = self.create_publisher(Image, name+'/image_raw', 3)
        # self.pubbin = self.create_publisher(Image, name+'/binary',    3)
        self.locpub = self.create_publisher(Point, '/point',     3)
        # self.locpubpose = self.create_publisher(Pose, '/pose',     3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Load descriptors
        load_descriptors_from_json()

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)

        # Report.
        self.get_logger().info("BackCardDetector running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Process the image (detect the ball).
    def process(self, msg):
        # Confirm the encoding and report.
        assert(msg.encoding == "rgb8")
        # self.get_logger().info(
        #     "Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (msg.width, msg.height, msg.step/msg.width, msg.encoding))

        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        processed_image = preprocess_image(frame)
        card_contours = find_cards(processed_image)

        for contour in card_contours:
            card_image = extract_card_from_image(frame, contour)
            if len(card_image) > 0:
                card_image = cv2.cvtColor(card_image, cv2.COLOR_BGR2GRAY)

                if is_back_of_card(card_image):
                    self.get_logger().info("back of card")
                    
                    (x, y), _, _ = cv2.minAreaRect(contour)

                    world_loc = pixelToWorld(frame, round(x), round(y), 0, 0.34)
                    self.get_logger().info(world_loc)

                    # And publish the centroid of the puck
                    if world_loc is not None:
                        self.locpub.publish(Point(x=float(world_loc[0]), y=float(world_loc[1]), z=float(0.01)))

        # Convert the frame back into a ROS image and republish.
        cv2.drawContours(frame, card_contours, -1, self.BLUE, 2)
        self.pubrgb.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))


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
