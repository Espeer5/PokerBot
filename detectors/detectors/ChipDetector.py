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
from detectors.utilities.chip_utilities import *


#
#  Detector Node Class
#
class ChipDetectorNode(Node):
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

        # Thresholds in Hmin/max, Smin/max, Vmin/max
        self.hsvlimits = np.array([[90, 120], [12, 20], [200, 230]])
        # 16, 238, 162)
        # 15, 231, 173

        # Create a publisher for the processed (debugging) images.
        # Store up to three images, just in case.
        self.pubrgb = self.create_publisher(Image, name+'/image_raw', 3)
        self.pubbin = self.create_publisher(Image, name+'/binary',    3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Finally, subscribe to the incoming image topic.  Using a
        # queue size of one means only the most recent message is
        # stored for the next subscriber callback.
        self.sub = self.create_subscription(
            Image, '/image_raw', self.process, 1)

        # Report.
        self.get_logger().info("Puck running...")

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

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Cheat: swap red/blue

        # Grab the image shape, determine the center pixel.
        (H, W, D) = frame.shape
        uc = W//2
        vc = H//2

        # Help to determine the HSV range...
        # if True:
        #     # Draw the center lines.  Note the row is the first dimension.
        #     frame = cv2.line(frame, (uc,0), (uc,H-1), self.white, 1)
        #     frame = cv2.line(frame, (0,vc), (W-1,vc), self.white, 1)

        #     # Report the center HSV values.  Note the row comes first.
        #     self.get_logger().info(
        #         "HSV = (%3d, %3d, %3d)" % tuple(hsv[vc, uc]))



        # Erode and Dilate. Definitely adjust the iterations!
        # iter = 2
        # binary = cv2.erode( binary, None, iterations=iter)
        # binary = cv2.dilate(binary, None, iterations=2*iter)
        # binary = cv2.erode( binary, None, iterations=iter)

        red, white, blue, black = preprocess_image(frame)
        # white = cv2.dilate(white, None, iterations=4)
        # white = cv2.erode(white, None, iterations=2)
        # white = cv2.dilate(white, None, iterations=2)
        cv2.imshow("red", red)
        # cv2.imshow("white", white)
        # cv2.imshow("blue", blue)
        # cv2.imshow("black", black)
        cv2.waitKey(0)
        binary = red
        # cv2.imshow("normal", processed_image)
        # cv2.waitKey(0)

        red_contours = find_chips(red)
        white_contours = find_chips(white)
        blue_contours = find_chips(blue)
        black_contours = find_chips(black)

        self.get_logger().info(f"{len(red_contours)}, {len(white_contours)}, {len(blue_contours)}, {len(black_contours)}")
        

        # processed_image = preprocess_image(frame, True)
        # cv2.imshow("inverse", processed_image)
        # cv2.waitKey(0)

        # binary = processed_image
        # contours = find_chips(processed_image)
        

        # for contour in contours:
        #     chip_image = extract_chip_from_image(frame, contour)
        #     cv2.imshow("chip", chip_image)
        #     cv2.waitKey(0)


        # Find contours in the mask and initialize the current
        # (x, y) center of the ball
        # (contours, hierarchy) = cv2.findContours(
        #     binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all contours on the original image for debugging.
        # contours = red_contours + white_contours + blue_contours + black_contours
        # cv2.drawContours(frame, contours, -1, self.blue, 2)

        # # Only proceed if at least one contour was found.  You may
        # # also want to loop over the contours...
        # if len(contours) > 0:
        #     # Pick the largest contour.
        #     contour = max(contours, key=cv2.contourArea)

        #     # Find the enclosing circle (convert to pixel values)
        #     ((ur, vr), radius) = cv2.minEnclosingCircle(contour)
        #     ur     = int(ur)
        #     vr     = int(vr)
        #     radius = int(radius)

        #     # Draw the circle (yellow) and centroid (red) on the
        #     # original image.
        #     cv2.circle(frame, (ur, vr), int(radius), self.yellow,  2)
        #     cv2.circle(frame, (ur, vr), 5,           self.red,    -1)

        #     # Report.
        #     self.get_logger().info(
        #         "Found Ball enclosed by radius %d about (%d,%d)" %
        #         (radius, ur, vr))

        # Convert the frame back into a ROS image and republish.
        self.pubrgb.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))

        # Also publish the binary (black/white) image.
        self.pubbin.publish(self.bridge.cv2_to_imgmsg(binary))


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
