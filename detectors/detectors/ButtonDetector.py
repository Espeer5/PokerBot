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
from sensor_msgs.msg    import Image
from detectors.utilities.base_node import Detector
from detectors.utilities.chip_utilities import *
from detectors.utilities.mapping_utilities import pixel_to_world_2


MIN_BUTTON_AREA = 250


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
        self.hsvlimits = np.array([[70, 100], [0, 255], [152, 255]])

        # Provice the /btn_detector service for the brain node to request the 
        # locations of all chips showing
        self.btn_service = self.create_service(Trigger, '/btn_detector', self.btn_callback)
        self.debugpub = self.create_publisher(Image, name+'/debug', 3)

        # Report.
        self.get_logger().info("ButtonDetector running...")

    # Process the image (detect the ball).
    def btn_callback(self, _, response):
        if self.prev_images is None:
            response.message = "No image available"
            response.success = False
            return response
        response.success = True

        # image = self.prev_images[-1]
        xs = []
        ys = []
        for image in self.prev_images:
            # Ensure the previous image is able to be processed
            assert(image.encoding == "rgb8")  # lies

            # Convert into OpenCV image, using RGB 8-bit (pass-through).
            frame = self.bridge.imgmsg_to_cv2(image, "bgr8")

            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            binary = cv2.inRange(hsv, self.hsvlimits[:,0], self.hsvlimits[:,1])
            # cv2.imshow("binary", binary)
            # cv2.waitKey(0)

            contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # assert len(contours) == 1
            for contour in contours:
                (u, v), r = cv2.minEnclosingCircle(contour)
                area = cv2.contourArea(contour) 
                circle_area = np.pi * r**2

                if area >= MIN_BUTTON_AREA and area / circle_area >= 0.75:
                    # self.get_logger().info(f"{cv2.contourArea(contour)}")

                    world_coords = pixel_to_world_2(frame, round(u), round(v))

                    if world_coords is not None:
                        debugging_frame = frame.copy()
                        cv2.drawContours(debugging_frame, [contour], -1, (255, 0, 0), 3)
                        self.debugpub.publish(self.bridge.cv2_to_imgmsg(debugging_frame, "rgb8"))
                        
                        # cv2.imshow("debug", frame)
                        # cv2.waitKey(0)

                        x, y = world_coords
                        xs.append(x)
                        ys.append(y)


        avg_x = np.average(xs)
        avg_y = np.average(ys)
        avg_point = np.array([avg_x, avg_y]).reshape(3, 1)
        last_point = np.array([xs[-1], ys[-1]]).reshape(3, 1)
        if np.linalg.norm(avg_point - last_point) < 0.01:
            point = f"{avg_x}, {avg_y}, {-0.02}"
            response.message = point
        return response


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = ButtonDetectorNode('ButtonDetectorNode')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
