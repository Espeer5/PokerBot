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
from detectors.utilities.mapping_utilities import pixel_to_world_2
from detectors.message_types.ChipMessage import ChipMessage
from detectors.message_types.Chip import Chip


#
#  Use Trackbars to Vary HSV Limits
#
# A single trackbar object
class TrackBar():
    def __init__(self, winname, barname, hsvlimits, channel, element, maximum):
        # Store the parameters.
        self.winname   = winname
        self.barname   = barname
        self.hsvlimits = hsvlimits
        self.channel   = channel
        self.element   = element
        # Create the trackbar.
        cv2.createTrackbar(barname, winname,
                           hsvlimits[channel,element], maximum, self.CB)

    def CB(self, val):
        # Make sure the threshold doesn't pass the opposite limit.
        if self.element == 0:  val = min(val, self.hsvlimits[self.channel,1])
        else:                  val = max(val, self.hsvlimits[self.channel,0])
        # Update the threshold and the tracker position.
        self.hsvlimits[self.channel,self.element] = val
        cv2.setTrackbarPos(self.barname, self.winname, val)

# A combined HSV limit tracker.
class HSVTracker():
    def __init__(self, hsvlimits):
        # Create a controls window for the trackbars.
        winname = 'Controls'
        cv2.namedWindow(winname)

        # Show the control window.  Note this won't actually appear/
        # update (draw on screen) until waitKey(1) is called below.
        cv2.imshow(winname, np.zeros((1, 500, 3), np.uint8))

        # Create trackbars for each limit.
        TrackBar(winname, 'Lower H', hsvlimits, 0, 0, 179)
        TrackBar(winname, 'Upper H', hsvlimits, 0, 1, 179)
        TrackBar(winname, 'Lower S', hsvlimits, 1, 0, 255)
        TrackBar(winname, 'Upper S', hsvlimits, 1, 1, 255)
        TrackBar(winname, 'Lower V', hsvlimits, 2, 0, 255)
        TrackBar(winname, 'Upper V', hsvlimits, 2, 1, 255)

    def update(self):
        # Call waitKey(1) to force the window to update.
        cv2.waitKey(1)




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
        self.pubbin = self.create_publisher(Image, name+'/binary',    3)
        self.debugpub = self.create_publisher(Image, name+'/debug', 3)

        # 85-95, 120-190, 151-214

        # Provice the /ch_detector service for the brain node to request the 
        # locations of all chips showing
        self.ch_service = self.create_service(Trigger, '/ch_detector', self.ch_callback)
        # self.hsvlimits = np.array([[0, 180], [0, 255], [0, 255]])
        # self.tracker = HSVTracker(self.hsvlimits)

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

            # self.tracker.update()

            # hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            # binary = cv2.inRange(hsv, self.hsvlimits[:,0], self.hsvlimits[:,1])
            # # cv2.imshow("binary", binary)
            # # cv2.waitKey(0)
            # self.pubbin.publish(self.bridge.cv2_to_imgmsg(binary))
            
            red, white, blue, black = preprocess_image(frame)

            red_contours = find_chips(frame, red, "red")
            white_contours = find_chips(frame, white, "white")
            blue_contours = find_chips(frame, blue, "blue")
            black_contours = find_chips(frame, black, "black")

            # contours_frame = frame = self.bridge.imgmsg_to_cv2(self.prev_images[-1], "bgr8")
            # contours = red_contours + white_contours + blue_contours + black_contours
            # cv2.drawContours(contours_frame, contours, -1, (0, 0, 255), 3)
            # cv2.imshow("contours", contours_frame)
            # cv2.waitKey(0)
            

            # self.get_logger().info(f"{len(red_contours)}, {len(white_contours)}, {len(blue_contours)}, {len(black_contours)}")
            # contours = red_contours + white_contours + blue_contours + black_contours
            # cv2.drawContours(frame, contours, -1, (0, 0, 255), 3)
            # cv2.imshow("contours", frame)
            # cv2.waitKey(0)

            def get_chips_from_contours(contours, color):
                chips = []
                for contour in contours:
                    (u, v), _ = cv2.minEnclosingCircle(contour)
                    world_coords = pixel_to_world_2(frame, round(u), round(v))
                    if world_coords is not None:
                        chip = Chip(color, (float(world_coords[0]), float(world_coords[1]), float(0.0)))
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
                    

        debugging_frame = self.bridge.imgmsg_to_cv2(self.prev_images[-1], "bgr8")

        chips = []
        for chip, coords in chip_to_coords_map.items():
            if len(coords[0]) > 0.6 * len(self.prev_images):
                average_chip = Chip(chip.color, (np.average(coords[0]), np.average(coords[1]), -0.03))
                chips.append(average_chip)
                # self.get_logger().info(average_chip.to_string())

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
                    color = self.black
                elif chip.color == "black":
                    color = self.white
                cv2.circle(debugging_frame, (round(np.average(x_values)), round(np.average(y_values))), 15, color, 1)

        if len(chips) > 0:
            self.debugpub.publish(self.bridge.cv2_to_imgmsg(debugging_frame, "bgr8"))
            # cv2.imshow("debug", debugging_frame)
            # cv2.waitKey(0)

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
