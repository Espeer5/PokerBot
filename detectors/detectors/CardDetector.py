import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from rclpy.node         import Node
from sensor_msgs.msg    import Image
from std_srvs.srv       import Trigger
from detectors.utilities.base_node import Detector
from detectors.utilities.mapping_utilities import pixelToWorld, pixel_to_world_2
from detectors.utilities.card_utilities import *
from detectors.message_types.CardMessage import CardMessage
from detectors.message_types.CardPose import CardPose
from utils.TransformHelpers import *


MINAREA = 800

#
#  Detector Node Class
#
class CardDetectorNode(Detector):
    BLUE = (0, 0, 255)

    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Load descriptors
        load_card_descriptors_map_from_json()
        
        # Provide the /foc_detector service for the brain node to request the 
        # locations of all front of cards
        self.usb_cam_service = self.create_service(Trigger, '/foc_detector', self.foc_callback)
        self.box_cam_service = self.create_service(Trigger, '/bot_foc_detector', self.bot_foc_callback)

        # Report.
        self.get_logger().info("CardDetector running...")

    def bot_foc_callback(self, _, response):
        if self.bot_prev_images is None:
            response.message = "No image available"
            response.success = False
            return response
        response.success = True
        # Ensure the previous image is able to be processed
        image = self.bot_prev_images[-1]
        response.message = self.get_card_message(image)
        return response

    def foc_callback(self, _, response):
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
        image = self.prev_images[-1]
        response.message = self.get_card_message(image)
        return response

    def get_card_message(self, image):
        assert image.encoding == "rgb8"
        # Convert into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(image, "passthrough")
        processed_image = preprocess_image(frame)
        card_contours = find_cards(processed_image)
        cards = []
        for contour in card_contours:
            card_image = extract_card_from_image(frame, contour)
            if card_image is not None:
                card_image = cv2.cvtColor(card_image, cv2.COLOR_BGR2GRAY)

                rank, suit = identify_card(card_image)

                (x, y), (w, h), alpha = cv2.minAreaRect(contour)

                if h/w < 1:
                    alpha += 90

                alpha = np.radians(alpha)

                # world_loc = pixelToWorld(frame, round(x), round(y), 0.0, 0.37, annotateImage=False)
                world_loc = pixel_to_world_2(frame, round(x), round(y))
                # self.get_logger().info(f"World loc: {world_loc}, World loc 2: {world_loc_2}")

                if world_loc is not None and rank is not None and suit is not None:
                    pose = CardPose((float(world_loc[0]), float(world_loc[1] - 0.042), float(-0.01)), alpha)
                    cards.append((pose, rank, suit))
        if len(cards) > 0:
            msg_object = CardMessage(cards)
            response_str = msg_object.to_string()
        else:
            response_str = "No cards found"
        return response_str

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()

    # Process the image (detect the ball).
    def process(self, msg):
        # Confirm the encoding and save for later processing
        assert(msg.encoding == "rgb8")
        self.prev_img = msg

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = CardDetectorNode('CardDetectorNode')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
