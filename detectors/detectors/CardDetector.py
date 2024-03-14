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
from statistics import mode


# MINAREA = 2000

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
        load_back_of_card_descriptors_from_json()
        load_box_card_descriptors_map_from_json()
        
        # Provide the /foc_detector service for the brain node to request the 
        # locations of all front of cards
        self.usb_cam_service = self.create_service(Trigger, '/foc_detector', self.foc_callback)
        self.box_cam_service = self.create_service(Trigger, '/bot_foc_detector', self.bot_foc_callback)

        self.debugpub = self.create_publisher(Image, name+'/debug', 3)

        # ranks = ['Ace','Two','Three','Four','Five','Six','Seven','Eight',
        #              'Nine','Ten','Jack','Queen','King']
        # ranks = ['Five']
        # suits = ['Diamonds', 'Clubs']
        
        # cards = []
        # for rank in ranks:
        #     for suit in suits:
        #         cards.append(rank + "_of_" + suit)

        # cards = ["Five_of_Diamonds", "Five_of_Clubs", "Jack_of_Spades", "King_of_Clubs"]

        # self.cards = cards
        # self.cards_idx = 0

        # Report.
        self.get_logger().info("CardDetector running...")

    def bot_foc_callback(self, _, response):
        if self.bot_prev_images is None:
            response.message = "No image available"
            response.success = False
            return response
        response.success = True
        # Ensure the previous image is able to be processed
        response.message = self.get_card_message(self.bot_prev_images, box_image=True)
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
        # image = self.prev_images[-1]
        response.message = self.get_card_message(self.prev_images)
        return response

    def get_card_message(self, images, box_image=False):
        cards_to_ranks_and_suits = {}

        for image in images:

            assert image.encoding == "rgb8"
            # Convert into OpenCV image, using RGB 8-bit (pass-through).
            frame = self.bridge.imgmsg_to_cv2(image, "passthrough")
            debug_frame = frame.copy()

            if box_image:
                processed_image = preprocess_box_image(frame)
                card_contours = find_box_cards(processed_image)
            else:
                processed_image = preprocess_image(frame)
                card_contours = find_cards(processed_image)

            # cv2.drawContours(frame, card_contours, -1, (255, 0, 0), 3)
            # cv2.imshow("frame", frame)
            # cv2.waitKey(0)

            
            # cv2.imshow()
            # cards = []
            for contour in card_contours:
                card_image = extract_card_from_image(frame, contour)

                if card_image is not None:
                    card_image = cv2.cvtColor(card_image, cv2.COLOR_BGR2GRAY)

                    # curr_card = self.cards[self.cards_idx]
                    # self.cards_idx += 1
                    
                    # if self.cards_idx < len(self.cards):
                    #     self.get_logger().info(self.cards[self.cards_idx])

                    # cv2.imwrite(f"{pkgdir('detectors')}/card_images/box_{curr_card}.jpg", card_image)
                    # card_image2 = cv2.imread(f"{pkgdir('detectors')}/card_images/box_{curr_card}.jpg")
                    # cv2.imshow(f"{curr_card}", card_image2)
                    # cv2.waitKey(0)

                    rank, suit = identify_card(card_image, box_image)

                    if rank is not None and suit is not None:
                        (x, y), (w, h), alpha = cv2.minAreaRect(contour)

                        debug_frame = draw_results(debug_frame, rank, suit, (round(x), round(y)))

                        if h/w < 1:
                            alpha += 90

                        alpha = np.radians(alpha)

                        # world_loc = pixelToWorld(frame, round(x), round(y), 0.0, 0.37, annotateImage=False)
                        world_loc = pixel_to_world_2(frame, round(x), round(y))
                        # self.get_logger().info(f"World loc: {world_loc}, World loc 2: {world_loc_2}")

                        # self.get_logger().info(f"world_loc={world_loc}")
                        if world_loc is not None:
                            pose = CardPose((float(world_loc[0]), float(world_loc[1] - 0.042), float(-0.01)), alpha)
                            if pose not in cards_to_ranks_and_suits:
                                cards_to_ranks_and_suits[pose] = [[], []]
                            cards_to_ranks_and_suits[pose][0].append(rank) 
                            cards_to_ranks_and_suits[pose][1].append(suit) 

                        if box_image:
                            pose = CardPose((x/100, y/100, 0.0), 0.0)
                            if pose not in cards_to_ranks_and_suits:
                                cards_to_ranks_and_suits[pose] = [[], []]
                            cards_to_ranks_and_suits[pose][0].append(rank) 
                            cards_to_ranks_and_suits[pose][1].append(suit) 

        if len(cards_to_ranks_and_suits.keys()) > 0:
            self.debugpub.publish(self.bridge.cv2_to_imgmsg(debug_frame, "rgb8"))

            cards = []
            for pose, (ranks, suits) in cards_to_ranks_and_suits.items():
                rank = mode(ranks)
                suit = mode(suits)
                cards.append((pose, rank, suit))
                self.get_logger().info(f"{rank} of {suit}")

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
