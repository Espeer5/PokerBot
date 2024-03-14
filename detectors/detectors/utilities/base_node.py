"""
This module defines a base detector node which provides the basic functionality
required by all detector nodes. This includes taking in an image from the camera 
node and storing it for processing in responding to service requests from the 
brain.
"""

import cv_bridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from collections import deque


class Detector(Node):
    """
    A general detector node which should be inherited from by each specific type
    of required detector.
    """
    # Initialization.
    def __init__(self, name):
        """
        Initialize the node, naming it as specified.
        """
        # Initialize the node.
        super().__init__(name)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()
        
        # Create a field which stores the previous image for processing on demand
        self.prev_images = deque([], maxlen=5)
        self.bot_prev_images = deque([], maxlen=1)

        # Subscribe to the incoming image from the usb cam
        self.usb_cam_sub = self.create_subscription(
                    Image, '/usb_cam/image_raw', self.process_usb_cam, 1)
        
        self.box_cam_sub = self.create_subscription(
                    Image, '/box_cam/image_raw', self.process_box_cam, 1)
        
    def shutdown(self):
        """
        Shut the node down on program exit
        """
        self.destroy_node()

    def process_usb_cam(self, msg):
        """
        Save each image to be processed as the brain node requests.
        """
        # Confirm the encoding and save for later processing
        assert(msg.encoding == "rgb8")
        
        self.prev_images.append(msg)
        assert len(self.prev_images) <= 5

    def process_box_cam(self, msg):
        """
        Save each image to be processed as the brain node requests.
        """
        # Confirm the encoding and save for later processing
        assert(msg.encoding == "rgb8")
        self.bot_prev_images.append(msg)
        assert len(self.bot_prev_images) <= 1
        