#!/usr/bin/env python3
#
#   facedepth.py
#
#   Detect faces with OpenCV on the RealSense RGB image and determine
#   the matching depth in the ALIGNED depth image.
#
#   Node:         /facedepth
#   Subscribers:  /rgb_image_raw        RGB image
#                 /depth_image_raw      Depth image
#   Publishers:   /facedepth/image_raw  Debug image
#
import cv2
import numpy as np

# ROS Imports
import rclpy
import cv_bridge

from message_filters    import TimeSynchronizer, Subscriber
from rclpy.node         import Node
from sensor_msgs.msg    import Image


#
#  Detector Node Class
#
class DetectorNode(Node):
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

        # Create a publisher for the processed (debugging) image.
        # Store up to three images, just in case.
        self.pub = self.create_publisher(Image, name+'/image_raw', 3)

        # Set up the OpenCV bridge.
        self.bridge = cv_bridge.CvBridge()

        # Get face detector model from xml file.
        folder  = "/usr/share/opencv4/haarcascades/"
        faceXML = "haarcascade_frontalface_default.xml"
        eyeXML1 = "haarcascade_eye.xml"
        eyeXML2 = "haarcascade_eye_tree_eyeglasses.xml"

        self.faceCascade = cv2.CascadeClassifier(folder + faceXML)
        self.eyeCascade  = cv2.CascadeClassifier(folder + eyeXML1)

        # Finally, subscribe to the incoming RGB and depth image
        # topics.  We synchronize these two, so matching pairs (with
        # the same time stamp) are processed in one handler.  Note
        # they originate from the same RealSense node, so the time
        # stamps will match perfectly.  And using a queue size of one
        # means only the most recent messages are stored for the next
        # subscriber callback.
        synchronizer = TimeSynchronizer(
            [Subscriber(self, Image, 'rgb_image_raw'),
             Subscriber(self, Image, 'depth_image_raw')], 1)
        synchronizer.registerCallback(self.process)
        
        # Report.
        self.get_logger().info("Face detector with depth running...")

    # Shutdown
    def shutdown(self):
        # No particular cleanup, just shut down the node.
        self.destroy_node()


    # Process the RGB and depth images together.
    def process(self, rgbmsg, depthmsg):
        # Confirm the encoding and report.
        assert(rgbmsg.encoding   == "rgb8")
        assert(depthmsg.encoding == "16UC1")
        assert(rgbmsg.width  == depthmsg.width)
        assert(rgbmsg.height == depthmsg.height)
        # self.get_logger().info(
        #     "RGB   Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (rgbmsg.width, rgbmsg.height,
        #      rgbmsg.step/rgbmsg.width, rgbmsg.encoding))
        # self.get_logger().info(
        #     "Depth Image %dx%d, bytes/pixel %d, encoding %s" %
        #     (depthmsg.width, depthmsg.height,
        #      depthmsg.step/depthmsg.width, depthmsg.encoding))

        # Extract the depth image information (distance in mm as uint16).
        depth = np.frombuffer(depthmsg.data, np.uint16).reshape(
            depthmsg.height, depthmsg.width)

        # Convert the RGB into OpenCV image, using RGB 8-bit (pass-through).
        frame = self.bridge.imgmsg_to_cv2(rgbmsg, "passthrough")

        # Convert to gray scale.
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # Grab the faces - the cascade detector returns the bounding boxes.
        faces = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor = 1.2,
            minNeighbors = 5,
            minSize = (30,30),
            flags = cv2.CASCADE_SCALE_IMAGE)

        # Process the face: Draw the bounding box, look for the eyes,
        # and record the depth.
        for (x, y, w, h) in faces: 
            # Draw the bounding box.
            frame = cv2.rectangle(frame, (x, y), (x+w, y+h), self.green, 3)

            # Add the distance.
            d = depth[y + h//2, x + w//2]   # [row,col]
            cv2.putText(frame, str(d)+"mm", (x,y-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.green, 2)

            # Also look for eyes - only within the face!
            eyes = self.eyeCascade.detectMultiScale(gray[y:y+h,x:x+w])

            # Draw circles around the eyes :)
            for (xe,ye,we,he) in eyes:
                eye_center = (x + xe + we//2, y + ye + he//2)
                eye_radius = int(round((we + he)*0.25))
                frame = cv2.circle(frame, eye_center, eye_radius, self.red, 3)

        # Convert the frame back into a ROS image and republish.
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))


#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the detector node.
    node = DetectorNode('facedepth')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
