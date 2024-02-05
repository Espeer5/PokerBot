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
from geometry_msgs.msg  import Point, Pose


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

        # Thresholds in Hmin/max, Smin/max, Vmin/max
        self.hsvlimits = np.array([[1, 20], [61, 255], [117, 202]])

        # Create a er for the processed (debugging) images.
        # Store up to three images, just in case.
        self.pubrgb = self.create_publisher(Image, name+'/image_raw', 3)
        self.pubbin = self.create_publisher(Image, name+'/binary',    3)
        self.locpub = self.create_publisher(Point, '/point',     3)
        self.locpubpose = self.create_publisher(Pose, '/pose',     3)

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


    # Pixel Conversion
    def pixelToWorld(self, image, u, v, x0, y0, annotateImage=True):
        '''
        Convert the (u,v) pixel position into (x,y) world coordinates
        Inputs:
          image: The image as seen by the camera
          u:     The horizontal (column) pixel coordinate
          v:     The vertical (row) pixel coordinate
          x0:    The x world coordinate in the center of the marker paper
          y0:    The y world coordinate in the center of the marker paper
          annotateImage: Annotate the image with the marker information

        Outputs:
          point: The (x,y) world coordinates matching (u,v), or None

        Return None for the point if not all the Aruco markers are detected
        '''

        # Detect the Aruco markers (using the 4X4 dictionary).
        markerCorners, markerIds, _ = cv2.aruco.detectMarkers(
            image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        if annotateImage:
            cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)

        # Abort if not all markers are detected.
        if (markerIds is None or len(markerIds) != 4 or
            set(markerIds.flatten()) != set([1,2,3,4])):
            return None


        # Determine the center of the marker pixel coordinates.
        uvMarkers = np.zeros((4,2), dtype='float32')
        for i in range(4):
            uvMarkers[markerIds[i]-1,:] = np.mean(markerCorners[i], axis=1)

        # Calculate the matching World coordinates of the 4 Aruco markers.
        DX = 0.1016
        DY = 0.06985
        xyMarkers = np.float32([[x0+dx, y0+dy] for (dx, dy) in
                                [(-DX, DY), (DX, DY), (-DX, -DY), (DX, -DY)]])


        # Create the perspective transform.
        M = cv2.getPerspectiveTransform(uvMarkers, xyMarkers)

        # Map the object in question.
        uvObj = np.float32([u, v])
        xyObj = cv2.perspectiveTransform(uvObj.reshape(1,1,2), M).reshape(2)


        # Mark the detected coordinates.
        if annotateImage:
            # cv2.circle(image, (u, v), 5, (0, 0, 0), -1)
            s = "(%7.4f, %7.4f)" % (xyObj[0], xyObj[1])
            cv2.putText(image, s, (u-80, v-8), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 0, 0), 2, cv2.LINE_AA)

        return xyObj


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
        if True:
            # Draw the center lines.  Note the row is the first dimension.
            frame = cv2.line(frame, (uc,0), (uc,H-1), self.white, 1)
            frame = cv2.line(frame, (0,vc), (W-1,vc), self.white, 1)

            # Report the center HSV values.  Note the row comes first.
            # self.get_logger().info(
            #     "HSV = (%3d, %3d, %3d)" % tuple(hsv[vc, uc]))

        
        # Threshold in Hmin/max, Smin/max, Vmin/max
        binary = cv2.inRange(hsv, self.hsvlimits[:,0], self.hsvlimits[:,1])

        # Erode and Dilate. Definitely adjust the iterations!
        iter = 4
        binary = cv2.erode( binary, None, iterations=iter)
        binary = cv2.dilate(binary, None, iterations=2*iter)
        binary = cv2.erode( binary, None, iterations=iter)


        # Find contours in the mask and initialize the current
        # (x, y) center of the ball
        (contours, hierarchy) = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all contours on the original image for debugging.
        cv2.drawContours(frame, contours, -1, self.blue, 2)

        # Only proceed if at least one contour was found.  You may
        # also want to loop over the contours...
        if len(contours) > 0:
            # Pick the largest contour.
            contour = max(contours, key=cv2.contourArea)

            # Find the enclosing circle (convert to pixel values)
            ((ur, vr), radius) = cv2.minEnclosingCircle(contour)

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # Sort the points based on x-coordinate
            # sorted_pts = sorted(box, key=lambda pt: pt[0])
            
            width = rect[1][0]
            height = rect[1][1]

            # STUFF
            # cv2.putText(frame, "0", (box[0][0], box[0][1]), cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5, (255, 0, 0), 2, cv2.LINE_AA)
            # cv2.putText(frame, "1", (box[1][0], box[1][1]), cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5, (255, 0, 0), 2, cv2.LINE_AA)
            # cv2.putText(frame, "2", (box[2][0], box[2][1]), cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5, (255, 0, 0), 2, cv2.LINE_AA)
            # cv2.putText(frame, "3", (box[3][0], box[3][1]), cv2.FONT_HERSHEY_SIMPLEX,
            #             0.5, (255, 0, 0), 2, cv2.LINE_AA)
            
            # side10 = np.sqrt((box[1][1] - box[0][1])**2 + (box[1][0] - box[0][0])**2)
            # orientation = np.degrees(np.arctan2(box[1][1]-box[0][1],box[1][0]-box[0][0]))
            # if side10 > 0.7*max(width,height):
            #     orientation = orientation*-1

            orientation = rect[2]
            if height/width < 1:
                orientation = orientation + 90
            orientation = np.radians(orientation)
            # self.get_logger().info(str(rect_center))
            

            ur     = int(ur)
            vr     = int(vr)
            radius = int(radius)

            # Draw the circle (yellow) and centroid (red) on the
            # original image.
            if height / width > 1.5 or height / width < 0.66:
                cv2.drawContours(frame,[box],0,(0,255,255),2)

                strip_p = self.pixelToWorld(frame, int(rect[0][0]), int(rect[0][1]), -0.33, 0.4)
                if strip_p is not None:
                    pose = Pose()
                    pose.position.x = float(strip_p[0])
                    pose.position.y = float(strip_p[1])
                    pose.position.z = float(0)
                    pose.orientation.x = float(0)
                    pose.orientation.y = float(0)
                    pose.orientation.z = np.sin(orientation/2)
                    pose.orientation.w = np.cos(orientation/2)
                    self.locpubpose.publish(pose)
            else:
                puck_p = self.pixelToWorld(frame, ur, vr, -0.33, 0.4)

                cv2.circle(frame, (ur, vr), int(radius), self.yellow,  2)

                # And publish the centroid of the puck
                if puck_p is not None:
                    self.locpub.publish(Point(x=float(puck_p[0]), y=float(puck_p[1]), z=float(0.01)))
            

            # Report.
            # self.get_logger().info(
            #     "Found Ball enclosed by radius %d about (%d,%d)" %
            #     (radius, ur, vr))

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
    node = DetectorNode('puckdetector')

    # Spin the node until interrupted.
    rclpy.spin(node)

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
