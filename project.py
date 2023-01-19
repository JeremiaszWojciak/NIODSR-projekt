#!/usr/bin/env python3
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge  # ROS2 package to convert between ROS and OpenCV Images
import cv2  # Python OpenCV library
import numpy as np


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.window_name = "camera"
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def listener_callback(self, image_data):
        cv_image = self.bridge.imgmsg_to_cv2(image_data, "bgr8")

        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.arucoDict, parameters=self.arucoParams)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (marker_corner, marker_id) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # draw the bounding box of the ArUCo detection
                cv2.line(cv_image, top_left, top_right, (0, 255, 0), 2)
                cv2.line(cv_image, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(cv_image, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(cv_image, bottom_left, top_left, (0, 255, 0), 2)

                # compute and draw the center (x, y)-coordinates of the ArUco
                # marker
                cx = int((top_left[0] + bottom_right[0]) / 2.0)
                cy = int((top_left[1] + bottom_right[1]) / 2.0)
                cv2.circle(cv_image, (cx, cy), 4, (0, 0, 255), -1)

                msg = Twist()
                if cy <= 256:
                    msg.linear.x = 1.0
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0
                    msg.angular.x = 0.0
                    msg.angular.y = 0.0
                    msg.angular.z = 0.0
                else:
                    msg.linear.x = -1.0
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0
                    msg.angular.x = 0.0
                    msg.angular.y = 0.0
                    msg.angular.z = 0.0
                self.publisher.publish(msg)

        cv2.line(cv_image, (0, 256), (700, 256), (255, 0, 0), 2)
        cv2.imshow(self.window_name, cv_image)
        cv2.waitKey(25)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
