"""
simple line following node
"""

import sys
import rclpy
import rclpy.node
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class followLineNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('followLineNode')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('boundary_left', 0.6)
        self.declare_parameter('boundary_right', 0.6)
        self.declare_parameter('speed_drive', 0.1)
        self.declare_parameter('speed_turn', 1.5)

        # position of highest contrast
        self.lineposition = 0.0
        self.maxdiff = 0

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'followLineTop', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)


    # handling received image data
    def scanner_callback(self, data):

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # convert image to grayscale
        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        # get image size
        height, width = img_gray.shape[:2]

        # get the lowest row from image
        img_row = img_gray[height-1,:]

        # finding highest contrast
        max_diff = 0
        max_diff_index = 0
        for i in range(0, (len(img_row)-6), 5):
            diff = abs(int(img_row[i+5]) - int(img_row[i]))
            if diff > max_diff:
                max_diff = diff
                max_diff_index = i

        half_length = len(img_row)/2        
        self.maxdiff = max_diff

        #normiert index so, dass er abhängig von mitte positiv oder negativ ist
        self.lineposition = ((max_diff_index - half_length - 220) / half_length) * -1

        # visible image row
        img_row[max_diff_index] = 255
        img_row_vis = np.repeat(img_row, 50).reshape(len(img_row), 50).swapaxes(0, 1)

        # show image
        cv2.imshow("IMG", img_gray)
        cv2.imshow("IMG_ROW", img_row_vis)
        cv2.waitKey(1)


    # driving logic
    def timer_callback(self):

        # caching the parameters for reasons of clarity
        boundary_left = self.get_parameter('boundary_left').get_parameter_value().double_value
        boundary_right = self.get_parameter('boundary_right').get_parameter_value().double_value
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value

        # todo logic
        speed = speed_drive
        turn = 0.0
        if (-boundary_right < self.lineposition < boundary_left):
            turn = speed_turn * self.lineposition

        # create message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn

        # send message
        self.publisher_.publish(msg)


def main(args=None):
    spinUntilKeyboardInterrupt(args, followLineNode)


if __name__ == '__main__':
    main()