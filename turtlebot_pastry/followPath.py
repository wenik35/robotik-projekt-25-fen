"""
simple line following node
"""

import sys
import rclpy
import rclpy.node
import cv2
import numpy as np
from matplotlib import pyplot as plt

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class followPathNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('followPathNode')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('max_line_offset', 300)
        self.declare_parameter('steering_quotient', 10)
        self.declare_parameter('line_expected_at', 550)
        self.declare_parameter('speed_drive', 0.1)
        self.declare_parameter('canny_high', 800)
        self.declare_parameter('canny_low', 150)

        # offset of highest contrast from where it is expected
        self.line_offset = 0.0

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
        self.publisher_ = self.create_publisher(Twist, 'follow_path_cmd', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.1  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)


    # handling received image data
    def scanner_callback(self, data):
        line_expect_at_param = self.get_parameter('line_expected_at').get_parameter_value().integer_value
        canny_high = self.get_parameter('canny_high').get_parameter_value().integer_value
        canny_low = self.get_parameter('canny_low').get_parameter_value().integer_value

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # get image size
        height, width = img_cv.shape[:2]

        # use cv2 edge detection
        edged = cv2.Canny(img_cv, canny_low, canny_high)
        grayscale = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        # get the lowest row from image
        img_row = edged[height-5,:]

        # line should be about 40px wide
        # finding highest contrast between 5 pixel steps
        edge_indices = []
        max_diff_index = 0
        for i in range(int(len(img_row)/2), (len(img_row))):
            if img_row[i] == 255:
                edge_indices.append(i)

        for i in range(0, len(edge_indices)-1):
            # difference too small/big = not important line
            # grayscalle under 130 = space between bigger lines
            if 15 < (edge_indices[i+1] - edge_indices[i]) < 40 and grayscale[height-5, edge_indices[i]+5] > 100:
                max_diff_index = edge_indices[i]
                break
                

        # calculate offset of line from where it is expected
        self.line_offset = max_diff_index - line_expect_at_param

        # convert image to BGR for visualization and draw a dot where it expects the line
        analyzer = cv2.cvtColor(img_row, cv2.COLOR_GRAY2BGR)
        #for i in edge_indices:
        #    analyzer[(i+30)%width] = np.array([0,255,0])
        analyzer[max_diff_index] = np.array([0,0,255])
        analyzer[(max_diff_index+10)%width] = np.array([0,255,0])
        analyzer[(max_diff_index+40)%width] = np.array([0,255,0])

        # resize and rotate image for better visualization
        resized = cv2.resize(analyzer, (0,0), fx=50, fy=1) 
        turned = cv2.rotate(resized, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # show image
        cv2.imshow("IMG", img_cv)
        cv2.imshow("edged", edged)
        cv2.imshow("row", turned)
        cv2.waitKey(1)
    

    # driving logic
    def timer_callback(self):
        # caching the parameters for reasons of clarity
        max_offset = self.get_parameter('max_line_offset').get_parameter_value().integer_value
        steering_quotient = self.get_parameter('steering_quotient').get_parameter_value().integer_value
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value

        turn = 0.0

        # steer if found line is within boundary
        if (abs(self.line_offset) < max_offset):
            # self.line_offset has to be divided by a large number to make steering less jerky
            # self.line_offset has to be multiplied by -1 to invert left/right
            turn = speed_drive * (self.line_offset / steering_quotient) * -1

        # create message
        msg = Twist()
        msg.linear.x = speed_drive
        msg.angular.z = turn

        # send message
        self.publisher_.publish(msg)


def main(args=None):
    spinUntilKeyboardInterrupt(args, followPathNode)


if __name__ == '__main__':
    main()