"""
simple line following node
"""

import rclpy
import rclpy.node
import cv2
import numpy as np
import math
from random import randrange

from std_msgs.msg import Int16, String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class imageProcessingNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('imageProcessingNode')

        self.declare_parameter('line_expected_at', 550)
        self.declare_parameter('canny_high', 400)
        self.declare_parameter('canny_low', 150)
        self.declare_parameter('threshold', 60)
        self.declare_parameter('minLineLength', 20)
        self.declare_parameter('maxLineGap', 10)

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

        self.steering = self.create_publisher(Int16, 'line_offset', qos_profile=qos_policy)
        self.boundary_detected = self.create_publisher(String, 'boundary_detected', qos_profile=qos_policy)
        self.parking_line = self.create_publisher(String, 'parking_line', qos_profile=qos_policy)
        self.parking_message = String()
        self.parking_message.data = "First parking bay found"

    # handling received image data
    def scanner_callback(self, data):
        canny_high = self.get_parameter('canny_high').get_parameter_value().integer_value
        canny_low = self.get_parameter('canny_low').get_parameter_value().integer_value

        threshold = self.get_parameter('threshold').get_parameter_value().integer_value
        minLineLength = self.get_parameter('minLineLength').get_parameter_value().integer_value
        maxLineGap = self.get_parameter('maxLineGap').get_parameter_value().integer_value

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # cut upper (uninteresting) half out
        height, width = img_cv.shape[:2]
        cut_img = img_cv[height-height//3:height, 0:width]
        height, width = cut_img.shape[:2]
        cut_img = cut_img[height-height//4:height, width-width//8:width]

        # use cv2 edge detection
        edged = cv2.Canny(cut_img, canny_low, canny_high)

        # detect parking bay
        parking_lines = unpack_lines(cv2.HoughLinesP(edged, rho=2, theta=np.pi/180, threshold=threshold, minLineLength=minLineLength, maxLineGap=maxLineGap))
        parking_bay_roi_color = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)

        display_parking_lines = display_lines(parking_bay_roi_color, parking_lines)
        parking_lines_img = cv2.addWeighted(parking_bay_roi_color, 0.8, display_parking_lines, 1, 10)

        filtered_parking_lines = filter_parking(parking_lines)
        display_filtered_parking_lines = display_lines(parking_bay_roi_color, filtered_parking_lines)
        filtered_parking_lines_img = cv2.addWeighted(parking_bay_roi_color, 0.8, display_filtered_parking_lines, 1, 10)

        combined_parking = np.concatenate((parking_lines_img, filtered_parking_lines_img), axis=0)
        cv2.imshow("parking", combined_parking)
        if len(filtered_parking_lines) > 0:
            self.parking_line.publish(self.parking_message)

        cv2.waitKey(1)

def filter_parking(lines):
    result = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            if (not x1 == x2) and (not y1 == y2):
                slope, y_int = np.polyfit((x1, x2), (y1, y2), 1)
                if -1 < slope < 0:
                    result.append(line)

    return np.array(result)

def unpack_lines(lines):
    unpacked = []

    if lines is not None:
        for line in lines:
            unpacked.append(line[0])
    return unpacked

def display_lines(image, lines, color=None):
    lines_image = np.zeros_like(image)
    random_color = (color == None)
    if lines is not None:
        for line in lines:
            try:
                if (len(line) > 0):
                    if random_color:
                        color = (randrange(25)*10, randrange(25)*10, randrange(25)*10)
                    x1, y1, x2, y2 = line 
                    cv2.line(lines_image, (x1, y1), (x2, y2), color, 2)
            except Exception as e:
                print("Error in display_lines: ", e)
                print(line)
                continue

    return lines_image
    

def main(args=None):
    spinUntilKeyboardInterrupt(args, imageProcessingNode)


if __name__ == '__main__':
    main()
