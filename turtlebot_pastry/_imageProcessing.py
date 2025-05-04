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

        self.line_offset = self.create_publisher(Int16, 'line_offset', qos_profile=qos_policy)
        self.boundary_detected = self.create_publisher(String, 'boundary_detected', qos_profile=qos_policy)

    # handling received image data
    def scanner_callback(self, data):
        line_expect_at_param = self.get_parameter('line_expected_at').get_parameter_value().integer_value
        canny_high = self.get_parameter('canny_high').get_parameter_value().integer_value
        canny_low = self.get_parameter('canny_low').get_parameter_value().integer_value

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # cut upper (uninteresting) half out
        cut_img = img_cv[img_cv.shape[0]//2:img_cv.shape[0], 0:img_cv.shape[1]]

        # warp image to birds eye view
        #warped = warp(cut_img)

        # use cv2 edge detection
        edged = cv2.Canny(cut_img, canny_low, canny_high)
        grayscale = cv2.cvtColor(cut_img, cv2.COLOR_BGR2GRAY)

        offset = Int16()
        offset.data = analyseImageRow(edged, grayscale, line_expect_at_param)
        self.line_offset.publish(offset)

        masked = region(edged)
        lines = cv2.HoughLinesP(masked, rho=2, theta=np.pi/180, threshold=100, minLineLength=40, maxLineGap=5)

        final_to_color = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)
        visual_lines = display_lines(final_to_color, lines, (255, 0, 0))
        lanes = cv2.addWeighted(final_to_color, 0.8, visual_lines, 1, 10)

        boundary_lines = detect_boundary(edged, lines)
        if boundary_lines is not None:
            boundary_message = String()
            boundary_message.data = "Boundary detected"
            self.boundary_detected.publish(boundary_message)

            boundary_display_lines = display_lines(final_to_color, boundary_lines, (0, 255, 0))
            lanes = cv2.addWeighted(boundary_display_lines, 0.8, lanes, 1, 10)

        # show image
        cv2.imshow("original", cut_img)
        cv2.imshow("edged+masked", masked)
        cv2.imshow("lanes", lanes)
        cv2.waitKey(1)

def region(image):
    height, width = image.shape[:2]
    trapezoid = np.array([[0.24*width, 0], [width - 0.24*width, 0], [width, height], [0, height]], np.int32)

    mask = np.zeros_like(image)
    
    mask = cv2.fillPoly(mask, [trapezoid], 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

def warp(image):
    height, width = image.shape[:2]

    src = np.float32([[width*0.42, 0], [width*0.54, 0], [width, height], [0, height]]) # The source points
    dst = np.float32([[0, 0], [width, 0], [width, height], [0, height]]) # The destination points
    transformation_matrix = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

    warped_img = cv2.warpPerspective(image, transformation_matrix, (width, height)) # Image warping

    return warped_img


def display_lines(image, lines, color):
    lines_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            try:
                x1, y1, x2, y2 = line[0]
                cv2.line(lines_image, (x1, y1), (x2, y2), color, 2)
            except Exception as e:
                print("Error in display_lines: ", e)
                print(line[0])
                continue

    return lines_image

def detect_boundary(image, lines):
    polys = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            y_int = parameters[1]
            if -0.05 < slope < 0.05 and 230 < y_int < 240:
                polys.append((slope, y_int))

    bound_lines = make_points(image, polys)
    return np.array(bound_lines)
    
def make_points(image, lines):
    point_arrays = []
    if lines is not None:
        for line in lines:
            slope, y_int = line 
            if not (slope == 0.0):
                y1 = image.shape[0]
                y2 = int(y1 * (3/5))
                x1 = int((y1 - y_int) // slope)
                x2 = int((y2 - y_int) // slope)
                point_arrays.append([[x1, y1, x2, y2]])
    return point_arrays


def analyseImageRow(edged, grayscale, line_expected_at):
    height, width = edged.shape[:2]
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
    line_offset = max_diff_index - line_expected_at
    # debug_image_row(img_row, max_diff_index)

    return line_offset

def debug_image_row(img_row, max_diff_index):
    length = length(img_row)

    # convert image to BGR for visualization and draw a dot where it expects the line
    analyzer = cv2.cvtColor(img_row, cv2.COLOR_GRAY2BGR)
    analyzer[max_diff_index] = np.array([0,0,255])
    analyzer[(max_diff_index+10)%length] = np.array([0,255,0])
    analyzer[(max_diff_index+40)%length] = np.array([0,255,0])

    # resize and rotate image for better visualization
    resized = cv2.resize(analyzer, (0,0), fx=50, fy=1) 
    turned = cv2.rotate(resized, cv2.ROTATE_90_COUNTERCLOCKWISE)
    cv2.imshow("row", turned)
    

def main(args=None):
    spinUntilKeyboardInterrupt(args, imageProcessingNode)


if __name__ == '__main__':
    main()
