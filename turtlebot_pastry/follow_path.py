"""
simple line following node
"""
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from turtlebot_pastry._stop_driving import spin_until_keyboard_interrupt

import collections
import rclpy
import rclpy.node
import cv2
import numpy as np
import time
from random import randrange

class followPathNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('followPathNode')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('speed_drive', 0.12)
        self.declare_parameter('canny_high', 300)
        self.declare_parameter('canny_low', 200)
        self.declare_parameter('threshold', 60)
        self.declare_parameter('min_line_length', 20)
        self.declare_parameter('max_line_gap', 3)

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
            self.cam_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        self.last_left = collections.deque(maxlen=5)
        self.last_right = collections.deque(maxlen=5)
        self.last_middle = collections.deque(maxlen=5)

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'follow_path_cmd', 1)

    # handling received image data
    def cam_callback(self, data):
        steering = self.analyse_image(data)

        if not steering == 0.0:
            speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value

            # create message
            msg = Twist()
            msg.linear.x = speed_drive
            msg.angular.z = speed_drive * steering

            # send message
            self.publisher_.publish(msg)
    
    def analyse_image(self, image):
        canny_low = self.get_parameter('canny_low').get_parameter_value().integer_value
        canny_high = self.get_parameter('canny_high').get_parameter_value().integer_value

        threshold = self.get_parameter('threshold').get_parameter_value().integer_value
        min_line_length = self.get_parameter('min_line_length').get_parameter_value().integer_value
        max_line_gap = self.get_parameter('max_line_gap').get_parameter_value().integer_value

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding = 'passthrough')

        # cut upper (uninteresting) half out
        height, width = img_cv.shape[:2]
        cut_img = img_cv[height-height//2:height, 0:width]

        # warp image to birds eye view
        birds_eye_view = get_birds_eye_view(cut_img)

        # use cv2 edge detection
        edged = cv2.Canny(birds_eye_view, canny_low, canny_high)
        edged = remove_image_edges(edged)
        edged2color = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)

        # apply hough lines algorithm
        lines = unpack_lines(cv2.HoughLinesP(edged, rho=2, theta=np.pi/180, threshold=threshold, min_line_length=min_line_length, max_line_gap=max_line_gap))
        visual_lines = display_lines(edged2color, lines)
        lines_img = cv2.addWeighted(edged2color, 0.8, visual_lines, 1, 10)

        # filter out line the do not belong to lanes
        filtered_lines = filter_lines(self, lines)
        display_filtered_lines = display_lines(edged2color, filtered_lines, (255, 0, 0))
        lanes = cv2.addWeighted(edged2color, 0.8, display_filtered_lines, 1, 10)

        # average lines and calculate driving info
        left, right, middle = average(self, edged, filtered_lines)

        # display lanes
        display_averaged_lines = display_lines(edged2color, [left, right], (0, 255, 0))
        lanes = cv2.addWeighted(lanes, 0.8, display_averaged_lines, 1, 10)

        display_middle_line = display_lines(edged2color, [middle], (0, 0, 255))
        lanes = cv2.addWeighted(lanes, 0.8, display_middle_line, 1, 10)

        # show combined images
        lane_imgs = np.concatenate((birds_eye_view, lines_img, lanes), axis=0)
        cv2.imshow("lanes", lane_imgs)
        cv2.waitKey(1)

        return calculate_steering(middle, edged.shape[1])


def get_birds_eye_view(image):
    padding = np.zeros_like(image)
    blank = np.concatenate((padding, padding, padding), axis=1)
    with_image = np.concatenate((padding, image, padding), axis=1)
    padded = np.concatenate((blank, with_image, blank), axis=0)

    height, width = padded.shape[:2]

    third = width // 3
    offset_bottom = 110
    offset_top = 40

    # map lane-trapezoid back to rectangle
    src = np.float32([[1230+offset_bottom, 480],  [750-offset_bottom, 480], [920-offset_top, 320], [1060+offset_top, 320]]) # The source points
    dst = np.float32([[third*2, height], [third, height], [third, 0], [third*2, 0]]) # The destination points
    transformation_matrix = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

    warped = cv2.warpPerspective(padded, transformation_matrix, (width, height)) # Image warping

    height, width = warped.shape[:2]
    cut_warped = warped[height//2:height, width//4:width-width//4]

    return cut_warped

def remove_image_edges(image):
    # warping to birds eye view and then applying houghlines finds the edges of the original image
    # we remove them to improve line filtering performance
    height, width = image.shape[:2]
    left_area = np.array([[0, 130], [0, 205], [150, height], [190, height]], np.int32)
    right_area = np.array([[width, 70], [width, 100], [740, height], [755, height]], np.int32)
    
    mask = cv2.fillPoly(image, [left_area, right_area], 0)
    return mask

def unpack_lines(lines):
    # houghlines returns a list of lines in the form of [[[x1, y1, x2, y2]], ...]
    # probably because it was designed with detecting other shapes in mind which need more lines
    # we remove the unecessary nesting
    unpacked = []

    if lines is not None:
        for line in lines:
            unpacked.append(line[0])
    return unpacked

def correct_line(line):
    x1, y1, x2, y2 = line

    # correct for vertical lines
    if x1 == x2:
        x1, x2 = x1 - 1, x2 + 1
    if y1 == y2:
        y1, y2 = y1 - 1, y2 + 1

    return (x1, y1, x2, y2)

def filter_lines(self, lines):
    # the big magic of this node
    result = []
    if lines is not None:

        line_data = []

        # create auxiliary data for each line and filter out lines that are definitely not lanes
        for line in lines:
            # correct line to avoid division by zero
            line = correct_line(line)

            # filter out lines that are too flat
            angle = line_angle(line)
            if angle < 45:
                continue

            # calculate linear function
            x1, y1, x2, y2  = line
            slope = (y2-y1) / (x2-x1)
            y_int = y1 - slope * x1

            line_data.append((line, (slope, y_int), angle))

        # filter using auxiliary data
        for data in line_data:
            line = data[0]
            slope1, y_int1 = data[1]

            middle = get_middle_point(line)

            # create "norm line" that is perpendicular to the line and passes through the middle point
            norm_slope = -1 / slope1
            norm_y_int = middle[1] - norm_slope * middle[0]

            # loop through all other lines to find possible partners
            for data2 in line_data:
                slope2, y_int2 = data2[1]

                # calculate intersection of line2 and norm line
                if (slope1 != slope2) and (abs(slope2 - norm_slope) > 0.0001):  # avoid division by zero
                    x = int((norm_y_int - y_int2) / (slope2 - norm_slope))
                    y = int(norm_slope * x + norm_y_int)
                    
                    distance = np.linalg.norm([x - middle[0], y - middle[1]])

                    angle_diff = abs(data[2] - data2[2])

                    # check if intersection is in correct distance and lines are similarly angled
                    if 22 < distance < 28 and angle_diff < 5:
                        result.append(line)
                        result.append(data2[0])
    
    result = np.unique(np.array(result), axis=0)
    return result

def calculate_steering(middle_line, image_width):
    if len(middle_line) == 0:
        # no steering information if no lane is found
        return 0.0
    else:
        # angle to follow turns
        angle = line_angle(middle_line) - 90
        
        # offset to drive in the middle of the lane
        offset = image_width // 2 - get_middle_point(middle_line)[0]

        return (angle + offset) / 20

def get_middle_point(line):
    # calculate the middle point of a line
    x1, y1, x2, y2 = line 
    x_middle = (x1 + x2) // 2
    y_middle = (y1 + y2) // 2
    return (x_middle, y_middle)

def display_lines(image, lines, color=None):
    # draws lines into a blank image with the same shape as the input image

    lines_image = np.zeros_like(image)
    random_color = (color == None)
    if lines is not None:
        for line in lines:
            if (len(line) > 0):
                if random_color:
                    color = (randrange(25)*10, randrange(25)*10, randrange(25)*10)
                x1, y1, x2, y2 = line 
                cv2.line(lines_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

    return lines_image
    
def average(self, image, lines):
    # averages the lines to get a single line for each side of the lane
    left = []
    right = []
    height, width = image.shape[:2]

    if lines is None:
        return [], [], []

    # sort lines into left and right
    for line in lines:
        middle = get_middle_point(line)

        if middle[0] < width//2:
            left.append(line)
        else:
            right.append(line)

    # average left and right lines
    left_line = []
    if len(left) > 0:
        left_line = np.average(left, axis=0)

        if len(self.last_left) > 0:
            last_avg = np.average(self.last_left, axis=0)
            left_line = np.average([left_line, last_avg], axis=0)
        self.last_left.append(left_line)
   
    right_line = []
    if len(right) > 0:
        right_line = np.average(right, axis=0)

        if len(self.last_right) > 0:
            last_avg = np.average(self.last_right, axis=0)
            right_line = np.average([right_line, last_avg], axis=0)
        self.last_right.append(right_line)
    
    # create middle line from either average of left and right or from offsetting one of them if the other is empty
    middle_line = []
    offset = 200
    if len(left_line) == len(right_line) == 0:
        pass
    elif len(left_line) == 0:
        middle_line = right_line.copy()
        middle_line[0] -= offset
        middle_line[2] -= offset
    elif len(right_line) == 0:
        middle_line = left_line.copy()
        middle_line[0] += offset
        middle_line[2] += offset
    else :
        middle_line = np.average([right_line, left_line], axis=0)

    if len(middle_line) > 0:
        self.last_middle.append(middle_line)

    return left_line, right_line, middle_line

def line_angle(line):
    # calculate the angle of a line in degrees relativ to the x-axis
    x1, y1, x2, y2 = line
    dx = x2 - x1
    dy = y2 - y1

    v1 = np.array([dx, dy])
    v2 = np.array([1, 0])

    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)

    angle = np.arccos(np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0))
    return np.rad2deg(angle)
    

def main(args=None):
    spin_until_keyboard_interrupt(args, followPathNode)


if __name__ == '__main__':
    main()
