"""
simple line following node
"""
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

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
        self.declare_parameter('minLineLength', 20)
        self.declare_parameter('maxLineGap', 3)

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

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'follow_path_cmd', 1)

        # create timer to periodically invoke the driving logic
        #timer_period = 0.1  # seconds
        #self.my_timer = self.create_timer(timer_period, self.timer_callback)    
    
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
        minLineLength = self.get_parameter('minLineLength').get_parameter_value().integer_value
        maxLineGap = self.get_parameter('maxLineGap').get_parameter_value().integer_value

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding = 'passthrough')

        # cut upper (uninteresting) half out
        height, width = img_cv.shape[:2]
        cut_img = img_cv[height-height//2:height, 0:width]

        birds_eye_view = get_birds_eye_view(cut_img)

        # use cv2 edge detection
        edged = cv2.Canny(birds_eye_view, canny_low, canny_high)
        edged = remove_image_edges(edged)
        edged2color = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)

        # apply hough lines algorithm
        lines = unpack_lines(cv2.HoughLinesP(edged, rho=2, theta=np.pi/180, threshold=threshold, minLineLength=minLineLength, maxLineGap=maxLineGap))
        visual_lines = display_lines(edged2color, lines)
        lines_img = cv2.addWeighted(edged2color, 0.8, visual_lines, 1, 10)

        # filter out line the do not belong to lanes
        filtered_lines = filter_lines(lines)
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
    #TODO: optimise, takes more than half of the runtime
    padding = np.zeros_like(image)
    blank = np.concatenate((padding, padding, padding), axis=1)
    with_image = np.concatenate((padding, image, padding), axis=1)
    padded = np.concatenate((blank, with_image, blank), axis=0)

    height, width = padded.shape[:2]

    third = width // 3
    offset_bottom = 110
    offset_top = 40

    src = np.float32([[1230+offset_bottom, 480],  [750-offset_bottom, 480], [920-offset_top, 320], [1060+offset_top, 320]]) # The source points
    dst = np.float32([[third*2, height], [third, height], [third, 0], [third*2, 0]]) # The destination points
    transformation_matrix = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

    warped = cv2.warpPerspective(padded, transformation_matrix, (width, height)) # Image warping

    height, width = warped.shape[:2]
    cut_warped = warped[height//2:height, width//4:width-width//4]

    return cut_warped

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


def remove_image_edges(image):
    height, width = image.shape[:2]
    left_area = np.array([[0, 130], [0, 205], [150, height], [190, height]], np.int32)
    right_area = np.array([[width, 70], [width, 100], [740, height], [755, height]], np.int32)
    
    mask = cv2.fillPoly(image, [left_area, right_area], 0)
    return mask

def unpack_lines(lines):
    unpacked = []

    if lines is not None:
        for line in lines:
            unpacked.append(line[0])
    return unpacked

def filter_lines(lines):
    result = []
    start = time.time_ns()
    if lines is not None:

        line_data = []

        for line in lines:
            x1, y1, x2, y2 = line

            # correct for vertical lines
            if x1 == x2:
                x1, x2 = x1 - 1, x2 + 1
            if y1 == y2:
                y1, y2 = y1 - 1, y2 + 1

            angle = line_angle(line)

            # filter out lines that are too flat
            if angle < 45:
                continue

            slope = (y2-y1) / (x2-x1)
            y_int = y1 - slope * x1

            line_data.append((line, (slope, y_int), angle))

        for data in line_data:
            line = data[0]
            slope1, y_int1 = data[1]

            middle = get_middle_point(line)

            # get norm of line
            norm_slope = -1 / slope1
            norm_y_int = middle[1] - norm_slope * middle[0]

            # calculate intersection of norm with other lines
            for data2 in line_data:
                slope2, y_int2 = data2[1]

                # calculate intersection
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
    # Left x: 263 Right x: 710
    # Image width: 960 Half width: 480 Calculated middle: 486
    # links positiv, rechts negativ
    if len(middle_line) == 0:
        return 0.0
    else:
        angle = line_angle(middle_line) - 90
        
        offset = image_width // 2 - get_middle_point(middle_line)[0]

        return (angle + offset) / 20

def get_middle_point(line):
    x1, y1, x2, y2 = line 
    x_middle = (x1 + x2) // 2
    y_middle = (y1 + y2) // 2
    return (x_middle, y_middle)

def display_lines(image, lines, color=None):
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
    #TODO: optimise, polyfit takes long
    left = []
    right = []
    height, width = image.shape[:2]
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            if (not x1 == x2) and (not y1 == y2):
                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                slope = parameters[0]
                y_int = parameters[1]
                middle = get_middle_point(line)

                if middle[0] < width//2:
                    left.append((slope, y_int))
                else:
                    right.append((slope, y_int))


    left_line = []
    if len(left) > 0:
        left_avg = np.average(left, axis=0)

        if len(self.last_left) > 0:
            last_avg = np.average(self.last_left, axis=0)
            left_avg = np.average([left_avg, last_avg], axis=0)
        
        left_line = make_points(image, left_avg)
        self.last_left.append(left_avg)
   

    right_line = []
    if len(right) > 0:
        right_avg = np.average(right, axis=0)

        if len(self.last_right) > 0:
            last_avg = np.average(self.last_right, axis=0)
            right_avg = np.average([right_avg, last_avg], axis=0)
        
        right_line = make_points(image, right_avg)
        self.last_right.append(right_avg)
    
    middle_line = []
    offset = 235
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

    return left_line, right_line, middle_line

def make_points(image, line):
    slope, y_int = line
    if not (abs(slope) < 0.00001):
        y1 = image.shape[0]
        y2 = 0
        x1 = int((y1 - y_int) // slope)
        x2 = int((y2 - y_int) // slope)
        return [x1, y1, x2, y2]
    else:
        return []

def line_angle(line):
    x1, y1, x2, y2 = line
    dx = x2 - x1
    dy = y2 - y1
    vector = np.array([dx, dy])

    return vector_angle(vector, np.array([1, 0]))

def vector_angle(v1, v2):
    unit_v1 = v1 / np.linalg.norm(v1)
    unit_v2 = v2 / np.linalg.norm(v2)

    angle = np.arccos(np.clip(np.dot(unit_v1, unit_v2), -1.0, 1.0))
    return np.rad2deg(angle)
    

def main(args=None):
    spinUntilKeyboardInterrupt(args, followPathNode)


if __name__ == '__main__':
    main()
