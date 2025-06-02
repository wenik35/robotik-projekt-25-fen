"""
simple line following node
"""

import rclpy
import rclpy.node
import cv2
import numpy as np
import math
import time
from random import randrange

from std_msgs.msg import Int16, String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class imageProcessingNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('imageProcessingNode')

        self.declare_parameter('line_expected_at', 550)
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
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        self.steering = self.create_publisher(Int16, 'line_offset', qos_profile=qos_policy)
        self.boundary_detected = self.create_publisher(String, 'boundary_detected', qos_profile=qos_policy)
        self.parking_line = self.create_publisher(String, 'parking_line', qos_profile=qos_policy)
        self.parking_message = String()
        self.parking_message.data = "First parking bay found"

        self.last_left = []
        self.last_right = []

    # handling received image data
    def scanner_callback(self, data):
        line_expect_at_param = self.get_parameter('line_expected_at').get_parameter_value().integer_value
        canny_high = self.get_parameter('canny_high').get_parameter_value().integer_value
        canny_low = self.get_parameter('canny_low').get_parameter_value().integer_value

        threshold = self.get_parameter('threshold').get_parameter_value().integer_value
        minLineLength = self.get_parameter('minLineLength').get_parameter_value().integer_value
        maxLineGap = self.get_parameter('maxLineGap').get_parameter_value().integer_value

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # cut upper (uninteresting) half out
        height, width = img_cv.shape[:2]
        cut_img = img_cv[height-height//2:height, 0:width]

        birds_eye_view = get_birds_eye_view(cut_img)

        #grayscale = cv2.cvtColor(birds_eye_view, cv2.COLOR_BGR2GRAY)
        #normalizedGrayscale = np.zeros_like(grayscale)
        #normalizedGrayscale = cv2.normalize(grayscale,  normalizedGrayscale, 0, 255, cv2.NORM_MINMAX)

        # use cv2 edge detection
        edged = cv2.Canny(birds_eye_view, canny_low, canny_high)
        edged = remove_image_edges(edged)
        edged2color = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)


        # publish driving info
        #offset = Int16()
        #offset.data = analyseImageRow(edged, grayscale, line_expect_at_param)
        #self.steering.publish(offset)

        # apply hough lines algorithm
        lines = unpack_lines(cv2.HoughLinesP(edged, rho=2, theta=np.pi/180, threshold=threshold, minLineLength=minLineLength, maxLineGap=maxLineGap))
        visual_lines = display_lines(edged2color, lines)
        lines_img = cv2.addWeighted(edged2color, 0.8, visual_lines, 1, 10)

        # filter out line the do not belong to lanes
        filtered_lines = filter_lines(lines)
        display_filtered_lines = display_lines(edged2color, filtered_lines, (255, 0, 0))
        filtered_lines_img = cv2.addWeighted(edged2color, 0.8, display_filtered_lines, 1, 10)

        # average lines and calculate driving info
        averaged = average(edged, filtered_lines, self.last_left, self.last_right)

        left_line = []
        if( len(averaged[0]) > 0):
            left_line = make_points(edged, averaged[0])
            self.last_left.append(averaged[0])
            if len(self.last_left) > 10:
                self.last_left.pop(0)

        right_line = []
        if( len(averaged[1]) > 0):
            right_line = make_points(edged, averaged[1])
            self.last_right.append(averaged[1])
            if len(self.last_right) > 10:
                self.last_right.pop(0)

        steering_factor = calculate_steering([left_line, right_line], edged.shape[1])
        if not steering_factor == 0:
            steering_msg = Int16()
            steering_msg.data = int(steering_factor)
            self.steering.publish(steering_msg)

        # display lanes
        display_averaged_lines = display_lines(edged2color, [left_line, right_line], (0, 255, 0))
        lanes = cv2.addWeighted(filtered_lines_img, 0.8, display_averaged_lines, 1, 10)

        # show combined images
        lane_imgs = np.concatenate((birds_eye_view, lines_img, lanes), axis=0)
        cv2.imshow("lanes", lane_imgs)



        # detect parking bay
        height, width = edged.shape[:2]
        parking_bay_roi = edged[height-height//4:height, width-width//8:width]
        parking_lines = unpack_lines(cv2.HoughLinesP(parking_bay_roi, rho=2, theta=np.pi/180, threshold=60, minLineLength=20, maxLineGap=10))
        parking_bay_roi_color = cv2.cvtColor(parking_bay_roi, cv2.COLOR_GRAY2BGR)

        display_parking_lines = display_lines(parking_bay_roi_color, parking_lines)
        parking_lines_img = cv2.addWeighted(parking_bay_roi_color, 0.8, display_parking_lines, 1, 10)

        filtered_parking_lines = filter_parking(parking_lines)
        display_filtered_parking_lines = display_lines(parking_bay_roi_color, filtered_parking_lines)
        filtered_parking_lines_img = cv2.addWeighted(parking_bay_roi_color, 0.8, display_filtered_parking_lines, 1, 10)

        combined_parking = np.concatenate((parking_lines_img, filtered_parking_lines_img), axis=0)
        #cv2.imshow("parking", combined_parking)
        if len(filtered_parking_lines) > 0:
            self.parking_line.publish(self.parking_message)

        cv2.waitKey(1)

def get_birds_eye_view(image):
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
    
    resized = cv2.resize(cut_warped, [height//2, width//4])

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
    #middle_tri = np.array([[300, height], [width-300, height], [width//2, 100]], np.int32)
    
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
                start = time.time_ns()
                line2 = data2[0]
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
    
    result = np.unique(np.array(result), axis=0)
    return result

def filter_lines_legacy(grayscale, lines):
    height, width = grayscale.shape[:2]
    result = []
    # classify as left bound or right bound
    left = []
    right = []
    if lines is not None:
        for line in lines:
            middle = get_middle_point(line)
            brightness_right =  grayscale[(middle[1] + 10) % width, middle[0]]
            brightness_left =  grayscale[(middle[1] - 10) % width, middle[0]]


            if brightness_right > brightness_left:
                left.append(line)
            else:
                right.append(line)

    groups = []
    # group left and right bounds
    for line_left in left:
        left_middle = get_middle_point(line_left)
        potential_partner = []

        # find potential partners to the right and roughly in the same height
        for line_right in right:
            right_x = get_x_sorted(line_right)
            right_middle = get_middle_point(line_right)

            if right_x[0] < left_middle[0] < right_x[1] and right_middle[1] > left_middle[1]:
                potential_partner.append(line_right)

        # if there is more than one potential partner, find the closest one
        if len(potential_partner) > 0:
            groups.append(line_left)

            if len(potential_partner) > 1:
                left_middle = get_middle_point(line_left)
                
                dists = []
                for line in potential_partner:
                    right_middle = get_middle_point(line)
                    dists.append(math.dist(left_middle, right_middle))

                smallest = np.argmin(dists)

                groups.append(potential_partner[smallest])
            else:
                groups.append(potential_partner[0])

    return [left, right]

def calculate_steering(lines, image_width):
    # Left x: 263 Right x: 710
    # Image width: 960 Half width: 480 Calculated middle: 486
    # links positiv, rechts negativ
    left = lines[0]    
    right = lines[1]

    angle = 0
    offset = 0

    if len(left) == len(right) == 0:
        return 0
    elif len(left) == 0:
        angle = 90 - line_angle(right)
        offset = image_width / 2 - right[0] + 220
    elif len(right) == 0:
        angle = 90 - line_angle(left)
        offset = image_width / 2 - left[0] - 220
    else :
        x1_left, y1, x2, y2 = left
        slope_left, y_int_left = np.polyfit((x1_left, x2), (y1, y2), 1)

        x1_right, y1, x2, y2 = right 
        slope_right, y_int_right = np.polyfit((x1_right, x2), (y1, y2), 1)

        left_angle = line_angle(left)
        if (slope_left > 0):
            left_angle = 180 - left_angle

        right_angle = line_angle(right)
        if (slope_right > 0):
            right_angle = 180 - right_angle

        angle = 90 - (left_angle + right_angle) / 2

        offset = image_width / 2 - (x1_left + x1_right) / 2
    
    steering = (angle + offset) / 4

    return steering * 2

def get_middle_point(line):
    x1, y1, x2, y2 = line 
    x_middle = (x1 + x2) // 2
    y_middle = (y1 + y2) // 2
    return (x_middle, y_middle)

def get_x_sorted(line):
    x1, y1, x2, y2 = line 
    if x1 < x2:
        return (x1, x2)
    else:
        return (x2, x1)

def display_lines(image, lines, color=None):
    lines_image = np.zeros_like(image)
    random_color = (color == None)
    if lines is not None:
        for line in lines:
            if (len(line) > 0):
                if random_color:
                    color = (randrange(25)*10, randrange(25)*10, randrange(25)*10)
                x1, y1, x2, y2 = line 
                cv2.line(lines_image, (x1, y1), (x2, y2), color, 2)

    return lines_image

def detect_boundary(image, lines):
    polys = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line 
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            y_int = parameters[1]
            if -0.05 < slope < 0.05 and 200 < y_int < 240:
                polys.append((slope, y_int))

    bound_lines = make_points(image, polys)
    return np.array(bound_lines)
    
def average(image, lines, last_left: list, last_right: list):
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

    left_avg = []
    if len(left) > 0:
        left_avg = np.average(left, axis=0)
        if len(last_left) > 0:
            last_avg = np.average(last_left, axis=0)
            left_avg = np.average([left_avg, last_avg], axis=0)
   
    right_avg = []
    if len(right) > 0:
        right_avg = np.average(right, axis=0)
        if len(last_right) > 0:
            last_avg = np.average(last_right, axis=0)
            right_avg = np.average([right_avg, last_avg], axis=0)

    return [left_avg, right_avg]

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
