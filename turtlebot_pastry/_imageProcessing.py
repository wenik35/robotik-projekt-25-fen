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
        cut_img = img_cv[height-height//3:height, 0:width]

        # warp image to birds eye view
        #warped = warp(cut_img)

        # use cv2 edge detection
        edged = cv2.Canny(cut_img, canny_low, canny_high)
        edged2color = cv2.cvtColor(edged, cv2.COLOR_GRAY2BGR)

        # publish driving info
        grayscale = cv2.cvtColor(cut_img, cv2.COLOR_BGR2GRAY)
        grayscale[0][0] = 255
        offset = Int16()
        offset.data = analyseImageRow(edged, grayscale, line_expect_at_param)
        self.steering.publish(offset)

        # apply hough lines algorithm
        masked = region(edged)
        lines = unpack_lines(cv2.HoughLinesP(masked, rho=2, theta=np.pi/180, threshold=50, minLineLength=11, maxLineGap=30))
        visual_lines = display_lines(edged2color, lines)
        lines_img = cv2.addWeighted(edged2color, 0.8, visual_lines, 1, 10)

        # filter out line the do not belong to lanes
        filtered_lines = filter_lines(grayscale, lines)
        display_filtered_lines = display_lines(edged2color, filtered_lines)
        filtered_lines_img = cv2.addWeighted(edged2color, 0.8, display_filtered_lines, 1, 10)

        # average lines and calculate driving info
        averaged = average(cut_img, filtered_lines)
        steering = calculate_steering(averaged)

        # display lanes
        display_averaged_lines = display_lines(edged2color, averaged, (0, 255, 0))
        lanes = cv2.addWeighted(edged2color, 0.8, display_averaged_lines, 1, 10)

        # show combined images
        grayscale_color = cv2.cvtColor(grayscale, cv2.COLOR_GRAY2BGR)
        masked_color = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
        raw_imgs = np.concatenate((cut_img, grayscale_color, masked_color), axis=0)
        lane_imgs = np.concatenate((lines_img, filtered_lines_img, lanes), axis=0)
        combined = np.concatenate((raw_imgs, lane_imgs), axis=1)
        cv2.imshow("lanes", combined)


        # detect parking bay
        height, width = edged.shape[:2]
        parking_bay_roi = edged[height-height//4:height, width-width//8:width]
        parking_lines = unpack_lines(cv2.HoughLinesP(parking_bay_roi, rho=2, theta=np.pi/180, threshold=threshold, minLineLength=minLineLength, maxLineGap=maxLineGap))
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


def region(image):
    height, width = image.shape[:2]
    trapezoid = np.array([[0.3*width, height//3], [width - 0.3*width, height//3], [width, height], [0, height]], np.int32)

    mask = np.zeros_like(image)
    
    mask = cv2.fillPoly(mask, [trapezoid], 255)
    mask = cv2.bitwise_and(image, mask)
    return mask

def warp(image):
    height, width = image.shape[:2]

    src = np.float32([[width*0.5, 0],  [width, height], [0, height]]) # The source points
    dst = np.float32([[0, 0], [width, 0], [width, height], [0, height]]) # The destination points
    transformation_matrix = cv2.getPerspectiveTransform(src, dst) # The transformation matrix

    warped_img = cv2.warpPerspective(image, transformation_matrix, (width, height)) # Image warping

    return warped_img

def unpack_lines(lines):
    unpacked = []

    if lines is not None:
        for line in lines:
            unpacked.append(line[0])
    return unpacked

def filter_lines(grayscale, lines):
    height, width = grayscale.shape[:2]
    result = []
    # classify as left bound or right bound
    left = []
    right = []
    if lines is not None:
        for line in lines:
            middle = get_middle_point(line)

            if grayscale[middle[1]][(middle[0] + 3) % width] > 140:
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

    return groups

def calculate_steering(lines):
    left = lines[0]
    right = lines[1]

    if len(left) == len(right) == 0:
        return 0
    elif len(left) == 0:
        return 90 - line_angle(right)
    elif len(right) == 0:
        return 90 - line_angle(left) 
    else :
        x1, y1, x2, y2 = left
        slope_left, y_int_left = np.polyfit((x1, x2), (y1, y2), 1)

        x1, y1, x2, y2 = right 
        slope_right, y_int_right = np.polyfit((x1, x2), (y1, y2), 1)

        left_angle = line_angle(left)
        if (slope_left < 0):
            left_angle = 180 - left_angle

        right_angle = line_angle(right)
        if (slope_right > 0):
            right_angle = 180 - right_angle

        return 90 - (left_angle + right_angle) // 2

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
    
def average(image, lines):
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

                if slope < 0 and x1 < width//2:
                    left.append((slope, y_int))

                if slope > 0 and x1 > width//2:
                    right.append((slope, y_int))

    left_line = []
    if len(left) > 0:
        left_avg = np.average(left, axis=0)
        left_line = make_points(image, left_avg)    
    
    right_line = []
    if len(right) > 0:
        right_avg = np.average(right, axis=0)
        right_line = make_points(image, right_avg)

    return [left_line, right_line]

def make_points(image, line):
    try:
        slope, y_int = line 
        if not (slope == 0.0):  #TODO: why?
            y1 = image.shape[0]
            y2 = 0
            x1 = int((y1 - y_int) // slope)
            x2 = int((y2 - y_int) // slope)
            return [x1, y1, x2, y2]
    except Exception as e:
        print("Error in make_points: ", e)
        print(line)
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
