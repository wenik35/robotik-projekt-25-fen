import rclpy
import rclpy.node
import cv2
import numpy as np
import math
from random import randrange
from time import sleep

from std_msgs.msg import Int64, String, Bool
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class parkingNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('parkingNode')

        self.declare_parameter('detection_distance', 0.30)
        self.declare_parameter('deadreconing_time', 3.14159265356)

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
        self.signSubscription = self.create_subscription(
            Int64,
            'sign_seen',
            self.sign_callback,
            qos_profile=qos_policy)
        self.signSubscription  # prevent unused variable warning

        self.laser_scanner_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.laser_scanner_sub  # prevent unused variable warning

        self.follow_line_sub = self.create_subscription(
            Twist,
            'follow_path_cmd',
            self.follow_line_callback,
            qos_profile=qos_policy)

        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.cam_callback,
            qos_profile=qos_policy)

        self.notice_publisher = self.create_publisher(Bool, 'parking_in_process', qos_profile=qos_policy)
        self.parking = Bool()
        self.command_publisher = self.create_publisher(Twist, 'parking_cmd', qos_profile=qos_policy)

        self.status = "Paused"
        self.lineNo = 0
        #self.line_timer = self.create_timer(2000000000, self.timer_callback)

        self.status_timer = self.create_timer(1, self.status_callback)

    def overwriteCallback(self, msg):
            if msg.data and not self.status == "Parking":
                self.status = "Overwriten"

            if not msg.data and self.status == "Overwriten":
                self.status = "Paused"

    def status_callback(self):
        self.get_logger().info(self.status)

    def sign_callback(self, data):
        if data.data == 0 and self.status == "Paused":
            self.get_logger().info("SIGN FOUND!")
            self.status = "Active"
            self.lineNo = 0

    def scanner_callback(self, data):

        if self.status == "Scanning":
            detection_distance = self.get_parameter('detection_distance').get_parameter_value().double_value


            self.get_logger().info("Disantce540: " + str(data.ranges[540]))
            self.get_logger().info("Disantce510: " + str(data.ranges[510]))
            self.get_logger().info("Disantce580: " + str(data.ranges[580]))
            self.get_logger().info("Disantce610: " + str(data.ranges[610]))
            self.get_logger().info("Disantce470: " + str(data.ranges[470]))
            self.get_logger().info("Disantce630: " + str(data.ranges[630]))
            self.get_logger().info("Disantce450: " + str(data.ranges[450]))
            space_detection = data.ranges[540] > 0.39 and data.ranges[510] > 0.40 and data.ranges[580] > 0.41 and data.ranges[610] > 0.25 and data.ranges[470] > 0.2 and data.ranges[630] > 0.19 and data.ranges[450] > 0.19
            if space_detection:
                self.status = "Parking"
                self.parking.data = True
                self.notice_publisher.publish(self.parking)
                self.park()

            else:
                self.status = "Searching"

    def timer_callback(self):
        self.get_logger().info("TIMER " + self.status + " " + str(self.lineNo))
        if self.status == "Searching" and 4 > self.lineNo > 0:
            self.status = "Scanning"
            self.lineNo += 1
            #self.last_call += 1

        elif self.lineNo == 4:
            self.lineNo = 0
            self.status = "Paused"


    def follow_line_callback(self, msg):
        self.last_path_cmd = msg

    def park(self):
        self.line_timer.cancel()
        self.get_logger().info("PARKING")
        self.turn90Deg(False)

        # drive forward
        twist = Twist()
        twist.linear.x = 0.2
        self.command_publisher.publish(twist)
        sleep(1.5)

        self.turn90Deg(True)
        self.stop()
        sleep(11)
        self.turn90Deg(True)

        twist.linear.x = 0.2
        self.command_publisher.publish(twist)
        sleep(1.5)

        self.turn90Deg(False)
        self.parking.data = False
        self.notice_publisher.publish(self.parking)

        # stop, give back control to lane follower
        self.status = "Paused"


    def turn90Deg(self, toLeft: bool):
        cached_cmd = self.last_path_cmd

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 1.0 if toLeft else -1.0
        self.command_publisher.publish(twist)

        # wait until robot has turned 90 degrees
        sleep(1.7)

        # stop
        self.command_publisher.publish(cached_cmd)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.command_publisher.publish(twist)

    def cam_callback(self, data):
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
        #cv2.imshow("parking", combined_parking)
        if len(filtered_parking_lines) > 0:
            if self.status == "Active":
                self.get_logger().info("LINE FOUND!")
                self.status = "Searching"
                timer_period = self.get_parameter('deadreconing_time').get_parameter_value().double_value  # seconds
                #self.line_timer.cancel()
                self.line_timer = self.create_timer(timer_period, self.timer_callback)
                self.lineNo += 1

        #cv2.waitKey(1)

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
    spinUntilKeyboardInterrupt(args, parkingNode)

if __name__ == '__main__':
    main()
