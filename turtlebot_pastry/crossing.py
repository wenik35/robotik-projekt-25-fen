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
from venv import create

class crossingNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('crossingNode')

        self.declare_parameter('left_time', 4.0)
        self.declare_parameter('straight_time', 6.0)
        self.declare_parameter('line_brightness', 245)

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


        self.follow_line_sub = self.create_subscription(
            Twist,
            'follow_path_cmd',
            self.follow_line_callback,
            qos_profile=qos_policy)

        self.line_call_sub = self.create_subscription(
            String,
            'parking_line',
            self.line_callback,
            qos_profile=qos_policy)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        self.notice_publisher = self.create_publisher(Bool, 'crossing_in_process', qos_profile=qos_policy)
        self.crossing_status = Bool()
        self.parking = Bool()
        self.command_publisher = self.create_publisher(Twist, 'crossing_cmd', qos_profile=qos_policy)
        self.status_publisher = self.create_publisher(String, 'crossing_status', qos_profile=qos_policy)
        self.status_status = String()
        self.status_timer = self.create_timer(1, self.status_callback)
        self.status = "Paused"
        self.lineNo = 0
        self.direction = 2
        #self.line_timer = self.create_timer(2000000000, self.timer_callback)

    def status_callback(self):
        self.status_status.data = self.status
        self.status_publisher.publish(self.status_status)

    def sign_callback(self, data):
        self.get_logger().info(data)
        if 0 < data.data < 4 and self.status == "Paused":
            self.get_logger().info("SIGN FOUND!")
            self.status = "Active"
            self.status_callback()
            self.direction = data.data

        if self.status == "Paused":
            self.get_logger().info("SIGN FOUND!")
            self.status = "Active"
            self.status_callback()

    def scanner_callback(self, data):
        line_brightness = self.get_parameter('line_brightness').get_parameter_value().integer_value
        img = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough'), cv2.COLOR_BGR2GRAY)
        check_pixel = max(img[-0:-20, img.shape[1]//2])
        self.get_logger().info(str(check_pixel))
        if self.status == "Paused" and check_pixel > line_brightness:
            self.get_logger().info("LINE FOUND")
            self.status = "Crossing"
            self.status_callback()
            self.crossing()

    def crossing(self):
        self.crossing_status.data = True
        self.notice_publisher.publish(self.crossing_status)
        print(self.direction)
        match self.direction:
            case 1:
                self.GoStraight()
            case 2:
                self.TurnLeft()
            case 3:
                self.TurnRight()

        self.status = "Paused"
        self.status_callback()

        self.crossing_status.data = False
        self.notice_publisher.publish(self.crossing_status)

    def GoStraight(self):
        time = self.get_parameter('straight_time').get_parameter_value().double_value
        twist = Twist()
        twist.linear.x = 0.2
        self.command_publisher.publish(twist)
        sleep(time)

    def TurnLeft(self):
        time = self.get_parameter('left_time').get_parameter_value().double_value
        twist = Twist()
        twist.linear.x = 0.2
        self.command_publisher.publish(twist)
        sleep(time)
        twist.linear.x = 0.0
        twist.angular.z = 1.0
        self.command_publisher.publish(twist)
        sleep(1.7)
        twist.linear.x = 0.2
        self.command_publisher.publish(twist)
        sleep(time)

    def TurnRight(self):
        sleep(5)

    def line_callback(self, data):
        sleep(1)

    def follow_line_callback(self, data):
         sleep(1)

def main(args=None):
    spinUntilKeyboardInterrupt(args, crossingNode)

if __name__ == '__main__':
    main()
