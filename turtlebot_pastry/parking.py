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

class parkingNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('parkingNode')

        self.declare_parameter('detection_distance', 0.30)
        self.declare_parameter('deadreconing_time', 3.0)

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

        self.line_call_sub = self.create_subscription(
            Twist,
            'parking_line',
            self.line_callback,
            qos_profile=qos_policy)

        self.notice_publisher = self.create_publisher(Bool, 'parking_in_process', qos_profile=qos_policy)
        self.parking = Bool()
        self.command_publisher = self.create_publisher(Twist, 'parking_cmd', qos_profile=qos_policy)
        self.status_publisher = self.create_publisher(String, 'parking_status', qos_profile=qos_policy)
        self.status_status = String()
        self.status_timer = self.create_timer(1, self.status_callback)
        self.status = "Paused"
        self.lineNo = 0
        #self.line_timer = self.create_timer(2000000000, self.timer_callback)

    def status_callback(self):
        self.status_status.data = self.status
        self.status_publisher.publish(self.status_status)

    def sign_callback(self, data):
        if data.data == 0:
            self.status = "Active"
            self.lineNo = 0

    def line_callback(self, data):
        if self.status == "Active":
            self.status = "Searching"
            timer_period = self.get_parameter('deadreconing_time').get_parameter_value().double_value  # seconds
            #self.line_timer.cancel()
            self.line_timer = self.create_timer(timer_period, self.timer_callback)
            self.lineNo += 1
            '''
            if self.lineNo != 0:
                self.status = "Scanning"
                timer_period = self.get_parameter('deadreconing_time').get_parameter_value().double_value  # seconds
                self.line_timer.cancel()
                self.line_timer = self.create_timer(timer_period, self.timer_callback)
                #last_call = self.lineNo
                self.lineNo += 1
                sleep(timer_period - 0.1)
                self.last_call = last_call

            if self.lineNo == 4:
                self.lineNo = 0
                self.status = "Paused"
            '''
    def scanner_callback(self, data):
        if self.status == "Scanning":
            detection_distance = self.get_parameter('detection_distance').get_parameter_value().double_value
            space_detection = min(data.ranges[500:580]) < detection_distance
            if space_detection:
                self.status = "Parking"
                self.parking.data = True
                self.notice_publisher.publish(self.parking)
                self.park()

            else:
                self.status = "Searching"

    def timer_callback(self):
        if self.status == "Searching" '''and self.last_call == self.lineNo''' and 4 > self.lineNo > 0:
            self.status = "Scanning"
            self.lineNo += 1
            #self.last_call += 1

        elif self.lineNo == 4:
            self.lineNo = 0
            self.status = "Paused"


    def follow_line_callback(self, msg):
        self.last_path_cmd = msg

    def park(self):
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
        sleep(1.5)

        # stop
        self.command_publisher.publish(cached_cmd)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.command_publisher.publish(twist)

def main(args=None):
    spinUntilKeyboardInterrupt(args, parkingNode)

if __name__ == '__main__':
    main()
