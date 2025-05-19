import rclpy
import rclpy.node
import cv2
import numpy as np
import math
from random import randrange

from std_msgs.msg import Int64, String, Bool
from sensor_msgs.msg import CompressedImage, LaserScan
from cv_bridge import CvBridge
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt
from venv import create

class imageProcessingNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('imageProcessingNode')

        self.declare_parameter('line_expected_at', 550)
        self.declare_parameter('canny_high', 400)
        self.declare_parameter('canny_low', 150)
        self.declare_parameter('threshold', 50)
        self.declare_parameter('minLineLength', 11)
        self.declare_parameter('maxLineGap', 30)

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for image data with changed qos
        self.signSubscription = self.create_subscription(
            Int64,
            '/sign_seen',
            self.sign_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        self.laser_scanner_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.laser_scanner_sub  # prevent unused variable warning


        self.notice_publisher = self.create_publisher(Bool, 'parking_in_process', qos_profile=qos_policy)


        self.status = "Paused"
        self.lineNo = 0

    def sign_callback(self, data):
        if data == 0:
            self.status = "Active"
            self.lineNo = 0

    """Hier einfügen wie wir die parklinien erkennen"""
        if self.status == "Active":
            if self.lineNo++ != 0:
                self.status = "Scanning"

            if self.lineNo == 4:
                self.lineNo = 0

    def scanner_callback(self, data):
        if self.status == "Scanning":
