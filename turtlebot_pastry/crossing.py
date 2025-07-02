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
from rcl_interfaces.msg import SetParametersResult
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt
from venv import create

class crossingNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('crossingNode')

        self.params = {
            'left_time' : 3.9,
            'straight_time' : 3.8,
            'right_time' : 2.2,
            'line_brightness' : 245,
        }

        width = 640
        height = 480
        new_width = 1280
        new_height = 960

        v = 480.0

        K = np.array([[v, 0.0, width / 2],
                      [0.0, v, height / 2],
                      [0.0, 0.0, 1.0]])
        D = np.array([[-0.3], [0.1], [0.0], [0.0]])

        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K,
                                                                       D,
                                                                       (width, height),
                                                                       np.eye(3),
                                                                       balance = 1,
                                                                       new_size = (new_width, new_height),
                                                                       fov_scale = 1.4)
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K,
                                                                   D,
                                                                   np.eye(3),
                                                                   new_K,
                                                                   ((new_width, new_height)),
                                                                   cv2.CV_16SC2)

        for param_name, default_value in self.params.items():
            self.declare_parameter(param_name, default_value)

        for param_name in self.params.keys():
            self.params[param_name] = self.get_parameter(param_name).value

        # init openCV-bridge
        self.bridge = CvBridge()

        width = 640
        height = 480
        self.img_cv = np.ones((height, width, 3), dtype= "uint8")


        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for image data with changed qos
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.signSubscription = self.create_subscription(
            Int64,
            'sign_seen',
            self.sign_callback,
            qos_profile=qos_policy)
        self.signSubscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        self.overwriteSubscribtion = self.create_subscription(
            Bool,
            'overwright',
            self.overwriteCallback,
            qos_profile=qos_policy)


        self.notice_publisher = self.create_publisher(Bool, 'crossing_in_process', qos_profile=qos_policy)
        self.crossing_status = Bool()
        self.parking = Bool()
        self.command_publisher = self.create_publisher(Twist, 'crossing_cmd', qos_profile=qos_policy)
        self.status_publisher = self.create_publisher(String, 'crossing_status', qos_profile=qos_policy)
        self.status_status = String()
        self.status_timer = self.create_timer(1, self.status_callback)
        self.status = "Paused"
        self.direction = 2
        self.onStart = True
        self.invert = False
        #self.line_timer = self.create_timer(2000000000, self.timer_callback)
        #
    def parameter_callback(self, params):
        succ = True
        for param in params:
            if param.name in self.params:
                self.params[param.name] = param.value
                #self.get_logger().info(f"Parameter {param.name} updated to {self.params[param.name]}")
            else:
                succ = False
        return SetParametersResult(successful = succ)

    def overwriteCallback(self, msg):
        if msg.data and not self.status == "Crossing":

            self.last_status = self.status
            self.status = "Overwriten"

        if not msg.data and self.status == "Overwriten":
            self.status = "Paused"


    def status_callback(self):
        self.status_status.data = self.status
        self.status_publisher.publish(self.status_status)
        #self.get_logger().info(self.status)

    def sign_callback(self, data):
        self.get_logger().info(str(data.data))
        if 0 < data.data < 4 and self.status == "Paused":
            self.get_logger().info("SIGN FOUND!")
            self.status = "Active"
            self.status_callback()
            self.direction = data.data

    def scanner_callback(self, data):
        self.img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')
        #line_brightness = self.get_parameter('line_brightness').get_parameter_value().integer_value
        line_brightness = 170
        gsimg =self.img_cv
        gsimg = cv2.remap(gsimg,
                         self.map1,
                         self.map2,
                         interpolation = cv2.INTER_LINEAR,
                         borderMode = cv2.BORDER_CONSTANT)

        #cv2.imshow("Noo", temp)
        gsimg = cv2.cvtColor(gsimg, cv2.COLOR_BGR2GRAY)
        gsimg = gsimg[730 : 760, 500 : 800]
        gsimg = (255-gsimg) if self.invert else gsimg
        brightness = np.mean(gsimg)
        if self.onStart and brightness > 100:
            self.onStart = False
            self.invert =True
            print("Inverted")
        gsimg = cv2.cvtColor(gsimg, cv2.COLOR_GRAY2BGR)
        #cv2.line(gsimg, [500, 700], [800, 700], (255,0,0), 2)
        cv2.imshow("JJ", gsimg)
        #self.get_logger().info(str(brightness))
        if self.status == "Active" and brightness > line_brightness:
            self.get_logger().info("LINE FOUND")
            self.status = "Crossing"
            self.status_callback()
            self.crossing()

        cv2.waitKey(1)


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
        twist.angular.z = 0.0
        self.command_publisher.publish(twist)
        sleep(time - 0.2)

    def TurnRight(self):
        time = self.get_parameter('right_time').get_parameter_value().double_value
        twist = Twist()
        twist.linear.x = 0.2
        self.command_publisher.publish(twist)
        sleep(time)
        twist.linear.x = 0.0
        twist.angular.z = -1.0
        self.command_publisher.publish(twist)
        sleep(1.7)
        twist.linear.x = 0.2
        twist.angular.z = 0.0
        self.command_publisher.publish(twist)
        sleep(time)


def main(args=None):
    spinUntilKeyboardInterrupt(args, crossingNode)

if __name__ == '__main__':
    main()
