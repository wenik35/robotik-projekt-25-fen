"""
Following node
"""

import rclpy
from rclpy.node import Node
from time import sleep

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class changeLaneNode(Node):
    def __init__(self):
        #initialize
        super().__init__('changeLaneNode')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('detection_distance', 0.30)

        # setup laserscanner subscription
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)

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

        self.parkingNoticeSub = self.create_subscription(
            Bool,
            'parking_in_process',
            self.parking_notice_callback,
            qos_profile=qos_policy)


        self.overwriteSubscribtion = self.create_subscription(
            Bool,
            'overwright',
            self.overwriteCallback,
            qos_profile=qos_policy)

        # publisher for driving commands
        self.notice_publisher = self.create_publisher(Bool, 'lane_change_in_process', qos_profile=qos_policy)
        self.laneChange = Bool()

        self.command_publisher = self.create_publisher(Twist, 'change_lane_cmd', qos_profile=qos_policy)

        # status
        self.status = "Driving right"
        self.lastDistanceRight = 0.0
        self.last_path_cmd = Twist()
        self.status_timer = self.create_timer(1, self.status_callback)

    def status_callback(self):
        self.get_logger().info(self.status)

    def overwriteCallback(self, msg):

        self.get_logger().info("Status before overwright: " + self.status)
        if msg.data and not(self.status == "Changing Lane" or self.status == "Driving Left"):

            self.last_status = self.status
            #self.status = "Overwriten"

        if not msg.data and self.status == "Overwriten":
            self.status = self.last_status


    def scanner_callback(self, msg):
        # caching the parameters for clarity
        detection_distance = self.get_parameter('detection_distance').get_parameter_value().double_value

        if self.status == "Driving right":
            # detection
            front_detection = msg.ranges[0] < detection_distance

            # message
            if (front_detection):
                self.status = "Changing lane"
                self.laneChange.data = True
                self.notice_publisher.publish(self.laneChange)

                self.changeLane(toLeft=True)

        if self.status == "Driving left":
            if (self.lastDistanceRight < 0.6):
                distance_d = msg.ranges[540] - self.lastDistanceRight
                right_detection = 0.4 < distance_d

                # let robot drive forward for a bit to get in front of obstacle
                sleep(1)

                # message
                if (right_detection):
                    self.status = "Changing lane"
                    self.laneChange.data = True
                    self.notice_publisher.publish(self.laneChange)

                    self.changeLane(toLeft=False)

        self.lastDistanceRight = msg.ranges[540]

    def follow_line_callback(self, msg):
        self.last_path_cmd = msg

    def changeLane(self, toLeft: bool):
        self.turn90Deg(toLeft)

        # drive forward
        twist = Twist()
        twist.linear.x = 0.2
        self.command_publisher.publish(twist)
        sleep(1.3)

        self.turn90Deg(not toLeft)

        # give lane detection node time to react
        sleep(0.5)

        self.laneChange.data = False
        self.notice_publisher.publish(self.laneChange)

        # stop, give back control to lane follower
        self.status = "Driving left" if toLeft else "Driving right"

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

    def parking_notice_callback(self, msg):
        if msg.data == True:
            self.status = "Parking"
        else:
            self.status = "Driving right"

def main(args=None):
    spinUntilKeyboardInterrupt(args, changeLaneNode)

if __name__ == '__main__':
    main()
