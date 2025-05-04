"""
Following node
"""

import rclpy
from rclpy.node import Node
from time import sleep
from numpy import array

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class changeLaneNode(Node):
    def __init__(self):
        #initialize
        super().__init__('changeLaneNode')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('detection_distance', 0.35)

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

        self.boundary_sub = self.create_subscription(
            String,
            'boundary_detected',
            self.boundary_callback,
            qos_profile=qos_policy)
        self.boundary_sub

        # publisher for driving commands
        self.notice_publisher = self.create_publisher(Bool, 'lane_change_in_process', qos_profile=qos_policy)
        self.laneChange = Bool()

        self.command_publisher = self.create_publisher(Twist, 'change_lane_cmd', qos_profile=qos_policy)

        # status
        self.status = "Driving"
        self.ranges = array([0.0] * 360)

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
            # detection
            front_detection = msg.ranges[180] < detection_distance
            
            # message
            if (front_detection):
                self.status = "Changing lane"
                self.laneChange.data = True
                self.notice_publisher.publish(self.laneChange)
                
                self.changeLane(toLeft=False)

    def boundary_callback(self, msg):
        if self.status == "Awaiting boundary":
            self.status = "Boundary detected"
    
    def changeLane(self, toLeft: bool):
        detection_distance = self.get_parameter('detection_distance').get_parameter_value().double_value

        twist = Twist()
        twist.angular.z = 2 if toLeft else -2
        self.command_publisher.publish(twist)

        # wait until robot has turned 90 degrees
        range_index = 90 if toLeft else 270
        while self.ranges[range_index] > detection_distance:
            sleep(0.01)
        
        # drive forward
        twist.angular.z = 0
        twist.linear.x = 2

        #wait for boundary
        self.status = "Awaiting boundary"
        while self.status == "Awaiting boundary":
            sleep(0.01)

        # turn back
        twist.linear.x = 0
        twist.angular.z = -2 if toLeft else 2
        self.command_publisher.publish(twist)

        # wait until robot has turned 90 degrees
        range_index = 90 if toLeft else 270
        while self.ranges[range_index] < detection_distance:
            sleep(0.01)

        # stop, give back control to lane follower
        self.status = "Driving left" if toLeft else "Driving right"
        self.laneChange = False
        self.notice_publisher.publish(self.laneChange)
        

def main(args=None):
    spinUntilKeyboardInterrupt(args, changeLaneNode)

if __name__ == '__main__':
    main()
