"""
Following node
"""

import rclpy
import rclpy.context
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class detectObstacleNode(Node):
    def __init__(self):
        #initialize 
        super().__init__('detectObstacleNode')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.35)

        # setup laserscanner subscription
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # publisher for driving commands
        self.publisher_ = self.create_publisher(Bool, 'obstacleTop', 10)

    def scanner_callback(self, msg):
        # caching the parameters for clarity
        distance_turn_param = self.get_parameter('distance_to_stop').get_parameter_value().double_value

        # get distance in front of robot
        out = Bool()
        out.data = msg.ranges[0] < distance_turn_param

        self.publisher_.publish(out)


def main(args=None):
    spinUntilKeyboardInterrupt(args, detectObstacleNode)

if __name__ == '__main__':
    main()
