"""
simple line following node
"""

import rclpy
import rclpy.node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class followPathNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('followPathNode')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('max_line_offset', 300)
        self.declare_parameter('steering_quotient', 4)
        self.declare_parameter('speed_drive', 0.15)

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            Int16,
            'line_offset',
            self.callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'follow_path_cmd', 1)

        # create timer to periodically invoke the driving logic
        #timer_period = 0.1  # seconds
        #self.my_timer = self.create_timer(timer_period, self.timer_callback)    

    # driving logic
    def callback(self, msg):
        # caching the parameters for reasons of clarity
        max_offset = self.get_parameter('max_line_offset').get_parameter_value().integer_value
        steering_quotient = self.get_parameter('steering_quotient').get_parameter_value().integer_value
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value

        line_offset = msg.data
        turn = 0.0

        # steer if found line is within boundary
        if (abs(line_offset) < max_offset):
            # self.line_offset has to be divided by a large number to make steering less jerky
            # self.line_offset has to be multiplied by -1 to invert left/right
            turn = speed_drive * (line_offset / steering_quotient) * -1

            # create message
            msg = Twist()
            msg.linear.x = speed_drive
            msg.angular.z = turn

            # send message
            self.publisher_.publish(msg)

def main(args=None):
    spinUntilKeyboardInterrupt(args, followPathNode)


if __name__ == '__main__':
    main()
