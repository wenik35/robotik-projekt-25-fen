import sys
from time import sleep

import rclpy
import rclpy.context
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist

class stopDrivingNode(Node):
    def __init__(self):
        super().__init__('stop')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

        print()
        print("sent stop command")

def spinUntilKeyboardInterrupt(args, mainNode, offset=0):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    spinner = mainNode()

    try:
        rclpy.spin(spinner)

    except KeyboardInterrupt:
        sleep(offset)
        stopNode = stopDrivingNode()

        spinner.destroy_node()
        stopNode.destroy_node()

        rclpy.shutdown()
        sys.exit(0)

def stopDriving():
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    stopNode = stopDrivingNode()
    stopNode.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    stopDriving()

if __name__ == '__main__':
    main()
