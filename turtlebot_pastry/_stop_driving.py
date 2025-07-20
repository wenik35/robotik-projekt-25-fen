import sys
from time import sleep

import rclpy
import rclpy.context
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist

class StopDrivingNode(Node):
    def __init__(self):
        super().__init__('stop_driving')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        for i in range(10):
            self.publisher_.publish(msg)

        print()
        print("sent stop command")

def spin_until_keyboard_interrupt(args, mainNode, offset=0):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    spinner = mainNode()

    try:
        rclpy.spin(spinner)

    except KeyboardInterrupt:
        sleep(offset)
        stopNode = StopDrivingNode()

        spinner.destroy_node()
        stopNode.destroy_node()

        rclpy.shutdown()
        sys.exit(0)

def stop_driving():
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    stopNode = StopDrivingNode()
    stopNode.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    stop_driving()

if __name__ == '__main__':
    main()
