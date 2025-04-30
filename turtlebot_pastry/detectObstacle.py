"""
Following node
"""

import rclpy
import rclpy.context

from rclpy.node import Node 
from rcl_interfaces.msg import SetParametersResult

from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class detectObstacleNode(Node):
    def __init__(self):
        #initialize 
        super().__init__('detectObstacleNode')

        # definition of the parameters that can be changed at runtime
        self.params = {
            'detection_distance' : 0.35 }

        for param_name, default_value in self.params.items():
            self.declare_parameter(param_name, default_value)

        for param_name in self.params.keys():
            self.params[param_name] = self.get_parameter(param_name).value

        self.add_on_set_parameters_callback(self.parameter_callback)

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
        self.publisher_ = self.create_publisher(Bool, 'obstacle_in_path', 10)

        # status
        self.obstacleDetected = True
    def parameter_callback(self, params):
        succ = True
        for param in params:
            if param.name in self.params:
                self.params[param.name] = param.value
                self.get_logger().info(f"Parameter {param.name} updated to {self.params[param.name]}")
            else:
                succ = False
        return SetParametersResult(successful = succ)

    def scanner_callback(self, msg):
        # caching the parameters for clarity
        distance_turn_param = self.get_parameter('detection_distance').get_parameter_value().double_value

        # detection
        detection = msg.ranges[0] < distance_turn_param
        
        # message
        if (detection != self.obstacleDetected):
            self.obstacleDetected = detection

            out = Bool()
            out.data = detection

            self.publisher_.publish(out)


def main(args=None):
    spinUntilKeyboardInterrupt(args, detectObstacleNode)

if __name__ == '__main__':
    main()
