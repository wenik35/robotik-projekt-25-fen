import rclpy
import rclpy.context
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

class stateMachineNode(Node):
    def __init__(self):
        #initialize
        super().__init__('stateMachineNode')

        # parameters
        self.declare_parameter('force_stop', False)

        # setup laserscanner subscription
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)

        self.lineFollowerSub = self.create_subscription(
            Twist,
            'follow_path_cmd',
            self.follower_callback,
            qos_profile=qos_policy)

        self.obstacleSub = self.create_subscription(
            Bool,
            'obstacle_in_path',
            self.obstacle_callback,
            qos_profile=qos_policy)

        self.trafficlightSub = self.create_subscription(
            Bool,
            'GreenLight',
            self.trafficlight_callback,
            qos_profile=qos_policy)

        # prevent unused variable warning
        self.lineFollowerSub
        self.obstacleSub
        self.trafficlightSub

        # status variables
        self.allowedToDrive = False
        self.greenLight = False
        self.statusMessage = String()

        # publisher for state info
        self.status = self.create_publisher(String, 'status', 10)

        # publisher for driving commands
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

    def follower_callback(self, msg):

        forbid_driving = self.get_parameter('force_stop').get_parameter_value().bool_value
        if(self.allowedToDrive and self.greenLight and not forbid_driving)):
            msg = msg
            self.cmd_vel.publish(msg)
        else:
            stop = Twist()
            self.cmd_vel.publish(stop)


    def obstacle_callback(self, msg):
        #set obstacle status
        self.allowedToDrive = not msg.data

        #announce status
        self.statusMessage.data = "Obstacle in path" if msg.data else "Driving"
        self.status.publish(self.statusMessage)

    def trafficlight_callback(self, msg):
        self.allowedToDrive = msg.data
        self.greenLight = msg.data
        self.statusMessage.data = "Driving"
        self.status.publish(self.statusMessage)

def main(args=None):
    spinUntilKeyboardInterrupt(args, stateMachineNode)

if __name__ == '__main__':
    main()
