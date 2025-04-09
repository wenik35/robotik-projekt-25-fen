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

        # setup laserscanner subscription
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.follower = self.create_subscription(
            Twist,
            'followLineTop',
            self.follower_callback,
            qos_profile=qos_policy)
        
        self.obstacle = self.create_subscription(
            Bool,
            'obstacleTop',
            self.obstacle_callback,
            qos_profile=qos_policy)
        
        # prevent unused variable warning
        self.follower
        self.obstacle

        # status variables
        self.obstacleInPath = False

        # publisher for state info
        self.status = self.create_publisher(String, 'status', 10)

        # publisher for driving commands
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

    def follower_callback(self, msg):
        if(not self.obstacleInPath):
            self.cmd_vel.publish(msg)
        else:
            stop = Twist()
            self.cmd_vel.publish(stop)


    def obstacle_callback(self, msg):
        self.obstacleInPath = msg.data

def main(args=None):
    spinUntilKeyboardInterrupt(args, stateMachineNode)

if __name__ == '__main__':
    main()
