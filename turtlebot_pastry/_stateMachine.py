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

        self.laneFollowerSub = self.create_subscription(
            Twist,
            'follow_path_cmd',
            self.lane_follower_callback,
            qos_profile=qos_policy)

        self.trafficlightSub = self.create_subscription(
            Bool,
            'GreenLight',
            self.trafficlight_callback,
            qos_profile=qos_policy)

        self.laneChangeNoticeSub = self.create_subscription(
            Bool,
            'lane_change_in_process',
            self.lane_change_notice_callback,
            qos_profile=qos_policy)

        self.laneChangeCommandSub = self.create_subscription(
            Twist,
            'change_lane_cmd',
            self.lane_changer_callback,
            qos_profile=qos_policy)

        self.parkingNoticeSub = self.create_subscription(
            Bool,
            'parking_in_process',
            self.parking_notice_callback,
            qos_profile=qos_policy)

        self.parkingCommandSub = self.create_subscription(
            Twist,
            'parking_cmd',
            self.parking_callback,
            qos_profile=qos_policy)

        self.crossingNoticeSub = self.create_subscription(
            Bool,
            'crossing_in_process',
            self.crossing_notice_callback,
            qos_profile=qos_policy)

        self.crossingCommandSub = self.create_subscription(
            Twist,
            'crossing_cmd',
            self.crossing_callback,
            qos_profile=qos_policy)

        # status variables
        self.driving_overwrite = False
        self.changingLane = False
        self.parking = False
        self.crossing = False
        self.greenLight = True
        self.statusMessage = String()
        self.get_logger().info("State machine node initialized")

        # publisher for state info
        self.status = self.create_publisher(String, 'status', 10)

        # publisher for driving commands
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.overright_publisher = self.create_publisher(Bool, 'overright', qos_policy)

    def activate_overright(self):
        self.driving_overwrite = True
        msg = Bool()
        msg.data = True
        self.overright_publisher.publish(msg)

    def deactivate_overright(self):
        self.driving_overwrite = False
        msg = Bool()
        msg.data = False
        self.overright_publisher.publish(msg)

    def lane_follower_callback(self, msg):
        forbid_driving = self.get_parameter('force_stop').get_parameter_value().bool_value

        if not self.driving_overwrite and self.greenLight and (not forbid_driving):
            self.cmd_vel.publish(msg)

    def trafficlight_callback(self, msg):
        self.greenLight = msg.data
        self.statusMessage.data = "Driving"
        self.status.publish(self.statusMessage)

    def lane_change_notice_callback(self, msg):
        if not self.driving_overwrite or self.changingLane:
            self.changingLane = msg.data

            #announce status
            self.statusMessage.data = "Changing lane" if msg.data else "Driving in lane"
            self.status.publish(self.statusMessage)
            if self.changingLane: self.activate_overright()
            else: self.deactivate_overright()

    def lane_changer_callback(self, msg):
        forbid_driving = self.get_parameter('force_stop').get_parameter_value().bool_value

        if (self.changingLane and self.greenLight and not forbid_driving):
            self.cmd_vel.publish(msg)

    def parking_notice_callback(self, msg):
        if not self.driving_overwrite or self.parking:
            self.parking = msg.data

            #announce status
            self.statusMessage.data = "Parking" if msg.data else "Driving in lane"
            self.status.publish(self.statusMessage)
            if self.parking: self.activate_overright()
            else: self.deactivate_overright()

    def parking_callback(self, msg):
        forbid_driving = self.get_parameter('force_stop').get_parameter_value().bool_value

        if (self.parking and self.greenLight and not forbid_driving):
            self.cmd_vel.publish(msg)

    def crossing_notice_callback(self, msg):
        if not self.driving_overwrite or self.crossing:
            self.crossing = msg.data

            #announce status
            self.statusMessage.data = "Crossing" if msg.data else "Driving in lane"
            self.status.publish(self.statusMessage)
            if self.crossing: self.activate_overright()
            else: self.deactivate_overright()

    def crossing_callback(self, msg):
        forbid_driving = self.get_parameter('force_stop').get_parameter_value().bool_value

        if (self.crossing and self.greenLight and not forbid_driving):
            self.cmd_vel.publish(msg)
def main(args=None):
    spinUntilKeyboardInterrupt(args, stateMachineNode)

if __name__ == '__main__':
    main()
