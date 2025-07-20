import rclpy
import rclpy.context
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from turtlebot_pastry._stop_driving import spin_until_keyboard_interrupt

class StateMachineNode(Node):
    def __init__(self):
        #initialize
        super().__init__('state_machine')

        # parameters
        self.declare_parameter('force_stop', False)

        # setup laserscanner subscription
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)

        self.lane_follower_sub = self.create_subscription(
            Twist,
            'follow_path_cmd',
            self.lane_follower_callback,
            qos_profile=qos_policy)

        self.trafficlight_sub = self.create_subscription(
            Bool,
            'green_light',
            self.trafficlight_callback,
            qos_profile=qos_policy)

        self.lane_change_notice_sub = self.create_subscription(
            Bool,
            'lane_change_in_process',
            self.lane_change_notice_callback,
            qos_profile=qos_policy)

        self.lane_change_command_sub = self.create_subscription(
            Twist,
            'change_lane_cmd',
            self.lane_changer_callback,
            qos_profile=qos_policy)

        self.parking_notice_sub = self.create_subscription(
            Bool,
            'parking_in_process',
            self.parking_notice_callback,
            qos_profile=qos_policy)

        self.parking_command_sub = self.create_subscription(
            Twist,
            'parking_cmd',
            self.parking_callback,
            qos_profile=qos_policy)

        self.crossing_notice_sub = self.create_subscription(
            Bool,
            'crossing_in_process',
            self.crossing_notice_callback,
            qos_profile=qos_policy)

        self.crossing_command_sub = self.create_subscription(
            Twist,
            'crossing_cmd',
            self.crossing_callback,
            qos_profile=qos_policy)

        # status variables
        self.driving_override = False
        self.changing_lane = False
        self.parking = False
        self.crossing = False
        self.green_light = True
        self.driving_right = True
        self.status_message = String()

        # publisher for state info
        self.status = self.create_publisher(String, 'status', 10)

        # publisher for driving commands
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        self.override_publisher = self.create_publisher(Bool, 'override', qos_policy)

    def activate_override(self):
        self.driving_override = True
        msg = Bool()
        msg.data = True
        self.override_publisher.publish(msg)

    def deactivate_override(self):
        self.driving_override = False
        msg = Bool()
        msg.data = False
        self.override_publisher.publish(msg)

    def lane_follower_callback(self, msg):
        forbid_driving = self.get_parameter('force_stop').get_parameter_value().bool_value

        if (not self.driving_override) and self.green_light and (not forbid_driving):
            self.cmd_vel.publish(msg)

    def trafficlight_callback(self, msg):
        self.green_light = msg.data
        self.status_message.data = "Driving"
        self.status.publish(self.status_message)

    def lane_change_notice_callback(self, msg):
        if not self.driving_override or self.changing_lane or not self.driving_right:
            self.changing_lane = msg.data

            #announce status
            self.status_message.data = "Changing lane" if msg.data else "Driving in lane"
            self.status.publish(self.status_message)
            if self.changing_lane:
                self.driving_override = True
                if self.driving_right:
                    self.activate_override()
                self.driving_right = not self.driving_right
            elif self.driving_right: self.deactivate_override()
            else: self.driving_override = False

    def lane_changer_callback(self, msg):
        forbid_driving = self.get_parameter('force_stop').get_parameter_value().bool_value

        if (self.changing_lane and self.green_light and not forbid_driving):
            self.cmd_vel.publish(msg)

    def parking_notice_callback(self, msg):
        if not self.driving_override or self.parking:
            self.parking = msg.data

            #announce status
            self.status_message.data = "Parking" if msg.data else "Driving in lane"
            self.status.publish(self.status_message)
            if self.parking: self.activate_override()
            else: self.deactivate_override()

    def parking_callback(self, msg):
        forbid_driving = self.get_parameter('force_stop').get_parameter_value().bool_value

        if (self.parking and self.green_light and not forbid_driving):
            self.cmd_vel.publish(msg)

    def crossing_notice_callback(self, msg):
        if not self.driving_override or self.crossing:
            self.crossing = msg.data

            #announce status
            self.status_message.data = "Crossing" if msg.data else "Driving in lane"
            self.status.publish(self.status_message)
            if self.crossing: self.activate_override()
            else: self.deactivate_override()

    def crossing_callback(self, msg):
        forbid_driving = self.get_parameter('force_stop').get_parameter_value().bool_value

        if (self.crossing and self.green_light and not forbid_driving):
            self.cmd_vel.publish(msg)
def main(args=None):
    spin_until_keyboard_interrupt(args, StateMachineNode)

if __name__ == '__main__':
    main()
