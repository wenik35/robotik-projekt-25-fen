import rclpy
import rclpy.node
import cv2
import numpy as np

from rclpy.node import Node 
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from tomlkit.items import Bool
import array

class TrafficlightStartNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('TrafficlightStartNode')

        self.params = {
            'lower_bound' :(0,0,0),
            'upper_bound' : (255,255,255) }

        for param_name, default_value in self.params.items():
            self.declare_parameter(param_name, default_value)

        for param_name in self.params.keys():
            self.params[param_name] = self.get_parameter(param_name).value

        self.add_on_set_parameters_callback(self.parameter_callback)


        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Bool, 'GreenLight', 1)
    def parameter_callback(self, params):
        succ = True
        for param in params:
            if param.name in self.params:
                self.params[param.name] = param.value
                self.get_logger().info(f"Parameter {param.name} updated to {self.params[param.name]}")
            else:
                succ = False
        return SetParametersResult(successful = succ)

    def scanner_callback(self, data):

        lower_bound = self.get_parameter('lower_bound').get_parameter_value().array
        upper_bound = self.get_parameter('upper_bound').get_parameter_value().array

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # cropping image
        crop_img = img_cv[:][128:][:] # TODO: Optimnize cropping

        # find green
        mask = cv2.inRange(crop_img, lower_bound, upper_bound)

        if max(mask) > 0:
            print("Traffic light detected")
            self.publisher_.publish(True)
            exit()

def main(args=None):

    rclpy.init(args=args)

    node = TrafficlightStartNode

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
