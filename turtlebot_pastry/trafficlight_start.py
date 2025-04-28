import rclpy
import rclpy.node
import cv2
import numpy as np

from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt

import array

class TrafficlightStartNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('TrafficlightStartNode')

        self.declare_parameter('lower_bound',[55,90,0]) # TODO: figure out boundaries [10,180,135]
        self.declare_parameter('upper_bound',[65,100,15])

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

    def scanner_callback(self, data):

        lower_bound = self.get_parameter('lower_bound').get_parameter_value().integer_array_value
        lower_bound = np.array(lower_bound, dtype = "uint8")
        upper_bound = self.get_parameter('upper_bound').get_parameter_value().integer_array_value
        upper_bound = np.array(upper_bound, dtype = "uint8")

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # cropping image
        crop_img = img_cv[50:][150:300][:] # TODO: Optimnize cropping

        # find green
        mask = cv2.inRange(crop_img, lower_bound, upper_bound)

        if np.amax(mask) > 0:
            out = Bool()
            out.data = True
            self.publisher_.publish(out)
            exit()
        out = Bool()
        out.data = False
        self.publisher_.publish(out)
        cv2.imshow("IMG", img_cv)
        cv2.imshow("CROP", crop_img)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

def main(args=None):
    spinUntilKeyboardInterrupt(args, TrafficlightStartNode)


if __name__ == '__main__':
    main()
