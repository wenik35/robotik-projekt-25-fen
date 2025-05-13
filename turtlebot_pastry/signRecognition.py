from math import ulp
from os import chroot
import rclpy
import rclpy.node
import cv2
import numpy as np

from skimage.util.arraycrop import crop
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt
from skimage.metrics import structural_similarity

import array

class SignRecognitionNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('SignRecognitionNode')

        self.declare_parameter('lower_bound',[87,235,20]) # TODO: figure out boundaries
        self.declare_parameter('upper_bound',[98,255,128])

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


        image_list = []
        image_list.append(cv2.imread("./Media/GoStraight.png"))
        image_list.append(cv2.imread("./Media/ParkPlatz.png"))
        image_list.append(cv2.imread("./Media/TurnLeft.png"))
        image_list.append(cv2.imread("./Media/TurnRight.png"))
        image_list.append(cv2.imread("./Media/ZebraStreifen.png"))


        self.crop_list = []
        lower_bound = np.array([140,55,0], dtype = "uint8")
        upper_bound = np.array([155,97,0], dtype = "uint8")

        for i in image_list:
            self.crop_list.append(cv2.inRange(i, lower_bound, upper_bound))

        '''
        cv2.imwrite("test", image_list[0])
        cv2.imwrite("test_crop", crop_list[0])
        '''


    def scanner_callback(self, data):

        lower_bound = self.get_parameter('lower_bound').get_parameter_value().integer_array_value
        lower_bound = np.array(lower_bound, dtype = "uint8")
        upper_bound = self.get_parameter('upper_bound').get_parameter_value().integer_array_value
        upper_bound = np.array(upper_bound, dtype = "uint8")

        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')
        cv2.imshow("IMG", img_cv)
        hsv_img = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HSV)

        # cropping image
        crop_img = hsv_img[:, 480:] # TODO: Optimize cropping
        crop_img = crop_img[150:430]
        crop_img = cv2.resize(crop_img, (crop_img.shape[1]//2, crop_img.shape[0]//2))
        # find blue
        mask = cv2.inRange(crop_img, lower_bound, upper_bound)
        if np.amax(mask > 0):
            crop_up = 0
            crop_down = mask.shape[0]
            crop_left = 0
            crop_right = mask.shape[1]
            print(mask.shape[0])
            print(mask.shape[1])

            for i in range (mask.shape[0]):
                #print(i)
                if(np.max(mask[i]) > 0):
                    crop_up = i
                    break

            for i in range (mask.shape[0]-1, -1, -1):
                #print(i)
                if(np.max(mask[i]) > 0):
                    crop_down = i
                    break

            for i in range(mask.shape[1]):
                #print(i)
                if(np.max(mask[:,i]) > 0):
                    crop_left = i
                    break

            for i in range (mask.shape[1]-1, -1, -1):
                #print(i)
                print(len(mask[0]))
                if(np.max(mask[:,i]) > 0):
                    crop_right = i
                    break

            print("crop_up: ", crop_up)
            print("crop_down: ", crop_down)
            print("crop_left: ", crop_left)
            print("crop_right: ", crop_right)

            crop_mask = mask[crop_up:crop_down, crop_left:crop_right]
            precise_crop = crop_img[crop_up:crop_down, crop_left:crop_right]

            if crop_mask.shape[0] > 0 and crop_mask.shape[1] > 0:
                cv2.resize(crop_mask, (100, 100))
                cv2.imshow("CROPMASK", crop_mask)
                cv2.imshow("PRECISECROP", precise_crop)

        cv2.imshow("CROP", crop_img)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

def main(args=None):
    spinUntilKeyboardInterrupt(args, SignRecognitionNode)


if __name__ == '__main__':
    main()
