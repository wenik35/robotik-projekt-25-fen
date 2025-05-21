from math import ulp
from os import chroot
import rclpy
import rclpy.node
import cv2
import numpy as np

from skimage.util.arraycrop import crop
from std_msgs.msg import Int64
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import SetParametersResult
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt
from skimage.metrics import structural_similarity

import array

class SignRecognitionNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('SignRecognitionNode')

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.params = {
            'lower_bound' : [85,200,20],
            'upper_bound' : [100,255,128],
            'scalar' : 30
        }

        for param_name, default_value in self.params.items():
            self.declare_parameter(param_name, default_value)

        for param_name in self.params.keys():
            self.params[param_name] = self.get_parameter(param_name).value

        self.img_cv = np.ones((480,640,3), dtype= "uint8")

        self.add_on_set_parameters_callback(self.parameter_callback)

        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Int64, 'sign_seen', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)


        image_list = []
        image_list.append(cv2.resize(cv2.imread("./Media/ParkPlatz100.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/GoStraight100.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/TurnLeft100.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/TurnRight100.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/ZebraStreifen100.png"), (100, 100)))


        self.crop_list = []
        lower_bound = np.array([140,55,0], dtype = "uint8")
        upper_bound = np.array([155,97,0], dtype = "uint8")

        for i in image_list:
            self.crop_list.append(cv2.inRange(i, lower_bound, upper_bound))

        self.image_list = image_list


        '''
        cv2.imwrite("test", image_list[0])
        cv2.imwrite("test_crop", crop_list[0])
        '''

    def parameter_callback(self, params):
        succ = True
        for param in params:
            if param.name in self.params:
                self.params[param.name] = param.value
                #self.get_logger().info(f"Parameter {param.name} updated to {self.params[param.name]}")
            else:
                succ = False
        return SetParametersResult(successful = succ)


    def scanner_callback(self, data):
        # convert message to opencv image
        self.img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')
        cv2.imshow("IMG", self.img_cv)

    def timer_callback(self):

        #print(self.img_cv)

        lower_bound = self.get_parameter('lower_bound').get_parameter_value().integer_array_value
        lower_bound = np.array(lower_bound, dtype = "uint8")
        upper_bound = self.get_parameter('upper_bound').get_parameter_value().integer_array_value
        upper_bound = np.array(upper_bound, dtype = "uint8")
        scalar = self.get_parameter('scalar').get_parameter_value().integer_value



        # cropping image
        crop_img = self.img_cv[:, 480:] # TODO: Optimize cropping
        crop_img = crop_img[150:350]

        img_width = crop_img.shape[1]
        img_height = crop_img.shape[0]

        # convert to HSV


        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # "Blurring" image
        img_resize = cv2.resize(hsv_img, (img_width//scalar, img_height//scalar))
        img_resize = cv2.resize(img_resize, (img_width, img_height))

        # find blue
        mask = cv2.inRange(img_resize, lower_bound, upper_bound)

        if np.amax(mask > 0):       # Only do when blue is found

            # do fine crop
            crop_up = 0
            crop_down = img_height
            crop_left = 0
            crop_right = img_width

            for i in range(img_height):
                #print(i)
                if(np.max(mask[i]) > 0):
                    crop_up = i
                    break

            for i in range(img_height-1, -1, -1):
                #print(i)
                if(np.max(mask[i]) > 0):
                    crop_down = i
                    break

            for i in range(img_width):
                #print(i)
                if(np.max(mask[:,i]) > 0):
                    crop_left = i
                    break

            for i in range(img_width-1, -1, -1):
                #print(i)
                #print(len(mask[0]))
                if(np.max(mask[:,i]) > 0):
                    crop_right = i
                    break

            #print("crop_up: ", crop_up)
            #print("crop_down: ", crop_down)
            #print("crop_left: ", crop_left)
            #print("crop_right: ", crop_right)

            buffer = 50
            crop_mask = mask[max(crop_up-buffer, 0) : min(crop_down + buffer, img_height), max(crop_left - buffer, 0) : min(crop_right + buffer, img_width)]
            precise_crop = crop_img[max(crop_up-buffer, 0) : min(crop_down, img_height), max(crop_left - buffer, 0) : min(crop_right + buffer, img_width)]

            cv2.imshow("MASK", mask)
            cv2.imshow("PRECISECROP", precise_crop)

            img_width = precise_crop.shape[1]
            img_height = precise_crop.shape[0]


            hsv_img = cv2.cvtColor(precise_crop, cv2.COLOR_BGR2HSV)

            # "Blurring" image
            scalar = 8
            img_resize = cv2.resize(hsv_img, (img_width//scalar, img_height//scalar))
            img_resize = cv2.resize(img_resize, (img_width, img_height))

            # find blue
            mask2 = cv2.inRange(img_resize, lower_bound, upper_bound)

            crop_up = 0
            crop_down = img_height
            crop_left = 0
            crop_right = img_width

            for i in range(img_height):
                #print(i)
                if(np.max(mask2[i]) > 0):
                    crop_up = i
                    break

            for i in range(img_height-1, -1, -1):
                #print(i)
                if(np.max(mask2[i]) > 0):
                    crop_down = i
                    break

            for i in range(img_width):
                #print(i)
                if(np.max(mask2[:,i]) > 0):
                    crop_left = i
                    break

            for i in range(img_width-1, -1, -1):
                #print(i)
                #print(len(mask2[0]))
                if(np.max(mask2[:,i]) > 0):
                    crop_right = i
                    break

            buffer = 10
            crop_mask = mask2[max(crop_up-buffer, 0) : min(crop_down + buffer, img_height), max(crop_left - buffer, 0) : min(crop_right + buffer, img_width)]
            precise_crop = precise_crop[max(crop_up-buffer, 0) : min(crop_down + buffer, img_height), max(crop_left - buffer, 0) : min(crop_right + buffer, img_width)]
            #cv2.imshow("MASK2", mask2)

            if crop_mask.shape[0] > 0 and crop_mask.shape[1] > 0:
                precise_crop = cv2.resize(precise_crop, (100, 100))
                edged = cv2.resize(cv2.Canny(precise_crop, 50, 200), (100,100))
                #compare to test images
                scores = []
                for i in self.image_list:
                #for i in self.crop_list:
                    #i = cv2.resize(cv2.Canny(i, 50, 200), (100,100))
                    scores.append(structural_similarity(cv2.cvtColor(i, cv2.COLOR_BGR2GRAY), cv2.cvtColor(precise_crop, cv2.COLOR_BGR2GRAY), gaussian_weights=True, multichannel=False))
                    #scores.append(structural_similarity(i, edged, gaussian_weight=False, multichannel=True))
                    #scores.append(structural_similarity(i, cv2.inRange(precise_crop, lower_bound, upper_bound), gaussian_weights=True, multichannel=True))

                scores = np.array(scores)

                #find best match
                i = np.argmax(scores)
                if scores[i] > 0.25:
                    msg = Int64()
                    msg.data = int(i)
                    self.publisher_.publish(msg)
                    print(i)

                print(scores)
                self.get_logger().info("SIGN FOUND!")
                #cv2.imshow("Edged", edged)
                
                #cv2.imshow("CROPMASK", crop_mask)
                #cv2.imshow("PRECISECROP2", precise_crop)

        #cv2.imshow("CROP", crop_img)

        #cv2.imshow("Resize", cv2.cvtColor(img_resize, cv2.COLOR_HSV2BGR))
        #cv2.waitKey(1)

def main(args=None):
    spinUntilKeyboardInterrupt(args, SignRecognitionNode)


if __name__ == '__main__':
    main()
