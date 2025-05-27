import rclpy
import rclpy.node
import cv2
import numpy as np

from enum import Enum
from std_msgs.msg import Int64
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from turtlebot_pastry._stop import spinUntilKeyboardInterrupt
from skimage.metrics import structural_similarity


class SignRecognitionNode(rclpy.node.Node):

    class SignType(Enum):
        PARKING = 0
        GO_STRAIGHT = 1
        TURN_LEFT = 2
        TURN_RIGHT = 3
        CROSSWALK = 4


    def __init__(self):
        super().__init__('SignRecognitionNode')

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.params = {
            'lower_bound' : [90,200,30],
            'upper_bound' : [102,255,200],
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
        timer_period = 0.1  # seconds
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
        crop_img = crop_img[145:280]
        cv2.imshow("IMG", crop_img)

        img_width = crop_img.shape[1]
        img_height = crop_img.shape[0]

        # convert to HSV
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, lower_bound, upper_bound)

        cv2.imshow("M0", mask)

        # blurring mask to remove artifacts
        scalar = 8
        mask = cv2.resize(mask, (img_width//scalar, img_height//scalar))
        mask = cv2.resize(mask, (img_width, img_height), interpolation=cv2.INTER_NEAREST)

            # do fine crop
        sensitivity = 140
        threshold = 1800

        bright_mask =  cv2.inRange(mask, sensitivity, 255)
        row_counts = np.sum(bright_mask, axis=1) // threshold
        col_counts = np.sum(bright_mask, axis=0) // threshold

        rows_nz = np.nonzero(row_counts)[0] - 1
        cols_nz = np.nonzero(col_counts)[0] - 1
        cv2.imshow("M1", mask)


        if np.sum(rows_nz) > 0 and sum(cols_nz) > 0:       # Only do when blue is found
            padding = 5
            mask = mask[max(0, rows_nz[0] - padding) : min(img_height, rows_nz[-1] + padding), max(0, cols_nz[0] - padding) : min(img_width, cols_nz[-1] + (padding // 1))]
            precise_crop = crop_img[max(0, rows_nz[0] - padding) : min(img_height, rows_nz[-1] + padding), max(0, cols_nz[0] - padding) : min(img_width, cols_nz[-1] + (padding // 1))]
            cv2.imshow("M2", mask)

            cv2.imshow("I", precise_crop)
            #hsv_img = cv2.cvtColor(precise_crop, cv2.COLOR_BGR2HSV)

            # find blue
            #mask2 = cv2.inRange(img_resize, lower_bound, upper_bound)
            
            if mask.shape[0] > 0 and mask.shape[1] > 0:
                precise_crop2 = cv2.resize(precise_crop, (100, 100))
                #compare to test images
                scores = []
                for i in self.image_list:
                #for i in self.crop_list:
                    #i = cv2.resize(cv2.Canny(i, 50, 200), (100,100))
                    scores.append(structural_similarity(cv2.cvtColor(i, cv2.COLOR_BGR2GRAY), cv2.cvtColor(precise_crop2, cv2.COLOR_BGR2GRAY), gaussian_weights=True, multichannel=False))
                    
                    #scores.append(structural_similarity(i, edged, gaussian_weighcrop, lower_bound, upper_bound), gaussian_weights=True, multichannel=True))

                scores = np.array(scores)
                #self.get_logger().info(str(scores))


                #find best match
                i = np.argmax(scores)
                if scores[i] > 0.0:
                    msg = Int64()
                    msg.data = int(i)
                    self.publisher_.publish(msg)
                    t = (self.SignType(i))
                    self.get_logger().info(str(t))
                
                #cv2.imshow("Edged", edged)

                #cv2.imshow("CROPMASK", crop_mask)
                #cv2.imshow("PRECISECROP2", precise_crop2)

            
        #cv2.imshow("CROP", crop_img)

        cv2.waitKey(1)

def main(args=None):
    spinUntilKeyboardInterrupt(args, SignRecognitionNode)


if __name__ == '__main__':
    main()
