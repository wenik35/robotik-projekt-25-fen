import rclpy
import rclpy.node
import cv2
import numpy as np
import time

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

    def __init__(self):
        super().__init__('SignRecognitionNode')

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.params = {
            'lower_bound' : [100,170,120],
            'upper_bound' : [110,245,255],
            'scalar' : 8,
            'padding' : 8,
            'crop_L' : 444,
            'crop_R' : 640,
            'crop_B' : 240,
            'crop_T' : 100
        }

        width = 640
        height = 480
        new_width = 1280
        new_height = 960
        
        v = 480.0

        K = np.array([[v, 0.0, width / 2],
                      [0.0, v, height / 2],
                      [0.0, 0.0, 1.0]])
        D = np.array([[-0.3], [0.1], [0.0], [0.0]])

        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, 
                                                                       D, 
                                                                       (width, height),
                                                                       np.eye(3),
                                                                       balance = 1,
                                                                       new_size = (new_width, new_height),
                                                                       fov_scale = 1.4)
        
        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(K, 
                                                                   D, 
                                                                   np.eye(3), 
                                                                   new_K, ((new_width, new_height)),
                                                                   cv2.CV_16SC2)

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
        timer_period = 0.05  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

        image_list = []

        image_list.append(cv2.resize(cv2.imread("./Media/Parking2.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/GoStraight2.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/TurnLeft2.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/TurnRight2.png"), (100, 100)))

        self.image_list = image_list
        self.sign_List = []

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
        #cv2.imshow("IMG", self.img_cv)

    def to_binary(self, img):
        lower_bound = self.get_parameter('lower_bound').get_parameter_value().integer_array_value
        lower_bound = np.array(lower_bound, dtype = "uint8")
        upper_bound = self.get_parameter('upper_bound').get_parameter_value().integer_array_value
        upper_bound = np.array(upper_bound, dtype = "uint8")

        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        return cv2.inRange(hsv_img, lower_bound, upper_bound)
    
    def whiteBalance(self, img):
        exp_grey = 120
        b, g, r = cv2.split(img)
        b = cv2.add(b, 10)
        g = cv2.subtract(g, 4)
        r = cv2.add(r, 8)
        img = cv2.merge([b, g, r])
        img = img.astype(np.float32)
        avg_b = np.mean(img[:, :, 0])
        avg_g = np.mean(img[:, :, 1])
        avg_r = np.mean(img[:, :, 2])

        avg_grey = (avg_g + avg_b + avg_r) / 3
        avg_grey *= exp_grey / avg_grey 

        scale_b = (avg_grey / avg_b) * 1.08
        scale_g = (avg_grey / avg_g) * 0.97
        scale_r = (avg_grey / avg_r)

        img[:, :, 0] *= scale_b
        img[:, :, 1] *= scale_g
        img[:, :, 2] *= scale_r
        img = np.clip(img, 0, 255).astype(np.uint8)
        return img
    
    def adjustSign(self, img):
        img = img.astype(np.float32)
        current_brightness = img.mean()
        brightness_scale = 120 / current_brightness
        img *= brightness_scale
        b, g, r = cv2.split(img)
        current_means = np.array([r.mean(), g.mean(), b.mean()])
        target_means = np.array((0.87, 0.94, 1.2)) * img.mean()
        scale_factors = target_means / current_means

        r *= scale_factors[0]
        g *= scale_factors[1]
        b *= scale_factors[2]

        adjusted = cv2.merge([b, g, r])
        adjusted = np.clip(adjusted, 0, 255).astype(np.uint8)

        return adjusted
    
    def timer_callback(self):
        temp = self.img_cv
        scalar = self.get_parameter('scalar').get_parameter_value().integer_value
        padding = self.get_parameter('padding').get_parameter_value().integer_value
        crop_L = self.get_parameter('crop_L').get_parameter_value().integer_value
        crop_R = self.get_parameter('crop_R').get_parameter_value().integer_value
        crop_B = self.get_parameter('crop_B').get_parameter_value().integer_value
        crop_T = self.get_parameter('crop_T').get_parameter_value().integer_value
        cv2.imshow("N", self.img_cv)

        temp = self.whiteBalance(temp)
        temp = cv2.remap(temp,
                         self.map1,
                         self.map2,
                         interpolation = cv2.INTER_LINEAR,
                         borderMode = cv2.BORDER_CONSTANT)
        #cv2.imshow("Noo", temp)

        
        img_v = temp.copy()
        cv2.rectangle(img_v, (crop_L, crop_B), (crop_R, crop_T), (0, 240, 0), 2)
        
        # cropping image
        crop_img = temp[:, crop_L:crop_R] # TODO: Optimize cropping
        crop_img = crop_img[crop_T:crop_B]
        #crop_img = self.whiteBalance(crop_img)
        cv2.imshow("C", crop_img)

        img_width = crop_img.shape[1]
        img_height = crop_img.shape[0]

        # convert to binary
        mask = self.to_binary(crop_img)

        cv2.imshow("M0", mask)

        # scaling mask to remove artifacts
        maskR = cv2.resize(mask, (img_width//scalar, img_height//scalar))
        maskR = cv2.resize(maskR, (img_width, img_height), interpolation=cv2.INTER_NEAREST)

        # do fine crop
        sensitivity = 140
        threshold = 3000

        bright_mask =  cv2.inRange(maskR, sensitivity, 255)
        row_counts = np.sum(bright_mask, axis=1) // threshold
        col_counts = np.sum(bright_mask, axis=0) // threshold

        #print(col_counts)

        rows_nz = np.nonzero(row_counts)[0] - 1
        cols_nz = np.nonzero(col_counts)[0] - 1
        cv2.imshow("M1", maskR)

        if np.sum(rows_nz) > 0 and sum(cols_nz) > 0:       # Only do when blue is found
            maskR = maskR[max(0, rows_nz[0] - padding) : min(img_height, rows_nz[-1] + padding), max(0, cols_nz[0] - padding) : min(img_width, cols_nz[-1] + (padding))]

            top = max(0, rows_nz[0] - padding)
            bottom = min(img_height, rows_nz[-1] + padding)
            left = max(0, cols_nz[0] - padding)
            right = min(img_width, cols_nz[-1] + padding)

            square_size = max(bottom - top, right - left)

            center_row = (top + bottom) // 2
            center_col = (left + right) // 2

            half_size = square_size // 2
            new_top = max(0, center_row - half_size)
            new_bottom = min(img_height, center_row + half_size)
            new_left = max(0, center_col - half_size)
            new_right = min(img_width, center_col + half_size)

            if new_bottom - new_top < square_size:
                if new_top == 0:
                    new_bottom = min(img_height, new_top + square_size)
                elif new_bottom == img_height:
                    new_top = max(0, new_bottom - square_size)

            if new_right - new_left < square_size:
                if new_left == 0:
                    new_right = min(img_width, new_left + square_size)
                elif new_right == img_width:
                    new_left = max(0, new_right - square_size)

            precise_crop = crop_img[new_top:new_bottom, new_left:new_right]
            max_b = np.max(precise_crop)
            precise_crop = ((precise_crop.astype(np.float32) / np.float32(max_b)) * 255.0).astype(np.uint8)
            #precise_crop = self.adjustSign(precise_crop)

            #cv2.rectangle(img_v, (crop_L + new_left, crop_T + new_bottom), (crop_L + new_right, crop_T + new_top), (0, 0, 240), 2)
            #cv2.rectangle(crop_img, (new_left, new_bottom), (new_right, new_top), (0, 0, 240), 2)

            #precise_crop = crop_img[max(0, rows_nz[0] - padding) : min(img_height, rows_nz[-1] + padding), max(0, cols_nz[0] - padding) : min(img_width, cols_nz[-1] + (padding))]
            #crop2 = crop_img[max(0, rows_nz[0] - buffer) : min(img_height, rows_nz[-1] + buffer), max(0, cols_nz[0] - buffer) : min(img_width, cols_nz[-1] + (buffer))]

            cv2.imshow("M2", maskR)

            #cv2.imshow("I", precise_crop)
            #cv2.imshow("J", crop2)

            if maskR.shape[0] > 1 and maskR.shape[1] > 1:
                precise_crop2 = cv2.resize(precise_crop, (100, 100))
                pcg = cv2.cvtColor(precise_crop2, cv2.COLOR_BGR2GRAY)
                cv2.imshow("GGG", pcg)
                #compare to test images
                scores = []
                for i in self.image_list:
                #for i in self.crop_list:
                    #i = cv2.resize(cv2.Canny(i, 50, 200), (100,100))
                    scores.append(structural_similarity(cv2.cvtColor(i, cv2.COLOR_BGR2GRAY), cv2.cvtColor(precise_crop2, cv2.COLOR_BGR2GRAY), gaussian_weights=True, multichannel=False))

                #self.get_logger().info(str(scores))

                scores = np.array(scores)
                #find best match
                i = np.argmax(scores)
                t = (self.SignType(i))
                if scores[i] > 0.44:
                    msg = Int64()
                    msg.data = int(i)
                    self.publisher_.publish(msg)
                    self.get_logger().info(str(t.name) + " " + str(100 * scores[i])[:5] + "%")
                else:
                    print(str(t.name) + " " + str(100 * scores[i])[:5] + "%")

                cv2.imshow("T", precise_crop2)
        cv2.imshow("V", img_v)
        cv2.waitKey(1)

def main(args=None):
    spinUntilKeyboardInterrupt(args, SignRecognitionNode)

if __name__ == '__main__':
    main()
