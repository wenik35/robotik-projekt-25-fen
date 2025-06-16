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

    def __init__(self):
        super().__init__('SignRecognitionNode')

        # init openCV-bridge
        self.bridge = CvBridge()

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.params = {
            'lower_bound' : [100,140,30],
            'upper_bound' : [125,230,120],
            'scalar' : 8,
            'padding' : 8,
            'crop_L' : 800,
            'crop_R' : 1100,
            'crop_B' : 450,
            'crop_T' : 320
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
        timer_period = 0.04  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

        image_list = []

        image_list.append(cv2.resize(cv2.imread("./Media/Parking2.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/GoStraight2.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/TurnLeft2.png"), (100, 100)))
        image_list.append(cv2.resize(cv2.imread("./Media/TurnRight2.png"), (100, 100)))

        for i in range(0, len(image_list)):
            image_list[i] = cv2.cvtColor(image_list[i], cv2.COLOR_BGR2GRAY)

            exp_grey = 128
            image_list[i] = image_list[i].astype(np.float32)
            avg_grey = np.mean(image_list[i])
            factor = exp_grey / avg_grey 
            image_list[i] *= factor
            image_list[i] = np.clip(image_list[i], 0, 255).astype(np.uint8)

        self.image_list = image_list

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

    def brightBalance(self, img):
        exp_grey = 128
        img = img.astype(np.float32)
        avg_grey = np.mean(img)
        factor = exp_grey / avg_grey 
        img *= factor
        img = np.clip(img, 0, 255).astype(np.uint8)
        return img

    def timer_callback(self):
        temp = self.img_cv
        scalar = self.get_parameter('scalar').get_parameter_value().integer_value
        padding = self.get_parameter('padding').get_parameter_value().integer_value
        crop_L = self.get_parameter('crop_L').get_parameter_value().integer_value
        crop_R = self.get_parameter('crop_R').get_parameter_value().integer_value
        crop_B = self.get_parameter('crop_B').get_parameter_value().integer_value
        crop_T = self.get_parameter('crop_T').get_parameter_value().integer_value
        #cv2.imshow("N", self.img_cv)

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
        #cv2.imshow("C", crop_img)

        img_width = crop_img.shape[1]
        img_height = crop_img.shape[0]

        # convert to binary
        mask = self.to_binary(crop_img)

        cv2.imshow("M0", mask)

        # scaling mask to remove artifacts
        maskR = cv2.resize(mask, (img_width//scalar, img_height//scalar))
        nz_rows = (maskR != 0).sum(axis=1) == 1         # rows with exactly one non-black pixel
        nz_cols = (maskR != 0).sum(axis=0) == 1         # cols with exactly one non-black pixel
        singleton  = (maskR != 0) & (nz_rows[:, None] | nz_cols[None, :])
        maskR[singleton] = 0
        maskR = cv2.resize(maskR, (img_width, img_height), interpolation=cv2.INTER_NEAREST)

        #print(col_counts)

        sensitivity = 120
        min_area = 1000
        max_area = 100 * 100

        # 1. Threshold to isolate bright regions
        bright_mask = cv2.inRange(maskR, sensitivity, 255)
        sensitivity = 120

        # 2. Find connected components
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(bright_mask)

        # 3. Filter components by size
        valid_components = []
        for i in range(1, num_labels):               # skip background (label 0)
            x, y, w, h, area = stats[i]

            # mean grey value inside this component
            comp_mask = (labels[y:y+h, x:x+w] == i)
            mean_val  = bright_mask[y:y+h, x:x+w][comp_mask].mean()

            if (min_area <= area <= max_area) and (mean_val >= sensitivity):
                valid_components.append((x, y, w, h, mean_val))

        # 4. If no valid components, return None
        if valid_components:
            # Pick the largest valid component (or first, or one nearest center — depends on your need)
            x, y, w, h, area = max(valid_components, key=lambda t: t[4])  # ← t[5] is mean brightness

            # 5.a Return bottom-left and top-right corners
            side = max(w, h)
            x -= (side - w) // 2
            y -= (side - h) // 2
            x = max(0, x)                       # keep inside image
            y = max(0, y)
            # (optionally clamp x+side ≤ width and y+side ≤ height if needed)

            # 5 b. update corners
            bottom_left = (x, y + side)
            top_right   = (x + side, y)

            maskR = cv2.cvtColor(maskR, cv2.COLOR_GRAY2BGR)
            cv2.rectangle(maskR, (bottom_left[0], bottom_left[1]), (top_right[0], top_right[1]), (0, 0, 240), 2)
            cv2.imshow("JJJJ", maskR)
            maskR = cv2.cvtColor(maskR, cv2.COLOR_BGR2GRAY)
            maskR = maskR[max(0, bottom_left[1] - padding) : min(img_height, top_right[1] + padding), max(0, bottom_left[0] - padding) : min(img_width, top_right[0] + (padding))]

            precise_crop = crop_img[top_right[1]:bottom_left[1], bottom_left[0]:top_right[0]]
            max_b = np.max(precise_crop)
            precise_crop = ((precise_crop.astype(np.float32) / np.float32(max_b)) * 255.0).astype(np.uint8)

            cv2.rectangle(img_v, (crop_L + bottom_left[0], crop_T + bottom_left[1]), (crop_L + top_right[0], crop_T + top_right[1]), (0, 0, 240), 2)

            #cv2.imshow("I", precise_crop)
            #cv2.imshow("J", crop2)
            #print(precise_crop.shape)

            precise_crop2 = cv2.resize(precise_crop, (100, 100))
            pcg = cv2.cvtColor(precise_crop2, cv2.COLOR_BGR2GRAY)
            #cv2.imshow("GGG", pcg)
            pcg = self.brightBalance(pcg)
            #cv2.imshow("HHH", pcg)

            #compare to test images
            scores = []
            for i in self.image_list:
                scores.append(structural_similarity(i, pcg, gaussian_weights=True, multichannel=False))

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

        cv2.imshow("V", img_v)

        cv2.waitKey(1)

def main(args=None):
    spinUntilKeyboardInterrupt(args, SignRecognitionNode)

if __name__ == '__main__':
    main()
