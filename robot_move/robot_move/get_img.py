from ament_index_python.packages import get_package_share_directory
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge


class Worker(Node):
    def __init__(self):
        super().__init__('worker_node')

        pkg_path = get_package_share_directory('robot_move')
        mask_path = os.path.join(pkg_path, 'resources', 'masks', 'mask.png')
        mask_ref = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        _, self.soft_mask_ref = cv2.threshold(mask_ref, 127, 255, cv2.THRESH_BINARY)

        self.bridge = CvBridge()

        self.team_aruco_publisher = self.create_publisher(Float32, "/team/aruco", 10)
        self.team_arrow_publisher = self.create_publisher(String, "/team/arrow", 10)
        self.team_start_publisher = self.create_publisher(String, "/team/start", 10)

        self.commands_sub = self.create_subscription(Image, '/team/commands', self.commands_callback, 10)
        self.color_sub = self.create_subscription(Image, '/color/image', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth/image', self.depth_callback, 10)
        self.depth_map = None
        self.img = None
        self.img_hsv = None

        self.timer = self.create_timer(0.1, self.loop)
        self.detect_aruco = False
        self.detect_start = False
        self.detect_arrow = False

    def loop(self):
        if self.detect_start:
            green_mask = self.get_green()
            if green_mask is not None and np.count_nonzero(green_mask) > 1000:
                start_msg = String()
                start_msg.data = "start"
                self.team_start_publisher.publish(start_msg)

        if self.detect_aruco:
            aruco = self.check_aruco()
            if aruco is not None:
                aruco_msg = Float32()
                aruco_msg.data = aruco
                self.team_aruco_publisher.publish(aruco_msg)

        if self.detect_arrow:
            sign = self.get_sign()
            if sign is not None:
                sign_msg = String()
                sign_msg.data = sign[0]
                self.team_arrow_publisher.publish(sign_msg)

    def check_aruco(self):
        if self.img is None:
            return None

        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(self.img, corners, ids)
            res = np.sqrt(ids.flatten()[0])
            return res

        return None
    
    def get_sign(self):
        if self.img_hsv is None:
            return None

        blue_mask = (self.img_hsv[..., 1] > 0.9) & (self.img_hsv[..., 0] >= np.deg2rad(200)) & (self.img_hsv[..., 0] <= np.deg2rad(260))

        if not np.any(blue_mask):
            return None

        y_coords, x_coords = np.where(blue_mask)
        bbox = (x_coords.min(), y_coords.min(), 
                x_coords.max() - x_coords.min(), 
                y_coords.max() - y_coords.min())

        if bbox[2] < 100 and bbox[3] < 100:
            return None

        sign = blue_mask[bbox[1]:bbox[1] + bbox[3] + 1,
                           bbox[0]:bbox[0] + bbox[2] + 1].astype(np.uint8)

        resized_sign = cv2.resize(sign, (100, 100), interpolation=cv2.INTER_NEAREST)

        intersection = np.logical_and(self.soft_mask_ref > 0, resized_sign > 0).sum()
        union = np.logical_or(self.soft_mask_ref > 0, resized_sign > 0).sum()
        
        iou = intersection / (union + 1e-10)
        
        return "left" if iou > 0.85 else "right", iou

    def get_green(self):
        if self.img_hsv is not None:        
            mask = (self.img_hsv[..., 1] >= 0.8)
            mask &= (self.img_hsv[..., 0] >= np.deg2rad(105)) & (self.img_hsv[..., 0] <= np.deg2rad(135))
            return mask
        return None

    def color_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV).astype(np.float32)
        self.img_hsv[..., 0] = self.img_hsv[..., 0] / 179 * np.pi * 2
        self.img_hsv[..., 1:] /= 255

        cv2.imshow('Color Image', self.img)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        
        depth_data = np.zeros(cv_depth.shape, dtype=np.float32)

        mask = ~np.isinf(cv_depth)
        if np.any(mask):
            depth_data[mask] = cv_depth[mask]
            self.depth_map = depth_data
            # np.save("depth.npy", self.depth_map)
        else:
            self.depth_map = None

    def commands_callback(self, msg):
        com = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = Worker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

