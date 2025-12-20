import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge


class Worker(Node):
    def __init__(self):
        super().__init__('worker_node')
        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.finish_publisher_ = self.create_publisher(String, "/robot_finish", 10)

        self.color_sub = self.create_subscription(Image, '/color/image', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth/image', self.depth_callback, 10)
        self.depth_map = None
        self.img = None
        self.img_hsv = None

        self.timer = self.create_timer(0.1, self.loop)
        self.move = False
        self.time_start = -1
        self.time_stop = 2.0

    def loop(self):
        green_mask = self.get_green()

        if self.time_start < 0 and green_mask is not None and np.count_nonzero(green_mask) > 1000:
            self.move = True

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        if self.move and self.time_start < self.time_stop:
            msg.linear.x = 0.2
            self.time_start += 0.1
        elif self.move and self.time_start >= self.time_stop:
            self.move = False
            self.finish()

        self.publisher_.publish(msg)

    def finish(self):
        msg = String()
        msg.data = "test"

        self.finish_publisher_.publish(msg)

    def get_green(self):
        if self.img_hsv is not None:        
            mask = (self.img_hsv[..., 1] >= 0.8)
            mask &= (self.img_hsv[..., 0] >= np.deg2rad(105)) & (self.img_hsv[..., 0] <= np.deg2rad(135))
            return mask
        return None

    def get_yellow(self):
        if self.img_hsv is not None:        
            mask = (self.img_hsv[..., 1] >= 0.8)
            mask &= (self.img_hsv[..., 0] >= np.deg2rad(50)) & (self.img_hsv[..., 0] <= np.deg2rad(70))
            return mask
        return None

    def color_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV).astype(np.float32)
        self.img_hsv[..., 0] = self.img_hsv[..., 0] / 179 * np.pi * 2
        self.img_hsv[..., 1:] /= 255

        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(self.img, corners, ids)
                
            print(f"Найдено маркеров: {len(ids)}")
            print(f"ID маркеров: {ids.flatten()}")
            print("Res:", np.sqrt(ids))

        cv2.imshow('Color Image', self.img)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        
        depth_data = np.zeros(cv_depth.shape, dtype=np.float32)

        mask = ~np.isinf(cv_depth)
        if np.any(mask):
            depth_data[mask] = cv_depth[mask]
            self.depth_map = depth_data
            np.save("depth.npy", self.depth_map)
        else:
            self.depth_map = None


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

