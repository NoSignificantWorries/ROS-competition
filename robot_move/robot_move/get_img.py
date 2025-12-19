import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class Worker(Node):
    def __init__(self):
        super().__init__('worker_node')
        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.color_sub = self.create_subscription(Image, '/color/image', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth/image', self.depth_callback, 10)
        self.depth_map = None
        self.img = None

        # self.timer = self.create_timer(0.1, self.move)
        self.x = 0

    def move(self):
        msg = Twist()
        msg.linear.x = np.cos(self.x)
        self.x = self.x + 0.1
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def color_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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

