import time
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class Worker(Node):
    def __init__(self):
        super().__init__("worker_node")

        pkg_path = get_package_share_directory("autorace_core_comand13")
        mask_path = os.path.join(pkg_path, "resources", "mask.png")
        self.get_logger().info(f"Mask path: {mask_path}")
        mask_ref = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        if mask_ref is None:
            self.get_logger().error("Mask not found in the path")
            return
        _, self.soft_mask_ref = cv2.threshold(mask_ref, 127, 255, cv2.THRESH_BINARY)
        self.get_logger().info("Mask succesfully readed")

        self.bridge = CvBridge()

        self.team_aruco_publisher = self.create_publisher(Float32, "/team/aruco", 10)
        self.team_arrow_publisher = self.create_publisher(String, "/team/arrow", 10)
        self.team_start_publisher = self.create_publisher(String, "/team/start", 10)
        self.team_depth_publisher = self.create_publisher(String, "/team/depth", 10)

        self.commands_sub = self.create_subscription(String, "/team/commands", self.commands_callback, 10)
        self.color_sub = self.create_subscription(Image, "/color/image", self.color_callback, 10)
        self.depth_subscriber = self.create_subscription(PointCloud2, '/depth/points', self.depth_callback, 10)
        self.depth = None
        self.safe_distance = 0.4
        self.img = None
        self.img_hsv = None

        self.timer = self.create_timer(0.1, self.loop)
        self.detect_aruco = False
        self.detect_start = True
        self.detect_arrow = False
        self.detect_depth = False

    def loop(self):
        if self.detect_start:
            green_mask = self.get_green()
            if green_mask is not None:
                self.get_logger().info(f"Detected group of green pixels with size {np.count_nonzero(green_mask)}")
                if np.count_nonzero(green_mask) > 1000:
                    self.get_logger().info("Sending start message")
                    start_msg = String()
                    start_msg.data = "start"
                    self.team_start_publisher.publish(start_msg)

        if self.detect_aruco:
            aruco = self.check_aruco()
            if aruco is not None:
                aruco_msg = Float32()
                aruco_msg.data = aruco
                self.get_logger().info("Sending aruco message")
                self.team_aruco_publisher.publish(aruco_msg)

        if self.detect_arrow:
            sign = self.get_sign()
            if sign is not None:
                sign_msg = String()
                sign_msg.data = sign[0]
                self.get_logger().info("Sending arrow message")
                self.team_arrow_publisher.publish(sign_msg)

        if self.detect_depth:
            if self.depth is not None:
                depth_msg = String()
                depth_msg.data = self.depth
                self.team_depth_publisher.publish(depth_msg)

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
            self.get_logger().info(f"Detected ArUco marker with id: {ids} -> value: {res:.3f}")
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

        res = "left" if iou > 0.85 else "right", iou
        self.get_logger().info(f"Detected arrow sign: {res} with IoU for left arrow {iou:.3f}")
        
        return res

    def get_green(self):
        if self.img_hsv is not None:        
            mask = (self.img_hsv[..., 1] >= 0.8)
            mask &= (self.img_hsv[..., 0] >= np.deg2rad(105)) & (self.img_hsv[..., 0] <= np.deg2rad(135))
            return mask
        return None

    def color_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.get_green or self.get_sign:
            self.img_hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV).astype(np.float32)
            self.img_hsv[..., 0] = self.img_hsv[..., 0] / 179 * np.pi * 2
            self.img_hsv[..., 1:] /= 255
        else:
            self.img_hsv = None

        # cv2.imshow("Color Image", self.img)
        # cv2.waitKey(1)

    def depth_callback(self, msg):
        if not self.detect_depth:
            return

        scan = msg

        start_idx = round(scan.height * 0.4) * scan.width + round(scan.width * 0.1)
        finish_idx = round(scan.height * 0.6) * scan.width + round(scan.width * 0.66)
        # index = (scan.width * scan.height) // 2 + (scan.width // 2)
        points = list(pc2.read_points(scan, field_names=("x", "y", "z"), skip_nans=True))
        points = list(map(list, points))

        if len(points) <= finish_idx - start_idx:
            return

        center_points = np.array(points[start_idx:finish_idx])
        # print(center_points)
        dist = np.min(center_points[:, 0])

        if dist > (self.safe_distance):
            self.depth = "front"
        elif dist > 0 and dist < (self.safe_distance):
            self.depth = "stop"
        else:
            self.depth = "stop"

    def commands_callback(self, msg):
        com = msg.data
        self.get_logger().info(f"Received command \"{com}\"")
        for icom in com.split("|"):
            if icom == "startoff":
                self.detect_start = False
                self.get_logger().info("Start green light detection turned off")
            elif icom == "arrowon":
                self.detect_arrow = True
                self.get_logger().info("Arrow detection turned on")
            elif icom == "arrowoff":
                self.detect_arrow = False
                self.get_logger().info("Arrow detection turned off")
            elif icom == "depthon":
                self.detect_depth = True
                self.get_logger().info("Depth detection turned on")
            elif icom == "depthoff":
                self.detect_depth = False
                self.get_logger().info("Depth detection turned off")
            elif icom == "arucoon":
                self.detect_aruco = True
                self.get_logger().info("ArUco detection turned on")
            elif icom == "arucooff":
                self.detect_aruco = False
                self.get_logger().info("ArUco detection turned off")

    def stop(self):
        self.detect_aruco = False
        self.detect_arrow = False
        self.detect_start = False
        self.detect_depth = False
        time.sleep(0.5)
        self.timer.cancel()
        self.timer.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = Worker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user")
        node.stop()
    except Exception as e:
        node.get_logger().error(f"Node error: {str(e)}")
    finally:
        node.destroy_node()
        time.sleep(1)
        rclpy.shutdown()


if __name__ == "__main__":
    main()

