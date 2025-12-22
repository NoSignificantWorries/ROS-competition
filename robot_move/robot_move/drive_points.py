import os
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

SPAWN_X_WORLD = 1.15
SPAWN_Y_WORLD = -2.2

WAYPOINTS_WORLD = [
    (1.15, -2.20),
    (2.09, -2.13),
    (2.16, -1.94),
    (2.16, -1.19),
]


def world_to_odom(x_world, y_world):
    x_odom = x_world - SPAWN_X_WORLD
    y_odom = y_world - SPAWN_Y_WORLD
    return x_odom, y_odom


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        pkg_path = get_package_share_directory("robot_move")
        path_file = os.path.join(pkg_path, "resources", "path.json")

        with open(path_file, "r") as file:
            self.path = json.load(file)
        print(self.path)
        self.points = self.path["points"]
        self.points_labels = list(self.points.keys())
        self.pidx = 0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_point = self.points["start"]

        self.have_pose = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.timer = self.create_timer(0.05, self.control_step)

    def odom_callback(self, msg: Odometry):
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.yaw = 2 * math.atan2(q.z, q.w)
            self.have_pose = True

    def control_step(self):
        if not self.have_pose:
            self.get_logger().warn('⌛️ Ждём первые данные /odom...')
            return

        if self.pidx >= len(self.points_labels):
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        if self.points_labels[self.pidx] == "fork":
            tmp_points = self.points["fork"]["w1"]
            self.points.update(tmp_points)
            self.points_labels = self.points_labels[:self.pidx] + list(tmp_points.keys()) + self.points_labels[self.pidx + 1:]
            print(self.points)
            self.current_point = self.points[self.points_labels[self.pidx]]

        tx, ty = world_to_odom(*self.current_point)
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            self.get_logger().info( f'✅ Достигли WP #{self.points_labels[self.pidx]}: ({tx:.2f},{ty:.2f})')
            self.pidx += 1
            if self.pidx < len(self.points_labels):
                self.current_point = self.points[self.points_labels[self.pidx]]
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = self._normalize_angle(target_yaw - self.yaw)

        K_lin = 0.6
        K_ang = 1.1

        v = K_lin * dist
        w = K_ang * yaw_err

        v = max(min(v, 0.4), -0.4)
        w = max(min(w, 1.0), -1.0)

        if abs(yaw_err) > math.radians(40):
            v = 0.0

        if self.points_labels[self.pidx].startswith("l") and yaw_err >= 0.05:
            v = 0.0

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
        main()

