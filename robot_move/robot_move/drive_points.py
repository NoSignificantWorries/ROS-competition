import os
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

SPAWN_X_WORLD = 1.15
SPAWN_Y_WORLD = -2.2


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
        self.events = self.path["events"]
        self.points = self.path["points"]
        self.points_labels = list(self.points.keys())
        self.pidx = 0

        self.finish_publisher = self.create_publisher(String, '/robot_finish', 10)
        self.aruco_pub = self.create_publisher(Float32, '/mission_aruco', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.team_aruco_sub = self.create_subscription(Float32, "/team/aruco", self.aruco_callback, 10)
        self.team_arrow_sub = self.create_subscription(String, "/team/arrow", self.arrow_callback, 10)
        self.team_start_sub = self.create_subscription(String, "/team/start", self.start_callback, 10)
        self.team_depth_sub = self.create_subscription(String, "/team/depth", self.depth_callback, 10)

        self.commands_publisher = self.create_publisher(String, "/team/commands", 10)

        self.current_point = self.points["start"]
        self.state = "start"
        self.current_direct = "w1"

        self.aruco = None
        self.start = None
        self.arrow = None
        self.depth = None
        self.arrow_activated = False
        self.aruco_activated = False
        self.depth_on = False

        self.pipe_i = 0
        self.pv = 0.4
        self.s = 0.05 * self.pv
        self.R = 2.40
        self.cp = np.pi * self.R / (2 * self.s)
        self.pw = self.pv / self.R

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

    def aruco_callback(self, msg):
        self.aruco = msg.data

    def arrow_callback(self, msg):
        self.arrow = msg.data

    def start_callback(self, msg):
        self.start = msg.data

    def depth_callback(self, msg):
        self.depth = msg.data

    def send_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.commands_publisher.publish(msg)

    def control_step(self):
        if self.state == "start":
            if self.start and self.start == "start":
                self.state = "drive"
                self.send_cmd("startoff")
        elif self.state == "arrow":
            if not self.arrow_activated:
                self.send_cmd("arrowon")
                self.arrow_activated = True
                return
            if self.arrow is None:
                return
            if self.arrow == "left":
                self.send_cmd("arrowoff")
                self.current_direct = "w1"
                self.state = "drive"
            elif self.arrow == "right":
                self.send_cmd("arrowoff")
                self.current_direct = "w2"
                self.state = "drive"
        elif self.state == "aruco":
            if not self.aruco_activated:
                self.send_cmd("arucoon")
                self.aruco_activated = True
                return
            if self.aruco is not None:
                self.send_cmd("arucooff")
                msg = Float32()
                msg.data = self.aruco
                self.aruco_pub.publish(msg)
                self.state = "drive"
        elif self.state == "pipe":
            twist = Twist()
            twist.linear.x = self.pv
            twist.angular.z = self.pw
            self.cmd_pub.publish(twist)
            if self.pipe_i > self.cp:
                self.state = "drive"
            self.pipe_i += 1
        elif self.state == "depthon":
            self.depth_on = True
            self.send_cmd("depthon")
            self.state = "drive"
        elif self.state == "depthoff":
            self.depth_on = False
            self.send_cmd("depthoff")
            self.state = "drive"
        elif self.state == "drive":
            if not self.have_pose:
                self.get_logger().warn('Waiting data from /odom...')
                return

            if self.pidx >= len(self.points_labels):
                twist = Twist()
                self.cmd_pub.publish(twist)
                self.state = "finish"
                return

            if self.points_labels[self.pidx] == "fork":
                tmp_points = self.points["fork"][self.current_direct]
                self.points.update(tmp_points)
                self.points_labels = self.points_labels[:self.pidx] + list(tmp_points.keys()) + self.points_labels[self.pidx + 1:]
                del self.points["fork"]
                self.current_point = self.points[self.points_labels[self.pidx]]

            tx, ty = world_to_odom(*self.current_point)
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)

            if dist < 0.05:
                self.get_logger().info( f'Reached WP {self.points_labels[self.pidx]}: ({tx:.2f},{ty:.2f})')
                if self.points_labels[self.pidx] in self.events.keys():
                    self.state = self.events[self.points_labels[self.pidx]]
                    twist = Twist()
                    self.cmd_pub.publish(twist)
                self.pidx += 1
                if self.pidx < len(self.points_labels):
                    self.current_point = self.points[self.points_labels[self.pidx]]
                    twist = Twist()
                    self.cmd_pub.publish(twist)
                return

            target_yaw = math.atan2(dy, dx)
            yaw_err = self._normalize_angle(target_yaw - self.yaw)

            K_lin = 0.4 if self.depth_on else 0.8
            K_ang = 1.1

            v = K_lin * dist
            w = K_ang * yaw_err

            v = max(min(v, 0.6), -0.6)
            w = max(min(w, 1.0), -1.0)

            if dist > 0.4 and not self.depth_on:
                v = 0.6

            if abs(yaw_err) > math.radians(40):
                v = 0.0

            if self.points_labels[self.pidx].startswith("l") and abs(yaw_err) >= 0.05:
                v = 0.0

            if self.depth_on:
                if self.depth is not None:
                    if self.depth == "stop":
                        v = 0.0

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.cmd_pub.publish(twist)
        else:
            msg = String()
            msg.data = "Command#13"
            self.finish_publisher.publish(msg)

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

