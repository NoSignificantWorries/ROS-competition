#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SPAWN_X_WORLD = 1.15
SPAWN_Y_WORLD = -2.2

WAYPOINTS_WORLD = [
    (1.15, -2.20),
    (1.86, -2.20),
    (2.05, -2.13),
    (2.15, -1.98),
    (2.15, -1.24),
]

def world_to_odom(x_world, y_world):
    x_odom = x_world - SPAWN_X_WORLD
    y_odom = y_world - SPAWN_Y_WORLD
    return x_odom, y_odom

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # конвертируем все waypoints сразу в odom
        self.waypoints_odom = [
            world_to_odom(x, y) for (x, y) in WAYPOINTS_WORLD
        ]
        self.wp_idx = 0

        self.get_logger().info(
            f'🎯 {len(self.waypoints_odom)} waypoints (world→odom) загружены'
        )

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
            self.get_logger().warn('⌛ Ждём первые данные /odom...')
            return

        if self.wp_idx >= len(self.waypoints_odom):
            # все точки пройдены — стоп
            twist = Twist()
            self.cmd_pub.publish(twist)
            return

        tx, ty = self.waypoints_odom[self.wp_idx]

        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        # порог достижения текущего waypoint
        if dist < 0.05:
            self.get_logger().info(
                f'✅ Достигли WP #{self.wp_idx}: ({tx:.2f},{ty:.2f})'
            )
            self.wp_idx += 1
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = self._normalize_angle(target_yaw - self.yaw)

        K_lin = 1.0
        K_ang = 1.0

        v = K_lin * dist
        w = K_ang * yaw_err

        v = max(min(v, 0.8), -0.8)
        w = max(min(w, 1.5), -1.5)

        # на очень резких поворотах сильнее режем линейную скорость, но не до нуля
        if abs(yaw_err) > math.radians(60):
            v *= 0.3


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
