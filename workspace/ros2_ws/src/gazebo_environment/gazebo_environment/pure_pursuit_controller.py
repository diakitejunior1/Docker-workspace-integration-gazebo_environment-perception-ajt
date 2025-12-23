import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Point
from visualization_msgs.msg import Marker
import csv
import os
import math
from ament_index_python.packages import get_package_share_directory


class PurePursuitAdaptive(Node):

    def __init__(self):
        super().__init__('pure_pursuit_adaptive')

        # Base lookahead parameters
        self.L0 = 1.0        # minimum lookahead (m)
        self.k = 0.5         # gain: how lookahead grows with velocity

        self.speed = 0.0     # will be updated from PID node
        self.max_omega = 1.0

        self.position = None
        self.yaw = 0.0
        self.current_index = 0
        self.i = 0

        # Load waypoints
        csv_path = os.path.join(
            get_package_share_directory("gazebo_environment"),
            "config",
            "path.csv"
        )
        self.waypoints = self.load_waypoints(csv_path)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_info_callback,
            10
        )
        self.speed_sub = self.create_subscription(
            Twist,
            '/cmd_vel_pid',
            self.speed_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Markers
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Publish full path once
        self.publish_path_marker()

        self.get_logger().info("Adaptive Pure Pursuit Node started.")

    def load_waypoints(self, path):
        wps = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    wps.append((float(row[0]), float(row[1])))
        return wps

    def pose_info_callback(self, msg: PoseArray):
        if len(msg.poses) > 1:
            pose = msg.poses[1]
            self.position = (pose.position.x, pose.position.y)
            q = pose.orientation
            self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def speed_callback(self, msg: Twist):
        self.speed = msg.linear.x

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def adaptive_lookahead(self):
        # Example linear adaptation: L = L0 + k * v
        return self.L0 + self.k * self.speed

    def find_goal_point(self, lookahead):
        if self.position is None:
            return None
        x, y = self.position
        for i in range(self.current_index, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            d = math.hypot(wx - x, wy - y)
            if d >= lookahead:
                self.current_index = i
                return wx, wy
        return None

    def control_loop(self):
        if self.position is None:
            return

        lookahead = self.adaptive_lookahead()
        goal = self.find_goal_point(lookahead)
        if goal is None:
            self.get_logger().info("No goal point found — stopping.")
            self.cmd_pub.publish(Twist())
            return

        gx, gy = goal
        x, y = self.position

        dx = gx - x
        dy = gy - y
        alpha = math.atan2(dy, dx) - self.yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        v = self.speed
        L = lookahead
        omega = 2 * v * math.sin(alpha) / L
        omega = max(min(omega, self.max_omega), -self.max_omega)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        # Publish marker of goal
        self.publish_goal_marker(gx, gy)

        if self.i % 10 == 0:
            self.get_logger().info(
                f"[PP] Pos: ({x:.2f},{y:.2f}) Goal: ({gx:.2f},{gy:.2f}) "
                f"v: {v:.2f} L: {L:.2f} α: {math.degrees(alpha):.1f}° ω: {omega:.2f}"
            )
        self.i += 1

    def publish_path_marker(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoints'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        for (wx, wy) in self.waypoints:
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.0
            marker.points.append(p)
        self.marker_pub.publish(marker)

    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal_point'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitAdaptive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
