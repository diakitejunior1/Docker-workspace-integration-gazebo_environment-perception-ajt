import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Point
from visualization_msgs.msg import Marker
import csv
import os
import math
from ament_index_python.packages import get_package_share_directory


class IntegratedController(Node):

    def __init__(self):
        super().__init__('integrated_controller')

        # PID gains (can be tuned)
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.0

        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

        # Pure Pursuit parameters
        self.L0 = 1.0
        self.k = 0.5
        self.wheelbase = 2.5
        self.max_omega = 1.0

        # State
        self.position = None
        self.yaw = 0.0
        self.current_wp_index = 0
        self.i = 0

        # Load waypoints
        csv_file = os.path.join(
            get_package_share_directory("gazebo_environment"),
            "config",
            "path.csv"
        )
        self.waypoints = self.load_waypoints(csv_file)
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints.")

        # Subscriptions
        self.pose_sub = self.create_subscription(PoseArray, '/pose_info', self.pose_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Visualization
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.publish_path_marker()

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def load_waypoints(self, path):
        wps = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    wps.append((float(row[0]), float(row[1])))
        return wps

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) > 1:
            pose = msg.poses[1]
            self.position = (pose.position.x, pose.position.y)
            q = pose.orientation
            self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def adaptive_lookahead(self, speed):
        return self.L0 + self.k * speed

    def find_goal_point(self, lookahead):
        if self.position is None:
            return None
        x, y = self.position
        for i in range(self.current_wp_index, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            d = math.hypot(wx - x, wy - y)
            if d >= lookahead:
                self.current_wp_index = i
                return wx, wy
        return None

    def compute_pid_speed(self, current, target):
        dist_error = math.hypot(target[0] - current[0], target[1] - current[1])
        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.1 if self.last_time is None else (now - self.last_time)

        self.integral += dist_error * dt
        derivative = (dist_error - self.prev_error) / dt if dt > 0 else 0.0

        speed = self.Kp * dist_error + self.Ki * self.integral + self.Kd * derivative
        speed = max(0.0, min(speed, 3.0))  # Clamp speed

        self.prev_error = dist_error
        self.last_time = now
        return speed, dist_error

    def control_loop(self):
        if self.position is None or self.current_wp_index >= len(self.waypoints):
            return

        x, y = self.position

        # Speed Control (PID)
        speed_target_index = min(self.current_wp_index + 5, len(self.waypoints) - 1)
        speed = 0.0
        speed, dist_error = self.compute_pid_speed((x, y), self.waypoints[speed_target_index])

        # Steering Control (Pure Pursuit)
        lookahead = self.adaptive_lookahead(speed)
        goal = self.find_goal_point(lookahead)
        if goal is None:
            self.get_logger().info("No valid goal point.")
            self.cmd_pub.publish(Twist())
            return

        gx, gy = goal
        dx = gx - x
        dy = gy - y
        alpha = math.atan2(dy, dx) - self.yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))  # normalize

        omega = 2 * speed * math.sin(alpha) / lookahead
        omega = max(min(omega, self.max_omega), -self.max_omega)

        # Publish command
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        # Visualize goal point
        self.publish_goal_marker(gx, gy)

        if self.i % 10 == 0:
            self.get_logger().info(
                f"[INTEGRATED] Pos: ({x:.2f},{y:.2f}) Goal: ({gx:.2f},{gy:.2f}) "
                f"DistErr: {dist_error:.2f}, v: {speed:.2f}, α: {math.degrees(alpha):.1f}°, ω: {omega:.2f}"
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
    node = IntegratedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
