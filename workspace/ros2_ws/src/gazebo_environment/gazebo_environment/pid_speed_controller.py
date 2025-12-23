import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
import math
import csv
import os
from ament_index_python.packages import get_package_share_directory


class PIDSpeedController(Node):

    def __init__(self):
        super().__init__('pid_speed_controller')

        # PID gains (tune these)
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.1

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

        # Max speed output
        self.max_speed = 10.0  # m/s (you can tune)
        self.min_speed = 0.0

        # Waypoint following parameters
        self.waypoints = []
        csv_file_path = os.path.join(
            get_package_share_directory("gazebo_environment"),
            "config",
            "path.csv"
        )
        self.waypoints = self.load_waypoints(csv_file_path)
        self.current_wp_index = 0
        # How many waypoints ahead for the “target” in speed control
        self.lookahead_index = 5

        # Pose subscription to get position
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_callback,
            10
        )

        # Publisher for velocity command
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_pid', 10)

        self.get_logger().info("PID Speed Controller (position‑error based) started.")

    def load_waypoints(self, path):
        wps = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    wps.append((float(row[0]), float(row[1])))
        return wps

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) < 2 or self.current_wp_index >= len(self.waypoints):
            return

        pose = msg.poses[1]
        x = pose.position.x
        y = pose.position.y
        current = (x, y)

        # Determine target waypoint for speed control
        target_idx = min(self.current_wp_index + self.lookahead_index,
                         len(self.waypoints) - 1)
        target = self.waypoints[target_idx]

        dist_error = self.distance(current, target)

        # Time
        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.1 if self.last_time is None else (now - self.last_time)

        # PID
        self.integral += dist_error * dt
        derivative = (dist_error - self.prev_error) / dt if dt > 0 else 0.0

        speed_cmd = self.Kp * dist_error + self.Ki * self.integral + self.Kd * derivative
        # Clamp
        speed_cmd = max(self.min_speed, min(speed_cmd, self.max_speed))

        # Publish
        msg = Twist()
        msg.linear.x = speed_cmd
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

        self.prev_error = dist_error
        self.last_time = now

        # Advance wp index if close to current one
        if self.distance(current, self.waypoints[self.current_wp_index]) < 0.5:
            self.current_wp_index += 1

        self.get_logger().info(
            f"[PID] DistErr: {dist_error:.2f}, SpeedCmd: {speed_cmd:.2f}, WP idx: {self.current_wp_index}"
        )

    @staticmethod
    def distance(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main(args=None):
    rclpy.init(args=args)
    node = PIDSpeedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
