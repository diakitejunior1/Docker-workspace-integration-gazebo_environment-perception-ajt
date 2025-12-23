import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
import math
import csv
import os
from ament_index_python.packages import get_package_share_directory


class PositionPIDController(Node):

    def __init__(self):
        super().__init__('position_pid_controller')

        # PID gains (you may need to tune)
        self.Kp = 1.2
        self.Ki = 0.1
        self.Kd = 0.0

        # Target lookahead waypoint (position based control)
        self.lookahead_index = 5  # How far ahead in the waypoint list to aim for
        self.max_speed = 2.0  # m/s

        # Internal PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

        # Load waypoints
        csv_file_path = os.path.join(
            get_package_share_directory("gazebo_environment"),
            "config",
            "path.csv"
        )
        self.waypoints = self.load_waypoints(csv_file_path)
        self.current_wp_index = 0

        # Subscribers and publishers
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info("Position-based PID controller started.")

    def load_waypoints(self, path):
        waypoints = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    waypoints.append((float(row[0]), float(row[1])))
        return waypoints

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) < 2 or self.current_wp_index >= len(self.waypoints):
            return

        pose = msg.poses[1]
        x = pose.position.x
        y = pose.position.y
        current_pos = (x, y)

        # Target waypoint at lookahead
        target_index = min(self.current_wp_index + self.lookahead_index, len(self.waypoints) - 1)
        target_pos = self.waypoints[target_index]

        # Distance (error) to target
        dist_error = self.distance(current_pos, target_pos)

        # PID computation
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = 0.1 if self.last_time is None else (current_time - self.last_time)

        self.integral += dist_error * dt
        derivative = (dist_error - self.prev_error) / dt if dt > 0 else 0.0

        # PID output as speed
        speed_cmd = self.Kp * dist_error + self.Ki * self.integral + self.Kd * derivative
        speed_cmd = max(min(speed_cmd, self.max_speed), 0.0)

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = speed_cmd
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        self.prev_error = dist_error
        self.last_time = current_time

        # Update waypoint index if we get close to the current one
        if self.distance(current_pos, self.waypoints[self.current_wp_index]) < 0.5:
            self.current_wp_index += 1

        self.get_logger().info(
            f"Dist to target: {dist_error:.2f} m | Speed command: {speed_cmd:.2f} m/s | WP Index: {self.current_wp_index}"
        )

    @staticmethod
    def distance(p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def main(args=None):
    rclpy.init(args=args)
    node = PositionPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
