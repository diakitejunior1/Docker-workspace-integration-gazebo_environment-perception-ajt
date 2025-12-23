#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Point
from visualization_msgs.msg import Marker
import csv, os, math, numpy as np
from ament_index_python.packages import get_package_share_directory


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # Stanley tuning (adaptive for Sonoma turns)
        # Softer cross-track + stronger damping for high-speed wobble
	
        # Stanley tuning (adaptive for Sonoma turns)
        self.k_gain = 1.10          # was 0.8 — more pull to center
        self.k_soft = 0.4           # was 0.5 — sharper steering at high speed
        self.k_yaw_damp = 0.35      # was 0.25 — more damping, kills wobble
        self.max_steer = 1.2        # was 1.5 — prevents oversteer jerk
        self.curve_boost = 0.35     # was 0.5 — smoother entry into sharp turns

        # Speed behavior
        self.min_speed_scale = 0.50 # was 0.70 — slows more on big curves
        self.max_speed_scale = 1.00 # unchanged
        self.max_track_speed = 14.0 # unchanged

        # Lookahead
        self.stanley_lookahead = 4  # was 3 — earlier curve detection

        # Steering smoothing
        self.steer_smooth_factor = 0.55  # was 0.70 — less lag, more responsive

	
        self.smoothed_steer = 0.0

        # Vehicle state
        self.speed = 0.0
        self.position = None
        self.yaw = 0.0
        self.last_yaw_error = 0.0
        self.current_index = 0
        self.i = 0

        # Load waypoints
        csv_path = os.path.join(
            get_package_share_directory("gazebo_environment"),
            "config",
            "path.csv"
        )
        self.waypoints = self.load_waypoints(csv_path)
        self.get_logger().info(f"✅ Loaded {len(self.waypoints)} waypoints")

        # Subscriptions
        self.pose_sub = self.create_subscription(PoseArray, '/pose_info', self.pose_cb, 10)
        self.speed_sub = self.create_subscription(Twist, '/cmd_vel_pid', self.speed_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Visualize path
        self.publish_path_marker()
        self.get_logger().info("🚗 Stanley Controller (adaptive + logging) initialized")

        # CSV logging
        log_path = os.path.join(os.path.expanduser("~"), "stanley_log.csv")
        self.log_file = open(log_path, "w", newline="")
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            "t_sec",
            "x", "y", "yaw",
            "v_pid", "v_cmd",
            "steer",
            "heading_err",
            "cross_track",
            "curvature",
            "wp_index"
        ])
        self.get_logger().info(f"📝 Logging Stanley data to {log_path}")

    # ─────────────────────────────
    def load_waypoints(self, path):
        wps = []
        with open(path, 'r') as f:
            for row in csv.reader(f):
                if len(row) >= 2:
                    wps.append((float(row[0]), float(row[1])))
        return wps

    def pose_cb(self, msg: PoseArray):
        if len(msg.poses) > 1:
            pose = msg.poses[1]
            self.position = (pose.position.x, pose.position.y)
            q = pose.orientation
            self.yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)

    def speed_cb(self, msg: Twist):
        self.speed = msg.linear.x

    @staticmethod
    def quat_to_yaw(x, y, z, w):
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    # ─────────────────────────────
    def nearest_point(self, x, y):
        d2 = [(x - wx) ** 2 + (y - wy) ** 2 for (wx, wy) in self.waypoints]
        i = int(np.argmin(d2))
        return i, self.waypoints[i]

    def compute_curvature(self, idx):
        """Approximate curvature using the angle change between segments."""
        n = len(self.waypoints)
        if n < 3:
            return 0.0

        idx_prev = max(idx - 1, 0)
        idx_next = min(idx + 1, n - 1)

        p_prev = self.waypoints[idx_prev]
        p_curr = self.waypoints[idx]
        p_next = self.waypoints[idx_next]

        v1x = p_curr[0] - p_prev[0]
        v1y = p_curr[1] - p_prev[1]
        v2x = p_next[0] - p_curr[0]
        v2y = p_next[1] - p_curr[1]

        # Avoid zero-length segments
        if (v1x == 0 and v1y == 0) or (v2x == 0 and v2y == 0):
            return 0.0

        ang1 = math.atan2(v1y, v1x)
        ang2 = math.atan2(v2y, v2x)
        d_ang = math.atan2(math.sin(ang2 - ang1), math.cos(ang2 - ang1))
        return abs(d_ang)

    def stanley_control(self):
        if self.position is None or len(self.waypoints) < 2:
            return 0.0, (0.0, 0.0), 0.0, 0.0, 0.0, 0

        x, y = self.position
        v = max(self.speed, 0.1)

        idx, nearest = self.nearest_point(x, y)
        self.current_index = idx

        # Use lookahead waypoint for heading so turns start earlier
        look_idx = min(idx + self.stanley_lookahead, len(self.waypoints) - 1)
        nxt = self.waypoints[look_idx]
        path_yaw = math.atan2(nxt[1] - nearest[1], nxt[0] - nearest[0])

        # Heading error (wrapped)
        heading_err = math.atan2(
            math.sin(path_yaw - self.yaw),
            math.cos(path_yaw - self.yaw)
        )

        # Cross-track error (signed)
        dx, dy = x - nearest[0], y - nearest[1]
        cross_track = -(dy * math.cos(path_yaw) - dx * math.sin(path_yaw))

        # Local curvature around this (lookahead) region
        curvature = self.compute_curvature(look_idx)

        # Dynamic gain: more aggressive on tighter curves,
        # but base gain is softer than before
        k_dynamic = self.k_gain + self.curve_boost * min(curvature, 1.0)

        # Stanley law + small derivative term on heading
        steer = heading_err + math.atan2(k_dynamic * cross_track, self.k_soft + v)
        steer += self.k_yaw_damp * (heading_err - self.last_yaw_error)
        self.last_yaw_error = heading_err

        # Clamp steering (raw)
        steer = max(min(steer, self.max_steer), -self.max_steer)

        return steer, nearest, curvature, heading_err, cross_track, idx

    # ─────────────────────────────
    def control_loop(self):
        if self.position is None:
            return

        steer_raw, nearest, curvature, heading_err, cross_track, idx = self.stanley_control()
        v_pid = self.speed  # speed coming from PID node

        # Smooth steering to reduce high-speed wobble
        alpha = self.steer_smooth_factor
        self.smoothed_steer = alpha * self.smoothed_steer + (1.0 - alpha) * steer_raw
        steer = self.smoothed_steer

        # Curvature-based speed limiting:
        #  - Below a small threshold, don't slow at all (full speed).
        #  - As curvature increases, limit toward min_speed_scale * max_track_speed.
        curvature_threshold = 0.15  # small curves ignored for speed limiting

        if curvature <= curvature_threshold:
            speed_scale = self.max_speed_scale
        else:
            # Map curvature in [threshold, ~1.0+] → k in [0, 1]
            k_raw = (curvature - curvature_threshold) / max(1.0 - curvature_threshold, 1e-3)
            k = max(0.0, min(k_raw, 1.0))
            speed_scale = self.max_speed_scale - (self.max_speed_scale - self.min_speed_scale) * k

        # Stanley's allowed speed (absolute, not relative to PID)
        stanley_speed_limit = self.max_track_speed * speed_scale

        # Final commanded speed: whichever is lower (PID vs Stanley limit)
        v_cmd = min(v_pid, stanley_speed_limit)

        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = steer
        self.cmd_pub.publish(cmd)
        self.publish_goal_marker(nearest[0], nearest[1])

        # Logging
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        try:
            self.csv_writer.writerow([
                now_sec,
                self.position[0], self.position[1], self.yaw,
                v_pid, v_cmd,
                steer,
                heading_err,
                cross_track,
                curvature,
                idx,
            ])
        except Exception as e:
            # avoid crashing controller if logging fails
            self.get_logger().warn(f"Log write failed: {e}")

        if self.i % 10 == 0:
            self.get_logger().info(
                f"[StanleyAdaptive] Pos:({self.position[0]:.2f},{self.position[1]:.2f}) "
                f"v_pid={v_pid:.2f} v_cmd={v_cmd:.2f} steer={steer:.2f} "
                f"idx={idx} curv={curvature:.3f} scale={speed_scale:.2f}"
            )
        self.i += 1

    # ─────────────────────────────
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
        marker.color.g = 1.0
        for wx, wy in self.waypoints:
            p = Point()
            p.x, p.y, p.z = wx, wy, 0.0
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
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.marker_pub.publish(marker)

    # Ensure log file is closed cleanly
    def destroy_node(self):
        try:
            if hasattr(self, "log_file") and not self.log_file.closed:
                self.log_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()