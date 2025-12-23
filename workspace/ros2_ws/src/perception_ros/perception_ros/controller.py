'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
import math


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        self.lookahead = 4.0
        self.speed = 2.0
        self.max_omega = 1.2

        self.goal = None
        self.yaw = 0.0

        self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_info_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/pose_msg',
            self.goal_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("✅ Pure Pursuit Controller started")

    def pose_info_callback(self, msg: PoseArray):
        if len(msg.poses) < 2:
            return

        pose = msg.poses[1]  # ✅ VEHICLE POSE
        self.yaw = self.quaternion_to_yaw(pose.orientation)

    def goal_callback(self, msg: PoseStamped):
        if msg.header.frame_id != "base_link":
            self.get_logger().warn("Goal not in base_link frame")
            return

        self.goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )

    def control_loop(self):
        if self.goal is None:
            return

        gx, gy = self.goal

        alpha = math.atan2(gy, gx)
        omega = 2 * self.speed * math.sin(alpha) / self.lookahead
        omega = max(min(omega, self.max_omega), -self.max_omega)

        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

    @staticmethod
    def quaternion_to_yaw(q):
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
---------------

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
import math


class PurePursuitController(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.lookahead = 5.0
        self.speed = 2.0
        self.max_omega = 1.2

        # State
        self.goal_world = None
        self.vehicle_pos = None
        self.vehicle_yaw = 0.0

        # Subscriptions
        self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_info_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/pose_msg',
            self.goal_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("✅ Pure Pursuit Controller (WORLD frame) started")

    # ================= CALLBACKS =================

    def pose_info_callback(self, msg: PoseArray):
        if len(msg.poses) < 2:
            return

        pose = msg.poses[1]  # vehicle pose
        self.vehicle_pos = (
            pose.position.x,
            pose.position.y
        )
        self.vehicle_yaw = self.quaternion_to_yaw(pose.orientation)

    def goal_callback(self, msg: PoseStamped):
        if msg.header.frame_id != "world":
            self.get_logger().warn("Goal not in world frame")
            return

        self.goal_world = (
            msg.pose.position.x,
            msg.pose.position.y
        )

    # ================= CONTROL =================

    def control_loop(self):
        if self.goal_world is None or self.vehicle_pos is None:
            return

        gx, gy = self.goal_world
        vx, vy = self.vehicle_pos

        # Vector from vehicle to goal (WORLD frame)
        dx = gx - vx
        dy = gy - vy

        # Transform into vehicle frame
        x_rel = math.cos(self.vehicle_yaw) * dx + math.sin(self.vehicle_yaw) * dy
        y_rel = -math.sin(self.vehicle_yaw) * dx + math.cos(self.vehicle_yaw) * dy

        # Safety: ignore goals behind vehicle
        if x_rel <= 0.1:
            return

        # Pure Pursuit
        alpha = math.atan2(y_rel, x_rel)
        omega = 2.0 * self.speed * math.sin(alpha) / self.lookahead
        omega = max(min(omega, self.max_omega), -self.max_omega)

        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = omega

        self.cmd_pub.publish(cmd)

    # ================= UTILS =================

    @staticmethod
    def quaternion_to_yaw(q):
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
import math
import numpy as np

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z

class PurePursuit:
    def __init__(self, wheelbase=1.0, lookahead_distance=2.0):
        self.wheelbase = wheelbase
        self.lookahead_distance = lookahead_distance
        self.steer_gain = 1.15   # <-- slight sensitivity increase

    def calculate_steering(self, current_pose, target_point):
        cx, cy, cyaw = current_pose
        tx, ty = target_point
        dx_world = tx - cx
        dy_world = ty - cy
        local_x = dx_world * math.cos(cyaw) + dy_world * math.sin(cyaw)
        local_y = -dx_world * math.sin(cyaw) + dy_world * math.cos(cyaw)
        lookahead_distance_sq = local_x**2 + local_y**2
        if lookahead_distance_sq < 0.01:
            return 0.0, 0.0
        steering_angle = math.atan(
            2 * self.wheelbase * local_y / lookahead_distance_sq
        )
        steering_angle *= self.steer_gain
        return steering_angle, local_y

class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def calculate_velocity(self, current_pose, target_point, steering_angle):
        cx, cy, _ = current_pose
        tx, ty = target_point
        error = math.sqrt((tx - cx)**2 + (ty - cy)**2)
        velocity_reference = self.kp * error
        velocity_reference = velocity_reference * max(0.2, math.cos(steering_angle))
        max_vel = 0.6
        min_vel = -0.6
        velocity_reference = max(min_vel, min(velocity_reference, max_vel))
        return velocity_reference

class IntegratedController(Node):
    def __init__(self):
        super().__init__('pp_pid_controller')
        self.wheelbase = 0.33
        self.lookahead_distance = 1.0
        self.kp = 1.0
        self.pure_pursuit = PurePursuit(self.wheelbase, self.lookahead_distance)
        self.pid = PID(self.kp, 0.0, 0.0)
        self.current_pose = None
        self.target_point = None
        self.pose_info_sub = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_info_callback,
            10
        )
        self.pose_msg_sub = self.create_subscription(
            PoseStamped,
            '/pose_msg',
            self.pose_msg_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def pose_info_callback(self, msg):
        if msg.poses:
            pose = msg.poses[0]
            x = pose.position.x
            y = pose.position.y
            _, _, yaw = euler_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            )
            self.current_pose = (x, y, yaw)

    def pose_msg_callback(self, msg):
        self.target_point = (msg.pose.position.x, msg.pose.position.y)

    def control_loop(self):
        if self.current_pose is None or self.target_point is None:
            return
        steering_angle, _ = self.pure_pursuit.calculate_steering(
            self.current_pose,
            self.target_point
        )
        velocity = self.pid.calculate_velocity(
            self.current_pose,
            self.target_point,
            steering_angle
        )
        cmd = Twist()
        cmd.linear.x = float(velocity)
        cmd.angular.z = float(steering_angle)
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()