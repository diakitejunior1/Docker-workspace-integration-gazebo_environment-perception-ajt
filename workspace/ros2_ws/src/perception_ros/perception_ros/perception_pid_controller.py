'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class PIDSpeedControllerPerception(Node):

    def __init__(self):
        super().__init__('pid_speed_controller_perception')

        # PID gains
        self.Kp = 1.0
        self.Ki = 0.0
        self.Kd = 0.1

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

        # Max speed output
        self.max_speed = 10.0
        self.min_speed = 0.0

        # Current robot position
        self.robot_position = None

        # Subscribe to perception pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/pose_msg',
            self.pose_callback,
            10
        )

        # Publisher for velocity command
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_pid', 10)

        self.get_logger().info("PID Speed Controller (Perception-based) started.")

    def pose_callback(self, msg: PoseStamped):
        # Use PoseStamped from perception as target
        target = (msg.pose.position.x, msg.pose.position.y)

        # Assume robot at origin in world frame if no odometry
        x, y = 0.0, 0.0  # If you have odom, replace with actual position
        self.robot_position = (x, y)
        current = self.robot_position

        # Distance error
        dist_error = math.hypot(target[0] - current[0], target[1] - current[1])

        # Time
        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.1 if self.last_time is None else (now - self.last_time)

        # PID computation
        self.integral += dist_error * dt
        derivative = (dist_error - self.prev_error) / dt if dt > 0 else 0.0
        speed_cmd = self.Kp * dist_error + self.Ki * self.integral + self.Kd * derivative

        # Clamp
        speed_cmd = max(self.min_speed, min(speed_cmd, self.max_speed))

        # Publish
        msg_cmd = Twist()
        msg_cmd.linear.x = speed_cmd
        msg_cmd.angular.z = 0.0
        self.cmd_pub.publish(msg_cmd)

        self.prev_error = dist_error
        self.last_time = now

        self.get_logger().info(f"[PID] Target: ({target[0]:.2f},{target[1]:.2f}), DistErr: {dist_error:.2f}, SpeedCmd: {speed_cmd:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PIDSpeedControllerPerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
import math

class PIDSpeedControllerPerception(Node):

    def __init__(self):
        super().__init__('pid_speed_controller_perception')

        # PID gains
        self.Kp = 0.8
        self.Ki = 0.0
        self.Kd = 0.05

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

        # Robot and target
        self.robot_position = None
        self.target_pose = None

        # Speed limits
        self.max_speed = 6.0
        self.min_speed = 1.5

        # Subscriptions
        self.create_subscription(PoseArray, '/pose_info', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/pose_msg', self.target_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_pid', 10)

        self.get_logger().info("✅ PID Speed Controller started")

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) > 1:
            p = msg.poses[1].position
            self.robot_position = (p.x, p.y)

    def target_callback(self, msg: PoseStamped):
        self.target_pose = (msg.pose.position.x, msg.pose.position.y)
        self.compute_pid()

    def compute_pid(self):
        if self.robot_position is None or self.target_pose is None:
            return

        x, y = self.robot_position
        tx, ty = self.target_pose

        # Linear distance error
        error = math.hypot(tx - x, ty - y)

        now = self.get_clock().now().nanoseconds / 1e9
        dt = 0.1 if self.last_time is None else max(now - self.last_time, 1e-3)

        # PID calculations
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        speed = self.Kp * error + self.Kd * derivative
        speed = max(self.min_speed, min(speed, self.max_speed))

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        self.prev_error = error
        self.last_time = now

        self.get_logger().info(f"[PID] dist={error:.2f} speed={speed:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PIDSpeedControllerPerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
