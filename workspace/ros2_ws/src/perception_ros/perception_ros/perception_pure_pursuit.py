'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class PurePursuitAdaptivePerception(Node):

    def __init__(self):
        super().__init__('pure_pursuit_adaptive_perception')

        # Lookahead parameters
        self.L0 = 1.0
        self.k = 0.5
        self.max_omega = 1.0

        self.speed = 0.0  # from PID
        self.robot_position = (0.0, 0.0)  # Assuming robot at origin
        self.yaw = 0.0

        # Subscribe to perception target
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/pose_msg',
            self.target_callback,
            10
        )

        # Subscribe to speed command
        self.speed_sub = self.create_subscription(
            Twist,
            '/cmd_vel_pid',
            self.speed_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Current target
        self.goal = None

        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Adaptive Pure Pursuit (Perception-based) started.")

    def target_callback(self, msg: PoseStamped):
        # Perception target in world frame
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def speed_callback(self, msg: Twist):
        self.speed = msg.linear.x

    def control_loop(self):
        if self.goal is None:
            return

        gx, gy = self.goal
        x, y = self.robot_position

        # Compute lookahead
        L = self.L0 + self.k * self.speed

        # Pure Pursuit geometry
        dx = gx - x
        dy = gy - y
        alpha = math.atan2(dy, dx) - self.yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        v = self.speed
        omega = 2 * v * math.sin(alpha) / L
        omega = max(min(omega, self.max_omega), -self.max_omega)

        # Publish Twist
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f"[PP] Goal: ({gx:.2f},{gy:.2f}), v: {v:.2f}, α: {math.degrees(alpha):.1f}°, ω: {omega:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitAdaptivePerception()
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

class PurePursuitAdaptivePerception(Node):

    def __init__(self):
        super().__init__('pure_pursuit_adaptive_perception')

        # Lookahead parameters
        self.L0 = 1.2
        self.k = 0.35
        self.min_L = 1.0
        self.max_L = 4.0

        self.max_omega = 1.2
        self.last_omega = 0.0

        # Robot and target
        self.speed = 0.0
        self.robot_position = None
        self.yaw = 0.0
        self.goal = None

        # Subscriptions
        self.create_subscription(PoseArray, '/pose_info', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/pose_msg', self.target_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_pid', self.speed_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("✅ Pure Pursuit Controller started")

    def target_callback(self, msg: PoseStamped):
        # Already global coordinates
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def speed_callback(self, msg: Twist):
        self.speed = msg.linear.x

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) > 1:
            pose = msg.poses[1]
            self.robot_position = (pose.position.x, pose.position.y)

            q = pose.orientation
            self.yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

    def control_loop(self):
        if self.goal is None or self.robot_position is None:
            return

        gx, gy = self.goal
        x, y = self.robot_position

        dx = gx - x
        dy = gy - y

        # Transform goal to robot frame
        goal_x_body = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy

        if goal_x_body < 0.5:  # ignore targets behind
            return

        # Adaptive lookahead
        L = self.L0 + self.k * abs(self.speed)
        L = max(self.min_L, min(L, self.max_L))

        # Steering angle
        alpha = math.atan2(dy, dx) - self.yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        omega = (2.0 * self.speed * math.sin(alpha)) / L
        omega = max(min(omega, self.max_omega), -self.max_omega)

        # Smooth steering
        omega = 0.7 * self.last_omega + 0.3 * omega
        self.last_omega = omega

        cmd = Twist()
        cmd.linear.x = self.speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"[PP] α={math.degrees(alpha):.1f}° ω={omega:.2f} L={L:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitAdaptivePerception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
