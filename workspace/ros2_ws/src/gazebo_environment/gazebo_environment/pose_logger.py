import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from datetime import datetime
import csv
import os

class PoseInfoLogger(Node):
    def __init__(self):
        super().__init__('pose_info_logger')

        # Subscribe to the PoseArray topic
        self.subscription = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_callback,
            10
        )

        # Prepare logging folder
        log_dir = os.path.expanduser('~/pose_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = os.path.join(log_dir, f'robot_pose_{timestamp}.csv')

        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['x', 'y', 'z'])  # No timestamp unless needed

        self.get_logger().info(f"Logging robot pose (index 1) to: {self.filename}")

    def pose_callback(self, msg: PoseArray):
        try:
            robot_pose = msg.poses[1]  # Robot is at index 1
            x = robot_pose.position.x
            y = robot_pose.position.y
            z = robot_pose.position.z
            self.writer.writerow([x, y, z])
        except IndexError:
            self.get_logger().warn("PoseArray does not have index 1!")
        except Exception as e:
            self.get_logger().error(f"Error while logging pose: {e}")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseInfoLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
