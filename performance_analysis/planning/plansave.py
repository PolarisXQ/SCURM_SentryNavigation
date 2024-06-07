import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class PathSaverNode(Node):
    def __init__(self):
        super().__init__('path_saver_node')
        self.subscription = self.create_subscription(
            Path,
            '/plan_smoothed',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        with open('SimpleSmoother.txt', 'w') as f:
            for pose_stamped in msg.poses:
                pose = pose_stamped.pose
                timestamp = pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9
                f.write(f'{timestamp} {pose.position.x} {pose.position.y} {pose.position.z} {pose.orientation.x} {pose.orientation.y} {pose.orientation.z} {pose.orientation.w}\n')

def main(args=None):
    rclpy.init(args=args)

    node = PathSaverNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()