import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CmdVelPublisherNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = time.time()

    def timer_callback(self):
        msg = Twist()
        # Set the velocity values
        if time.time() - self.start_time < 1.0:
            msg.linear.x = 1.0  # Forward speed
            # msg.linear.y = 1.0  # Forward speed
            # msg.angular.z = 1.0  # Turn left
        else:
            msg.linear.x = 0.0  # Stop
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.timer.cancel()
        # Publish the cmd_vel message
        self.publisher.publish(msg)

    def run_a_square(self):
        # Create a timer with a callback that is called every 0.1 seconds
        msg = Twist()
        if time.time() - self.start_time < 1.0:
            msg.linear.x = 1.0
            msg.linear.y = 0.0
        elif time.time() - self.start_time < 2.0:
            msg.linear.y = 1.0
            msg.linear.x = 0.0
        elif time.time() - self.start_time < 3.0:
            msg.linear.x = -1.0
            msg.linear.y = 0.0
        elif time.time() - self.start_time < 4.0:
            msg.linear.y = -1.0
            msg.linear.x = 0.0
        else:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            self.timer.cancel()

        self.publisher.publish(msg)

    def spin_left_then_right(self):
        msg = Twist()
        if time.time() - self.start_time < 2.0:
            msg.angular.z = 3.14
        elif time.time() - self.start_time < 4.0:
            msg.angular.z = -3.14
        else:
            msg.angular.z = 0.0
            self.timer.cancel()

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = CmdVelPublisherNode()

    # Create a timer with a callback that is called every 0.1 seconds
    node.timer = node.create_timer(0.1, node.timer_callback)
    # node.timer = node.create_timer(0.1, node.run_a_square)
    # node.timer = node.create_timer(0.1, node.spin_left_then_right)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()