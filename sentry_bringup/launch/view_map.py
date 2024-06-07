import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from foxglove_msgs.msg import RawImage

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher = self.create_publisher(RawImage, 'map_image', 10)
        self.info_publisher = self.create_publisher(CameraInfo, 'camera_info', 10)  # New publisher for CameraInfo

    def timer_callback(self):
        # Read the image from a file
        img = cv2.imread('/home/sentry_ws/src/sentry_bringup/maps/RMUL_img.png', cv2.IMREAD_COLOR)

        # convert to rgba
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)

        # set alpha to 255
        img[:, :, 3] = 0.1

        # Create a message
        msg = RawImage()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.frame_id = 'map'
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = 'rgba8'
        # msg.is_bigendian = False
        msg.step = img.shape[1] * 4
        msg.data = img.tobytes()

        # Publish the message
        self.publisher.publish(msg)

        # Create and publish CameraInfo message
        info_msg = CameraInfo()
        info_msg.header.stamp = self.get_clock().now().to_msg()
        info_msg.header.frame_id = 'map'
        info_msg.width = img.shape[1]
        info_msg.height = img.shape[0]
        info_msg.distortion_model = 'plumb_bob'
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        info_msg.r = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        info_msg.p = [0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0]
        self.info_publisher.publish(info_msg)  # Publish CameraInfo message

def main(args=None):
    rclpy.init(args=args)

    node = ImagePublisherNode()

    # Create a timer with a callback that is called every 1 second
    node.timer = node.create_timer(1.0, node.timer_callback)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()