import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class ImageDotNode(Node):
    def __init__(self):
        super().__init__('image_dot_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/udp_video/image_raw',
            self.image_callback,
            10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/udp_video/camera_info',
            self.camera_info_callback,
            10)

        self.image_pub = self.create_publisher(Image, '/udp_video/image_with_dot', 10)

        self.camera_info = None

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def image_callback(self, msg):
        if self.camera_info is None:
            return

        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Draw red dot at center
        height, width, _ = cv_image.shape
        center = (width // 2, height // 2)
        cv2.circle(cv_image, center, radius=5, color=(0, 0, 255), thickness=-1)

        # Convert back to ROS Image and publish
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        ros_image.header = msg.header  # Preserve the original timestamp and frame
        self.image_pub.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = ImageDotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
