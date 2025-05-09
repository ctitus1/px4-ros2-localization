import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class UDPVideoPublisher(Node):
    def __init__(self):
        super().__init__('udp_video_publisher')
        self.publisher_ = self.create_publisher(Image, 'udp_video/image_raw', 10)
        self.bridge = CvBridge()

        # GStreamer pipeline to receive UDP stream (change port if needed)
        self.pipeline = (
            'udpsrc port=5600 ! application/x-rtp, encoding-name=H264, payload=96 ! '
            'rtph264depay ! avdec_h264 ! videoconvert ! appsink'
        )

        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video stream!')
            return

        timer_period = 0.033  # ~30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to read frame')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UDPVideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
