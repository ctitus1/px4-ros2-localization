import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml


class UDPVideoPublisher(Node):
    def __init__(self):
        super().__init__('udp_video_publisher')

        self.publisher_image = self.create_publisher(Image, 'udp_video/image_raw', 10)
        self.publisher_info = self.create_publisher(CameraInfo, 'udp_video/camera_info', 10)

        self.bridge = CvBridge()

        # Load YAML manually using PyYAML
        self.camera_info = self.load_camera_info_from_yaml("/home/ctitus/.ros/camera_info/gz500_camera.yaml")

        # GStreamer pipeline to receive UDP stream
        self.pipeline = (
            'udpsrc port=5600 ! application/x-rtp, encoding-name=H264, payload=96 ! '
            'rtph264depay ! avdec_h264 ! videoconvert ! appsink'
        )

        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video stream!')
            return

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # ~30 FPS

    def load_camera_info_from_yaml(self, path):
        with open(path, 'r') as f:
            calib = yaml.safe_load(f)

        cam_info = CameraInfo()
        cam_info.width = calib['image_width']
        cam_info.height = calib['image_height']
        cam_info.distortion_model = calib['distortion_model']
        cam_info.d = calib['distortion_coefficients']['data']
        cam_info.k = calib['camera_matrix']['data']
        cam_info.r = calib['rectification_matrix']['data']
        cam_info.p = calib['projection_matrix']['data']
        cam_info.binning_x = 0
        cam_info.binning_y = 0
        cam_info.roi.x_offset = 0
        cam_info.roi.y_offset = 0
        cam_info.roi.height = 0
        cam_info.roi.width = 0
        cam_info.roi.do_rectify = False
        return cam_info

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_link'

            self.camera_info.header = msg.header
            self.publisher_image.publish(msg)
            self.publisher_info.publish(self.camera_info)
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
