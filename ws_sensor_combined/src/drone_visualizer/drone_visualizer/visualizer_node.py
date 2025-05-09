import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, GimbalDeviceSetAttitude, HomePosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial.transform import Rotation as R

def ned_to_flu_xyz(xyz_enu):
    """Convert position from ENU to NED coordinate frame."""
    x_enu, y_enu, z_enu = xyz_enu
    return [x_enu, -y_enu, -z_enu]

def ned_to_flu_quat(q_enu):
    """
    Convert quaternion from ENU (x-East, y-North, z-Up)
    to NED (x-North, y-East, z-Down).

    q_enu: list or np.array of shape (4,), in [x, y, z, w] format.
    Returns q_ned in [x, y, z, w] format.
    """
    # ENU to NED axis transform
    R_conv = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])

    # Convert input quaternion to rotation object
    r_enu = R.from_quat(q_enu)  # input: [x, y, z, w]

    # Apply ENU to NED conversion: R_ned = R_conv * R_enu * R_conv.T
    R_ned_matrix = R_conv @ r_enu.as_matrix() @ R_conv.T
    r_ned = R.from_matrix(R_ned_matrix)

    return r_ned.as_quat()  # returns [x, y, z, w]

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.drone_pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.gimbal_pose_pub = self.create_publisher(PoseStamped, '/gimbal/pose', 10)
        self.ray_pub = self.create_publisher(Marker, '/gimbal/ray', 10)

        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_q = [0.0, 0.0, 0.0, 1.0]
        self.gimbal_q = [0.0, 0.0, 0.0, 1.0] # set this to resting orientation of gimbal
        self.home_pos = [0.0, 0.0, 0.0]

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos)
        self.create_subscription(GimbalDeviceSetAttitude, '/fmu/in/gimbal_device_set_attitude', self.gimbal_callback, qos)
        self.create_subscription(HomePosition, '/fmu/in/home_position', self.home_callback, qos)

        self.timer = self.create_timer(0.1, self.publish_transforms)

    def home_callback(self, msg):
        self.home_pos = ned_to_flu_xyz([float(msg.x), float(msg.y), float(msg.z)])


    def local_pos_callback(self, msg):
        self.drone_pos = ned_to_flu_xyz([float(msg.x), float(msg.y), float(msg.z)])

    def attitude_callback(self, msg):
        q = msg.q
        self.drone_q = ned_to_flu_quat([float(q[1]), float(q[2]), float(q[3]), float(q[0])])

    def gimbal_callback(self, msg):
        q = msg.q
        if np.any(np.isnan(q)):
            self.get_logger().warn(f"Quaternion contains NaN — skipping. q = {q}")
            return
        self.gimbal_q = ned_to_flu_quat([float(q[1]), float(q[2]), float(q[3]), float(q[0])])

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()

        # TF: home → drone_frame
        drone_tf = TransformStamped()
        drone_tf.header.stamp = now
        drone_tf.header.frame_id = 'home_frame'
        drone_tf.child_frame_id = 'drone_frame'
        drone_tf.transform.translation.x = self.drone_pos[0] - self.home_pos[0]
        drone_tf.transform.translation.y = self.drone_pos[1] - self.home_pos[1]
        drone_tf.transform.translation.z = self.drone_pos[2] - self.home_pos[2]
        drone_tf.transform.rotation.x = self.drone_q[0]
        drone_tf.transform.rotation.y = self.drone_q[1]
        drone_tf.transform.rotation.z = self.drone_q[2]
        drone_tf.transform.rotation.w = self.drone_q[3]

        # TF: drone_frame → gimbal_frame
        gimbal_tf = TransformStamped()
        gimbal_tf.header.stamp = now
        gimbal_tf.header.frame_id = 'drone_frame'
        gimbal_tf.child_frame_id = 'gimbal_frame'
        gimbal_tf.transform.translation.x = 0.0
        gimbal_tf.transform.translation.y = 0.0
        gimbal_tf.transform.translation.z = 0.0
        gimbal_tf.transform.rotation.x = self.gimbal_q[0]
        gimbal_tf.transform.rotation.y = self.gimbal_q[1]
        gimbal_tf.transform.rotation.z = self.gimbal_q[2]
        gimbal_tf.transform.rotation.w = self.gimbal_q[3]

        self.tf_broadcaster.sendTransform([drone_tf, gimbal_tf])

        # PoseStamped: drone
        drone_pose = PoseStamped()
        drone_pose.header.stamp = now
        drone_pose.header.frame_id = 'home_frame'
        drone_pose.pose.position.x = drone_tf.transform.translation.x
        drone_pose.pose.position.y = drone_tf.transform.translation.y
        drone_pose.pose.position.z = drone_tf.transform.translation.z
        drone_pose.pose.orientation = drone_tf.transform.rotation
        self.drone_pose_pub.publish(drone_pose)

        # PoseStamped: gimbal
        gimbal_pose = PoseStamped()
        gimbal_pose.header.stamp = now
        gimbal_pose.header.frame_id = 'drone_frame'
        gimbal_pose.pose.position.x = 0.0
        gimbal_pose.pose.position.y = 0.0
        gimbal_pose.pose.position.z = 0.0
        gimbal_pose.pose.orientation.x = self.gimbal_q[0]
        gimbal_pose.pose.orientation.y = self.gimbal_q[1]
        gimbal_pose.pose.orientation.z = self.gimbal_q[2]
        gimbal_pose.pose.orientation.w = self.gimbal_q[3]
        self.gimbal_pose_pub.publish(gimbal_pose)

        # Marker: ray from gimbal_frame
        ray_marker = Marker()
        ray_marker.header.stamp = now
        ray_marker.header.frame_id = 'gimbal_frame'
        ray_marker.ns = 'gimbal_ray'
        ray_marker.id = 0
        ray_marker.type = Marker.ARROW
        ray_marker.action = Marker.ADD
        ray_marker.lifetime.sec = 0  # persist until replaced

        ray_marker.pose.position.x = 0.0
        ray_marker.pose.position.y = 0.0
        ray_marker.pose.position.z = 0.0
        ray_marker.pose.orientation.x = 0.0
        ray_marker.pose.orientation.y = 0.0
        ray_marker.pose.orientation.z = 0.0
        ray_marker.pose.orientation.w = 0.0

        ray_marker.scale.x = 50.0  # arrow length
        ray_marker.scale.y = 0.01
        ray_marker.scale.z = 0.01

        ray_marker.color.a = 1.0
        ray_marker.color.r = 1.0
        ray_marker.color.g = 0.0
        ray_marker.color.b = 0.0

        self.ray_pub.publish(ray_marker)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
