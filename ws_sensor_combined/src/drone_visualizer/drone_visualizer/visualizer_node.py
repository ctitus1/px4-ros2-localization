import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, GimbalDeviceSetAttitude
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.drone_pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.gimbal_pose_pub = self.create_publisher(PoseStamped, '/gimbal/pose', 10)

        self.drone_pos = [0.0, 0.0, 0.0]
        self.drone_q = [0.0, 0.0, 0.0, 1.0]
        self.gimbal_q = [0.0, 0.0, 0.0, 1.0]

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos)
        self.create_subscription(GimbalDeviceSetAttitude, '/fmu/in/gimbal_device_set_attitude', self.gimbal_callback, qos)

        self.timer = self.create_timer(0.1, self.publish_transforms)

    def local_pos_callback(self, msg):
        self.drone_pos = [float(msg.x), float(msg.y), -float(msg.z)] # negate z to have positive altitude

    def attitude_callback(self, msg):
        q_px4 = msg.q  # [w, x, y, z]
        self.drone_q = [float(q_px4[1]), float(q_px4[2]), float(q_px4[3]), float(q_px4[0])]

    def gimbal_callback(self, msg):
        q = msg.q
        self.gimbal_q = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()

        # TF: map → base_link
        drone_tf = TransformStamped()
        drone_tf.header.stamp = now
        drone_tf.header.frame_id = 'map'
        drone_tf.child_frame_id = 'drone_frame'
        drone_tf.transform.translation.x = float(self.drone_pos[0])
        drone_tf.transform.translation.y = float(self.drone_pos[1])
        drone_tf.transform.translation.z = float(self.drone_pos[2])
        drone_tf.transform.rotation.x = float(self.drone_q[0])
        drone_tf.transform.rotation.y = float(self.drone_q[1])
        drone_tf.transform.rotation.z = float(self.drone_q[2])
        drone_tf.transform.rotation.w = float(self.drone_q[3])

        # TF: base_link → gimbal_link
        gimbal_tf = TransformStamped()
        gimbal_tf.header.stamp = now
        gimbal_tf.header.frame_id = 'drone_frame'
        gimbal_tf.child_frame_id = 'gimbal_frame'
        gimbal_tf.transform.translation.x = 0.0
        gimbal_tf.transform.translation.y = 0.0
        gimbal_tf.transform.translation.z = -0.1
        gimbal_tf.transform.rotation.x = float(self.gimbal_q[0])
        gimbal_tf.transform.rotation.y = float(self.gimbal_q[1])
        gimbal_tf.transform.rotation.z = float(self.gimbal_q[2])
        gimbal_tf.transform.rotation.w = float(self.gimbal_q[3])

        self.tf_broadcaster.sendTransform([drone_tf, gimbal_tf])

        # PoseStamped: drone
        drone_pose = PoseStamped()
        drone_pose.header.stamp = now
        drone_pose.header.frame_id = 'map'
        drone_pose.pose.position.x = float(self.drone_pos[0])
        drone_pose.pose.position.y = float(self.drone_pos[1])
        drone_pose.pose.position.z = float(self.drone_pos[2])
        drone_pose.pose.orientation.x = float(self.drone_q[0])
        drone_pose.pose.orientation.y = float(self.drone_q[1])
        drone_pose.pose.orientation.z = float(self.drone_q[2])
        drone_pose.pose.orientation.w = float(self.drone_q[3])
        self.drone_pose_pub.publish(drone_pose)

        # PoseStamped: gimbal
        gimbal_pose = PoseStamped()
        gimbal_pose.header.stamp = now
        gimbal_pose.header.frame_id = 'drone_frame'
        gimbal_pose.pose.position.x = 0.0
        gimbal_pose.pose.position.y = 0.0
        gimbal_pose.pose.position.z = -0.1
        gimbal_pose.pose.orientation.x = float(self.gimbal_q[0])
        gimbal_pose.pose.orientation.y = float(self.gimbal_q[1])
        gimbal_pose.pose.orientation.z = float(self.gimbal_q[2])
        gimbal_pose.pose.orientation.w = float(self.gimbal_q[3])
        self.gimbal_pose_pub.publish(gimbal_pose)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
