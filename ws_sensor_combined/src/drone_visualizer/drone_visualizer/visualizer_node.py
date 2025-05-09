import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
import math
import time

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.drone_pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.gimbal_pose_pub = self.create_publisher(PoseStamped, '/gimbal/pose', 10)

        # Timer to update @ 10 Hz
        self.timer = self.create_timer(0.1, self.publish_transforms)

    def publish_transforms(self):
        now = self.get_clock().now().to_msg()

        # Example: drone is at (10, 5, 1.5) with no rotation
        drone_pos = [10.0, 5.0, 1.5]
        drone_q = [0.0, 0.0, 0.0, 1.0]  # identity quaternion

        # Example: gimbal is tilted down, child of base_link
        gimbal_q = [0.0, 0.7071, 0.0, 0.7071]  # 90 deg pitch down

        # TF: map → base_link
        drone_tf = TransformStamped()
        drone_tf.header.stamp = now
        drone_tf.header.frame_id = 'map'
        drone_tf.child_frame_id = 'base_link'
        drone_tf.transform.translation.x = drone_pos[0]
        drone_tf.transform.translation.y = drone_pos[1]
        drone_tf.transform.translation.z = drone_pos[2]
        drone_tf.transform.rotation.x = drone_q[0]
        drone_tf.transform.rotation.y = drone_q[1]
        drone_tf.transform.rotation.z = drone_q[2]
        drone_tf.transform.rotation.w = drone_q[3]

        # TF: base_link → gimbal_link
        gimbal_tf = TransformStamped()
        gimbal_tf.header.stamp = now
        gimbal_tf.header.frame_id = 'base_link'
        gimbal_tf.child_frame_id = 'gimbal_link'
        gimbal_tf.transform.translation.x = 0.0
        gimbal_tf.transform.translation.y = 0.0
        gimbal_tf.transform.translation.z = -0.1
        gimbal_tf.transform.rotation.x = gimbal_q[0]
        gimbal_tf.transform.rotation.y = gimbal_q[1]
        gimbal_tf.transform.rotation.z = gimbal_q[2]
        gimbal_tf.transform.rotation.w = gimbal_q[3]

        self.tf_broadcaster.sendTransform([drone_tf, gimbal_tf])

        # PoseStamped for drone
        drone_pose = PoseStamped()
        drone_pose.header.stamp = now
        drone_pose.header.frame_id = 'map'
        drone_pose.pose.position.x = drone_pos[0]
        drone_pose.pose.position.y = drone_pos[1]
        drone_pose.pose.position.z = drone_pos[2]
        drone_pose.pose.orientation.x = drone_q[0]
        drone_pose.pose.orientation.y = drone_q[1]
        drone_pose.pose.orientation.z = drone_q[2]
        drone_pose.pose.orientation.w = drone_q[3]
        self.drone_pose_pub.publish(drone_pose)

        # PoseStamped for gimbal
        gimbal_pose = PoseStamped()
        gimbal_pose.header.stamp = now
        gimbal_pose.header.frame_id = 'base_link'
        gimbal_pose.pose.orientation.x = gimbal_q[0]
        gimbal_pose.pose.orientation.y = gimbal_q[1]
        gimbal_pose.pose.orientation.z = gimbal_q[2]
        gimbal_pose.pose.orientation.w = gimbal_q[3]
        self.gimbal_pose_pub.publish(gimbal_pose)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
