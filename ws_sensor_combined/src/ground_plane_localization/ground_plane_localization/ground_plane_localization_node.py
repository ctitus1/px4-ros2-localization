#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial.transform import Rotation as R

class RayIntersectionNode(Node):
    def __init__(self):
        super().__init__('ground_plane_localization_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, 'ground_intersection_marker', 10)

        self.timer = self.create_timer(1.0, self.compute_intersection)
        self.ground_z = 0.0  # Set your home Z ground level here, relative to home frame I think

    def compute_intersection(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'home_frame',  # Target frame (world)
                'gimbal_frame',  # Source frame (gimbal)
                rclpy.time.Time()
            )

            origin = np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])

            q = t.transform.rotation
            q_xyzw = [q.x, q.y, q.z, q.w]
            r = R.from_quat(q_xyzw)

            ray_dir = r.apply([1, 0, 0])  # +X in gimbal frame

            dz = self.ground_z - origin[2]
            if ray_dir[2] > 0:
                self.get_logger().warn("Ray is above parallel to the ground plane, discarding.")
                return

            t_param = dz / ray_dir[2]
            intersection = origin + t_param * ray_dir

            distance = np.linalg.norm(intersection - origin)
            if distance > 50.0:
                self.get_logger().warn(f"Intersection too far ({distance:.2f}m), discarding.")
                return

            marker = Marker()
            marker.header.frame_id = 'home_frame'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'ground_intersection'
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = intersection[0]
            marker.pose.position.y = intersection[1]
            marker.pose.position.z = intersection[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # diameter in meters
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime.sec = 1  # lasts 1 second (matches timer rate)

            self.marker_pub.publish(marker)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

def main():
    rclpy.init()
    node = RayIntersectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
