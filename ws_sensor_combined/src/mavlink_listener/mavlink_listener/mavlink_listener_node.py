import rclpy
from rclpy.node import Node
from px4_msgs.msg import GimbalDeviceSetAttitude
from builtin_interfaces.msg import Time

from pymavlink import mavutil
import threading
import time

class MavlinkListenerNode(Node):
    def __init__(self):
        super().__init__('mavlink_listener_node')

        self.publisher_ = self.create_publisher(GimbalDeviceSetAttitude, '/fmu/in/gimbal_device_set_attitude', 10)

        self.get_logger().info('Starting MAVLink listener on UDP port 14550...')
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14445') # requires mavlink forwarding in QGC to be checked (or mavlink forwarded to this port)

        self.thread = threading.Thread(target=self.mavlink_loop, daemon=True)
        self.thread.start()

    def mavlink_loop(self):
        while rclpy.ok():
            msg = self.mav.recv_match(type='GIMBAL_DEVICE_SET_ATTITUDE', blocking=True, timeout=1.0)
            if msg is not None:
                ros_msg = GimbalDeviceSetAttitude()

                # Timestamp (convert to ROS time)
                now = int(self.get_clock().now().nanoseconds / 1000)  # Convert to microseconds
                ros_msg.timestamp = now


                # Set fields
                ros_msg.q = list(msg.q)
                ros_msg.target_system = msg.target_system
                ros_msg.target_component = msg.target_component
                ros_msg.flags = msg.flags

                # Optional fields if needed:
                # ros_msg.angular_velocity_x = ...
                # ros_msg.angular_velocity_y = ...
                # ros_msg.angular_velocity_z = ...

                self.publisher_.publish(ros_msg)
                self.get_logger().info(f'Published GIMBAL_DEVICE_SET_ATTITUDE to PX4')

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
