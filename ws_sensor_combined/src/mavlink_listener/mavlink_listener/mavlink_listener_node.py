import rclpy
from rclpy.node import Node
from px4_msgs.msg import GimbalDeviceSetAttitude, HomePosition

from pymavlink import mavutil
import threading

class MavlinkListenerNode(Node):
    def __init__(self):
        super().__init__('mavlink_listener_node')

        self.gimbal_pub = self.create_publisher(GimbalDeviceSetAttitude, '/fmu/in/gimbal_device_set_attitude', 10)
        self.home_pub = self.create_publisher(HomePosition, '/fmu/in/home_position', 10)

        self.get_logger().info('Starting MAVLink listener on UDP port 14550...')
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14445')  # QGC or PX4 forwarding must be enabled

        self.thread = threading.Thread(target=self.mavlink_loop, daemon=True)
        self.thread.start()

    def mavlink_loop(self):
        while rclpy.ok():
            msg = self.mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue

            msg_type = msg.get_type()

            if msg_type == 'GIMBAL_DEVICE_SET_ATTITUDE':
                ros_msg = GimbalDeviceSetAttitude()
                ros_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
                ros_msg.q = list(msg.q)
                ros_msg.target_system = msg.target_system
                ros_msg.target_component = msg.target_component
                ros_msg.flags = msg.flags

                self.gimbal_pub.publish(ros_msg)
                self.get_logger().info('Published GIMBAL_DEVICE_SET_ATTITUDE to PX4')

            elif msg_type == 'HOME_POSITION':
                ros_msg = HomePosition()
                ros_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

                # Location
                ros_msg.lat = msg.latitude / 1e7
                ros_msg.lon = msg.longitude / 1e7
                ros_msg.alt = msg.altitude / 1e3

                # Local position (in millimeters)
                ros_msg.x = msg.x
                ros_msg.y = msg.y
                ros_msg.z = msg.z

                self.home_pub.publish(ros_msg)
                self.get_logger().info('Published HOME_POSITION to PX4')

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
