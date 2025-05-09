import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleAttitude, GimbalDeviceSetAttitude, VehicleLocalPosition, HomePosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class Px4ListenerNode(Node):
    def __init__(self):
        super().__init__('px4_listener_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # self.create_subscription(
        #     VehicleAttitude,
        #     '/fmu/out/vehicle_attitude',
        #     self.attitude_callback,
        #     qos_profile
        # )

        self.create_subscription(
            GimbalDeviceSetAttitude,
            '/fmu/out/gimbal_set_attitude',
            self.gimbal_callback,
            qos_profile
        )

        # self.create_subscription(
        #     VehicleLocalPosition,
        #     '/fmu/out/vehicle_local_position',
        #     self.local_pos_callback,
        #     qos_profile
        # )

        # self.create_subscription(
        #     HomePosition,
        #     '/fmu/out/home_position',
        #     self.home_callback,
        #     qos_profile
        # )

        self.get_logger().info('PX4 listener node started.')

    def attitude_callback(self, msg):
        q = msg.q
        self.get_logger().info(f'VEHICLE_ATTITUDE: q=[{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}]')

    def gimbal_callback(self, msg):
        q = msg.q
        self.get_logger().info(f'GIMBAL_DEVICE_ATTITUDE_STATUS: q=[{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}]')

    def local_pos_callback(self, msg):
        self.get_logger().info(f'VEHICLE_LOCAL_POSITION: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')

    def home_callback(self, msg):
        self.get_logger().info(
            f'HOME_POSITION: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}, '
            f'x={msg.x}, y={msg.y}, z={msg.z}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = Px4ListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
