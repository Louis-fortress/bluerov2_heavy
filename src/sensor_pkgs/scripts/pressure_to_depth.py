#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped


class PressureToDepth(Node):
    def __init__(self):
        super().__init__('pressure_to_depth')

        # These must match the PressureSensor plugin settings in the xacro
        self.declare_parameter('atm_pressure', 101325.0)
        self.declare_parameter('fluid_density', 1000.0)
        self.declare_parameter('gravity', 9.80665)
        self.declare_parameter('surface_z', 0.0)

        # Fallback if the incoming message has no variance (Pa^2)
        self.declare_parameter('default_variance', 400.0)

        # Frame the z measurement is expressed in (match robot_localization)
        self.declare_parameter('frame_id', 'bluerov2/odom')

        # Covariance for the unmeasured axes (x, y, roll, pitch, yaw)
        self.declare_parameter('unused_covariance', 1.0e6)

        self.p_atm = self.get_parameter('atm_pressure').value
        self.rho = self.get_parameter('fluid_density').value
        self.g = self.get_parameter('gravity').value
        self.surface_z = self.get_parameter('surface_z').value
        self.default_variance = self.get_parameter('default_variance').value
        self.frame_id = self.get_parameter('frame_id').value
        self.unused_cov = self.get_parameter('unused_covariance').value

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'depth_pose',
            10
        )

        self.pressure_sub = self.create_subscription(
            FluidPressure,
            'pressure',
            self.pressure_callback,
            10
        )

        self.get_logger().info('Pressure to depth converter started')

    def pressure_callback(self, msg):
        rho_g = self.rho * self.g

        depth = (msg.fluid_pressure - self.p_atm) / rho_g
        variance_pa = msg.variance if msg.variance > 0.0 else self.default_variance

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id

        out.pose.pose.position.z = self.surface_z - depth
        out.pose.pose.orientation.w = 1.0

        # 6x6 row-major covariance: x, y, z, roll, pitch, yaw
        cov = [0.0] * 36
        for i in (0, 7, 21, 28, 35):
            cov[i] = self.unused_cov
        cov[14] = variance_pa / (rho_g ** 2)
        out.pose.covariance = cov

        self.pose_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PressureToDepth()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()