#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_pkgs.msg import Dvl
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped


class DvlTwistToCov(Node):
    def __init__(self):
        super().__init__('dvl_twist_to_cov')

        # Frame the velocity is expressed in (the dvl link in the xacro)
        self.declare_parameter('frame_id', 'bluerov2/dvl')

        # White noise stddev from the plugin (m/s), and a factor to inflate
        # the variance since the filter cannot observe the bias
        self.declare_parameter('stddev', 0.00)
        self.declare_parameter('variance_scale', 0.0)

        self.frame_id = self.get_parameter('frame_id').value
        stddev = self.get_parameter('stddev').value
        scale = self.get_parameter('variance_scale').value
        self.variance = scale * stddev ** 2
        self.altitude = float('nan')
        self.dvl_pub = self.create_publisher(Dvl, 'dvl', 10)
        self.altitude_sub = self.create_subscription(
            Float64,
            'dvl_altitude',
            self.altitude_callback,
            10
        )

        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            'dvl_twist_cov',
            10
        )

        self.twist_sub = self.create_subscription(
            Twist,
            'dvl_twist',
            self.twist_callback,
            10
        )

        self.get_logger().info('DVL twist converter started')

    def altitude_callback(self, msg):
        self.altitude = msg.data

    def twist_callback(self, msg):
        out = TwistWithCovarianceStamped()
        # The bridge drops the Gazebo header, so stamp with (sim) time now
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist.twist = msg

        cov = [0.0] * 36
        for i in (0, 7, 14):
            cov[i] = self.variance
        for i in (21, 28, 35):
            cov[i] = -1.0  # angular velocity not available, as in UUV
        out.twist.covariance = cov

        self.twist_pub.publish(out)

        dvl = Dvl()
        dvl.header = out.header
        dvl.velocity = msg.linear
        dvl.velocity_covariance = [
            self.variance, 0.0, 0.0,
            0.0, self.variance, 0.0,
            0.0, 0.0, self.variance
        ]
        dvl.altitude = self.altitude
        self.dvl_pub.publish(dvl)


def main(args=None):
    rclpy.init(args=args)
    node = DvlTwistToCov()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()