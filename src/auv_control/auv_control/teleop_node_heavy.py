#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Wrench
from rclpy.qos import QoSProfile, DurabilityPolicy


class Bluerov2Teleop(Node):
    def __init__(self):
        super().__init__('bluerov2_teleop')

        # Joystick parameters
        self.declare_parameter('surge_axis', 4)
        self.declare_parameter('sway_axis', 3)
        self.declare_parameter('heave_axis', 1)
        self.declare_parameter('yaw_axis', 0)

        # Buttons
        self.declare_parameter('deadman_button', -1)
        self.declare_parameter('mode_toggle_btn', 2)
        self.declare_parameter('station_keep_btn', 3)

        # Force / torque limits
        # self.declare_parameter('surge_force', 10.0)
        # self.declare_parameter('sway_force', 10.0)
        # self.declare_parameter('heave_force', 10.0)

        self.declare_parameter('surge_force', 125.7046)
        self.declare_parameter('sway_force', 125.7046)
        self.declare_parameter('heave_force', 177.8000)

        # self.declare_parameter('roll_torque', 15.0)
        # self.declare_parameter('pitch_torque', 3.0)
        # self.declare_parameter('yaw_torque', 3.0)

        self.declare_parameter('roll_torque', 38.7604)
        self.declare_parameter('pitch_torque', 21.3360)
        self.declare_parameter('yaw_torque', 33.5686)

        self.declare_parameter('deadzone', 0.05)

        # Get parameters
        self.surge_axis = self.get_parameter('surge_axis').value
        self.sway_axis = self.get_parameter('sway_axis').value
        self.heave_axis = self.get_parameter('heave_axis').value
        self.yaw_axis = self.get_parameter('yaw_axis').value

        self.deadman_button = self.get_parameter('deadman_button').value
        self.mode_toggle_btn = self.get_parameter('mode_toggle_btn').value
        self.deadzone = self.get_parameter('deadzone').value

        self.surge_force = self.get_parameter('surge_force').value
        self.sway_force = self.get_parameter('sway_force').value
        self.heave_force = self.get_parameter('heave_force').value

        self.roll_torque = self.get_parameter('roll_torque').value
        self.pitch_torque = self.get_parameter('pitch_torque').value
        self.yaw_torque = self.get_parameter('yaw_torque').value

        self.station_keep_btn = self.get_parameter('station_keep_btn').value

        #Change axes of the surge/sway to roll/pitch
        self.toggled_mode = False
        #X button state whether pressed or not
        self.previous_toggle_btn = 0

        #Station keep toggle
        self.station_keep_enabled = False
        self.previous_station_keep_btn = 0

        # ROS infrastructure
        self.wrench_pub = self.create_publisher(
            Wrench,
            '/bluerov2_heavy/cmd_wrench_joy',
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.station_keep_pub = self.create_publisher(
            Bool,
            '/bluerov2_heavy/station_keep_enable',
            latched_qos
        )

        self.publish_station_keep()

        self.get_logger().info(
            'Bluerov2 teleop started'
        )

    def get_axis(self, joy, axis):

        if axis < 0:
            return 0.0

        if axis >= len(joy.axes):
            return 0.0

        value = joy.axes[axis]

        # Deadzone
        if abs(value) < self.deadzone:
            return 0.0

        # Rescale after deadzone
        sign = 1.0 if value >= 0.0 else -1.0

        value = (
            abs(value) - self.deadzone
        ) / (1.0 - self.deadzone)

        return sign * value

    def update_mode(self, joy):
        button = self.mode_toggle_btn

        #Check for the index of the button
        if button < 0 or button >= len(joy.buttons):
            return

        current_button = joy.buttons[button]
        if current_button == 1 and self.previous_toggle_btn == 0:
            self.toggled_mode = not self.toggled_mode

            if self.toggled_mode:
                self.get_logger().info(
                    'Toggle DETECTED'
                    'Surge axis -> roll'
                    'Sway axis -> pitch'
                )

            else:
                self.get_logger().info(
                    'Normal operation'
                    'Surge axis -> surge'
                    'Sway axis -> sway'
                )

        self.previous_toggle_btn = current_button


    def joy_callback(self, joy):
        self.update_mode(joy)
        self.update_station_keep(joy)

        # Deadman
        enabled = True

        if self.deadman_button >= 0:
            if self.deadman_button >= len(joy.buttons):
                enabled = False
            else:
                enabled = (
                    joy.buttons[self.deadman_button] == 1
                )
        wrench = Wrench()

        if not enabled:
            self.wrench_pub.publish(wrench)
            return

        # Read joystick
        surge = self.get_axis(
            joy,
            self.surge_axis
        )

        sway = self.get_axis(
            joy,
            self.sway_axis
        )

        heave = self.get_axis(
            joy,
            self.heave_axis
        )

        roll = self.get_axis(
            joy,
            self.sway_axis
        )

        pitch = self.get_axis(
            joy,
            self.surge_axis
        )

        yaw = self.get_axis(
            joy,
            self.yaw_axis
        )

        if self.toggled_mode:
            # Map joystick to wrench
            wrench.force.x = 0.0

            wrench.force.y = 0.0

            wrench.torque.x = (roll * self.roll_torque)
            wrench.torque.y = (pitch * self.pitch_torque)
        else:
            wrench.force.x = (surge * self.surge_force)
            wrench.force.y = (sway * self.sway_force)

            wrench.torque.x = 0.0
            wrench.torque.y = 0.0

        wrench.force.z = (
            heave * self.heave_force
        )

        wrench.torque.z = (
            yaw * self.yaw_torque
        )

        self.wrench_pub.publish(wrench)

    def publish_station_keep(self):
        msg = Bool()
        msg.data = self.station_keep_enabled
        self.station_keep_pub.publish(msg)

    def update_station_keep(self, joy):
        button = self.station_keep_btn

        #Check for the index of the button
        if button < 0 or button >= len(joy.buttons):
            return

        current_button = joy.buttons[button]
        if current_button ==1 and self.previous_station_keep_btn == 0:
            self.station_keep_enabled = not self.station_keep_enabled
            self.publish_station_keep()

            if self.station_keep_enabled:
                self.get_logger().info('Station keeping ENABLED')
            else:
                self.get_logger().info('Station keeping DISABLED')

        self.previous_station_keep_btn = current_button


def main(args=None):

    rclpy.init(args=args)

    node = Bluerov2Teleop()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()