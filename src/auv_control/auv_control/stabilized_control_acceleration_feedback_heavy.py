#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Vector3


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PID:
    """
    PID with derivative on measurement (the caller supplies the measured rate,
    so there is no differencing and no derivative kick) and clamping anti-windup.
    """

    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, rate, dt):
        p_term = self.kp * error
        d_term = -self.kd * rate
        integral = self.integral + self.ki * error * dt

        output = p_term + integral + d_term

        if abs(output) > self.limit:
            # Saturated: only keep integrating if it pulls the output back in
            if error * output > 0.0:
                integral = self.integral
            output = clamp(output, self.limit)

        self.integral = integral
        return output


class HeavyStabilizedControl(Node):
    def __init__(self):
        super().__init__('bluerov2_stabilized_control')

        # Topics
        self.declare_parameter('odom_topic', '/bluerov2_heavy/odometry/filtered')
        self.declare_parameter('joy_wrench_topic', '/bluerov2_heavy/cmd_wrench_joy')
        self.declare_parameter('wrench_topic', '/bluerov2_heavy/wrench')
        self.declare_parameter('station_keep_topic', '/bluerov2_heavy/station_keep_enable')

        # Loop / input handling
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('joy_timeout', 0.5)
        self.declare_parameter('input_threshold', 1e-3)
        self.declare_parameter('attitude_return_time', 2.0)
        self.declare_parameter('roll_limit_deg', 60.0)
        self.declare_parameter('pitch_limit_deg', 60.0)

        self.control_rate = self.get_parameter('control_rate').value
        self.joy_timeout = self.get_parameter('joy_timeout').value
        self.input_threshold = self.get_parameter('input_threshold').value
        self.attitude_return_time = self.get_parameter('attitude_return_time').value
        self.roll_limit = math.radians(self.get_parameter('roll_limit_deg').value)
        self.pitch_limit = math.radians(self.get_parameter('pitch_limit_deg').value)

        # PID gains (stabilised mode, from MATLAB tuning)
        #the D term acts on the measured rate from the EKF twist
        self.depth_pid = self.make_pid('depth', 11.9403874387169, 4.07948623930136, 3.24177393275929, 40.0)
        self.roll_pid = self.make_pid('roll', 16.0, 2.0, 17.0, 20.0)
        self.pitch_pid = self.make_pid('pitch', 26.0, 3.0, 26.0, 20.0)
        self.yaw_pid = self.make_pid('yaw', 15.8, 1.5, 19.0, 10.0)

        # vel_*: brake body velocity to zero when the stick is released
        # pos_*: hold position while station keeping (kd acts as velocity damping)
        self.vel_x_pid = self.make_pid('vel_x', 30.0, 3.0, 0.0, 20.0)
        self.vel_y_pid = self.make_pid('vel_y', 30.0, 3.0, 0.0, 20.0)
        self.pos_x_pid = self.make_pid('pos_x', 8.0, 0.5, 25.0, 20.0)
        self.pos_y_pid = self.make_pid('pos_y', 8.0, 0.5, 25.0, 20.0)

        # State
        self.odom = None
        self.joy_cmd = Wrench()
        self.last_joy_time = None
        self.last_time = self.get_clock().now()

        self.station_keep = False

        self.depth_sp = 0.0
        self.yaw_sp = 0.0
        self.x_sp = 0.0
        self.y_sp = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0

        # Roll/pitch return-to-zero ramp: [angle when stick was last active, time]
        self.ramp = {
            'roll': [0.0, self.get_clock().now()],
            'pitch': [0.0, self.get_clock().now()],
        }

        # ROS infrastructure
        self.wrench_pub = self.create_publisher(
            Wrench,
            self.get_parameter('wrench_topic').value,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10
        )

        self.joy_sub = self.create_subscription(
            Wrench,
            self.get_parameter('joy_wrench_topic').value,
            self.joy_callback,
            10
        )

        # Must match the latched (TRANSIENT_LOCAL) publisher in the teleop node
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.station_keep_sub = self.create_subscription(
            Bool,
            self.get_parameter('station_keep_topic').value,
            self.station_keep_callback,
            latched_qos
        )

        #Visualization topics 
        ns = '/bluerov2_heavy/stabilized'
        self.position_pub = self.create_publisher(Vector3, f'{ns}/position', 10)
        self.position_target_pub = self.create_publisher(Vector3, f'{ns}/position_target', 10)
        self.position_error_pub = self.create_publisher(Vector3, f'{ns}/position_error', 10)
        self.attitude_pub = self.create_publisher(Vector3, f'{ns}/attitude_deg', 10)  # roll, pitch, yaw
        self.attitude_target_pub = self.create_publisher(Vector3, f'{ns}/attitude_target_deg', 10)
        self.attitude_error_pub = self.create_publisher(Vector3, f'{ns}/attitude_error_deg', 10)
        self.attitude_limit_pub = self.create_publisher(Vector3, f'{ns}/attitude_limit_deg', 10)
        self.attitude_margin_pub = self.create_publisher(Vector3, f'{ns}/attitude_limit_margin_deg', 10)
        self.velocity_pub = self.create_publisher(Vector3, f'{ns}/velocity', 10)  # body vx, vy, world vz
        self.station_keep_state_pub = self.create_publisher(Bool, f'{ns}/station_keep_active', 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Bluerov2 stabilized control started')

    def make_pid(self, name, kp, ki, kd, limit):
        self.declare_parameter(f'{name}_kp', kp)
        self.declare_parameter(f'{name}_ki', ki)
        self.declare_parameter(f'{name}_kd', kd)
        self.declare_parameter(f'{name}_limit', limit)

        return PID(
            self.get_parameter(f'{name}_kp').value,
            self.get_parameter(f'{name}_ki').value,
            self.get_parameter(f'{name}_kd').value,
            self.get_parameter(f'{name}_limit').value
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        first = self.odom is None
        self.odom = msg

        if first:
            self.latch_setpoints()
            self.get_logger().info('First odometry received, setpoints latched')

    def joy_callback(self, msg):
        self.joy_cmd = msg
        self.last_joy_time = self.get_clock().now()

    def limit_attitude_command(self, roll_cmd, pitch_cmd):
        #Roll protection
        if roll_cmd > 0.0 and self.current_roll >= self.roll_limit:
            self.get_logger().info(
                f"Roll limit reached: roll={math.degrees(self.current_roll):.1f} deg",
                throttle_duration_sec=1.0
            )
            roll_cmd = 0.0
        elif roll_cmd < 0.0 and self.current_roll <= -self.roll_limit:
            self.get_logger().info(
                f"Roll limit reached: roll={math.degrees(self.current_roll):.1f} deg",
                throttle_duration_sec=1.0
            )
            roll_cmd = 0.0
        #Pitch protection
        if pitch_cmd > 0.0 and self.current_pitch >= self.pitch_limit:
            self.get_logger().info(
                f"Pitch limit reached: pitch={math.degrees(self.current_pitch):.1f} deg",
                throttle_duration_sec=1.0
            )
            pitch_cmd = 0.0

        elif pitch_cmd < 0.0 and self.current_pitch <= -self.pitch_limit:
            self.get_logger().info(
                f"Roll limit reached: roll={math.degrees(self.current_roll):.1f} deg",
                throttle_duration_sec=1.0
            )
            pitch_cmd = 0.0
        return roll_cmd, pitch_cmd

    def station_keep_callback(self, msg):
        if msg.data == self.station_keep:
            return

        self.station_keep = msg.data

        for pid in (self.vel_x_pid, self.vel_y_pid, self.pos_x_pid, self.pos_y_pid):
            pid.reset()

        if self.station_keep:
            self.latch_setpoints()
            self.get_logger().info('Station keeping ENABLED, holding position')
        else:
            self.get_logger().info('Station keeping DISABLED')

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def read_state(self):
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist

        q = pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w

        # Body -> world rotation matrix
        R = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]

        roll = math.atan2(R[2][1], R[2][2])
        pitch = -math.asin(clamp(R[2][0], 1.0))
        yaw = math.atan2(R[1][0], R[0][0])

        # Body velocity (EKF twist is expressed in base_link)
        vx = twist.linear.x
        vy = twist.linear.y
        vz = twist.linear.z

        # Body rates -> Euler rates
        p, q_rate, r = twist.angular.x, twist.angular.y, twist.angular.z
        cos_pitch = math.cos(pitch)
        if abs(cos_pitch) < 1e-3:
            cos_pitch = math.copysign(1e-3, cos_pitch if cos_pitch != 0.0 else 1.0)
        sin_roll, cos_roll = math.sin(roll), math.cos(roll)
        tan_pitch = math.tan(pitch)

        roll_rate = p + sin_roll * tan_pitch * q_rate + cos_roll * tan_pitch * r
        pitch_rate = cos_roll * q_rate - sin_roll * r
        yaw_rate = (sin_roll * q_rate + cos_roll * r) / cos_pitch

        # Vertical velocity in the world frame
        world_vz = R[2][0] * vx + R[2][1] * vy + R[2][2] * vz

        return {
            'R': R,
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'vx': vx,
            'vy': vy,
            'world_vz': world_vz,
            'roll_rate': roll_rate,
            'pitch_rate': pitch_rate,
            'yaw_rate': yaw_rate,
        }

    def latch_setpoints(self):
        if self.odom is None:
            return

        s = self.read_state()
        self.x_sp = s['x']
        self.y_sp = s['y']
        self.depth_sp = s['z']
        self.yaw_sp = s['yaw']

    def attitude_setpoint(self, axis, angle, active, now):
        """
        Roll/pitch reference. While the stick is active it follows the vehicle.
        After release it eases from the last angle to 0 with a smoothstep over
        attitude_return_time.
        """
        if active:
            self.ramp[axis] = [angle, now]
            return angle

        start, t0 = self.ramp[axis]
        elapsed = (now - t0).nanoseconds * 1e-9
        s = min(max(elapsed / max(self.attitude_return_time, 1e-3), 0.0), 1.0)
        s = s * s * (3.0 - 2.0 * s)

        return start * (1.0 - s)

    def publish_debug(self, s, roll_sp, pitch_sp):
        # State, targets and errors for plotting. Errors are target - current.
        nan = float('nan')

        depth = -s['z']
        depth_target = -self.depth_sp

        # x/y targets only exist while station keeping; NaN gives a gap in the plot
        if self.station_keep:
            x_t, y_t = self.x_sp, self.y_sp
        else:
            x_t, y_t = nan, nan

        self.position_pub.publish(Vector3(x=s['x'], y=s['y'], z=depth))
        self.position_target_pub.publish(Vector3(x=x_t, y=y_t, z=depth_target))
        self.position_error_pub.publish(Vector3(
            x=x_t - s['x'], y=y_t - s['y'], z=depth_target - depth
        ))

        yaw_error = wrap_angle(self.yaw_sp - s['yaw'])
        self.attitude_pub.publish(Vector3(
            x=math.degrees(s['roll']), y=math.degrees(s['pitch']), z=math.degrees(s['yaw'])
        ))
        self.attitude_target_pub.publish(Vector3(
            x=math.degrees(roll_sp), y=math.degrees(pitch_sp), z=math.degrees(self.yaw_sp)
        ))
        self.attitude_error_pub.publish(Vector3(
            x=math.degrees(roll_sp - s['roll']),
            y=math.degrees(pitch_sp - s['pitch']),
            z=math.degrees(yaw_error),
        ))

        self.attitude_limit_pub.publish(Vector3(
            x=math.degrees(self.roll_limit), y=math.degrees(self.pitch_limit), z=0.0
        ))
        self.attitude_margin_pub.publish(Vector3(
            x=math.degrees(self.roll_limit - abs(s['roll'])),
            y=math.degrees(self.pitch_limit - abs(s['pitch'])),
            z=0.0,
        ))

        self.velocity_pub.publish(Vector3(x=s['vx'], y=s['vy'], z=s['world_vz']))
        self.station_keep_state_pub.publish(Bool(data=self.station_keep))

    # Control loop
    def control_loop(self):
        wrench = Wrench()

        if self.odom is None:
            self.wrench_pub.publish(wrench)
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.1:
            dt = 1.0 / self.control_rate

        s = self.read_state()

        self.current_roll = s['roll']
        self.current_pitch = s['pitch']

        # Stick input: ignored while station keeping, zero if the teleop goes quiet
        joy_alive = (
            self.last_joy_time is not None
            and (now - self.last_joy_time).nanoseconds * 1e-9 < self.joy_timeout
        )

        if joy_alive and not self.station_keep:
            cmd = self.joy_cmd
        else:
            cmd = Wrench()

        eps = self.input_threshold

        # --- Surge / sway ---
        if self.station_keep:
            ex_w = self.x_sp - s['x']
            ey_w = self.y_sp - s['y']
            R = s['R']

            # World error -> body frame (R transposed)
            ex_b = R[0][0] * ex_w + R[1][0] * ey_w
            ey_b = R[0][1] * ex_w + R[1][1] * ey_w

            wrench.force.x = self.pos_x_pid.update(ex_b, s['vx'], dt)
            wrench.force.y = self.pos_y_pid.update(ey_b, s['vy'], dt)
        else:
            if abs(cmd.force.x) > eps:
                wrench.force.x = cmd.force.x
                self.vel_x_pid.reset()
            else:
                wrench.force.x = self.vel_x_pid.update(-s['vx'], 0.0, dt)

            if abs(cmd.force.y) > eps:
                wrench.force.y = cmd.force.y
                self.vel_y_pid.reset()
            else:
                wrench.force.y = self.vel_y_pid.update(-s['vy'], 0.0, dt)

        # --- Heave (depth hold) ---
        if abs(cmd.force.z) > eps:
            wrench.force.z = cmd.force.z
            self.depth_pid.reset()
            self.depth_sp = s['z']
        else:
            wrench.force.z = self.depth_pid.update(
                self.depth_sp - s['z'], s['world_vz'], dt
            )

        # --- Roll ---
        roll_active = abs(cmd.torque.x) > eps
        roll_sp = self.attitude_setpoint('roll', s['roll'], roll_active, now)

        if roll_active:
            wrench.torque.x, _ = self.limit_attitude_command(cmd.torque.x, 0)
            self.roll_pid.reset()
        else:
            #Return back to level
            wrench.torque.x = self.roll_pid.update(
                roll_sp - s['roll'], s['roll_rate'], dt
            )

        # --- Pitch ---
        pitch_active = abs(cmd.torque.y) > eps
        pitch_sp = self.attitude_setpoint('pitch', s['pitch'], pitch_active, now)

        if pitch_active:
            _, wrench.torque.y = self.limit_attitude_command(0,cmd.torque.y)
            self.pitch_pid.reset()
        else:
            #Return back to level
            wrench.torque.y = self.pitch_pid.update(
                pitch_sp - s['pitch'], s['pitch_rate'], dt
            )

        # --- Yaw (heading hold) ---
        if abs(cmd.torque.z) > eps:
            wrench.torque.z = cmd.torque.z
            self.yaw_pid.reset()
            self.yaw_sp = s['yaw']
        else:
            wrench.torque.z = self.yaw_pid.update(
                wrap_angle(self.yaw_sp - s['yaw']), s['yaw_rate'], dt
            )

        self.wrench_pub.publish(wrench)
        self.publish_debug(s, roll_sp, pitch_sp)


def main(args=None):

    rclpy.init(args=args)

    node = HeavyStabilizedControl()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()