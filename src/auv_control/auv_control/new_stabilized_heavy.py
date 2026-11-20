#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench


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


class HeavyStationKeep(Node):
    def __init__(self):
        super().__init__('heavy_station_keep')

        # Topics
        self.declare_parameter('odom_topic', '/bluerov2_heavy/odometry/filtered')
        self.declare_parameter('wrench_topic', '/bluerov2_heavy/wrench')
        self.declare_parameter('station_keep_topic', '/bluerov2_heavy/station_keep_enable')

        # Loop / behaviour
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('odom_timeout', 0.5)
        # True: hold the roll/pitch the vehicle has when toggled
        # False: latch x, y, z and yaw but keep the vehicle level (roll = pitch = 0)
        self.declare_parameter('latch_roll_pitch', True)

        self.control_rate = self.get_parameter('control_rate').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.latch_roll_pitch = self.get_parameter('latch_roll_pitch').value

        # PID gains. Starting values from the Gazebo xacro plant (inertia, added
        # mass, linear damping), NOT tuned in simulation.
        self.pos_x_pid = self.make_pid('pos_x', 29.0, 2.0, 4.0, 20.0)
        self.pos_y_pid = self.make_pid('pos_y', 39.6, 2.0, 9.0, 20.0)
        self.depth_pid = self.make_pid('depth', 11.9403874387169, 4.07948623930136, 3.24177393275929, 40.0)
        self.roll_pid = self.make_pid('roll', 6.6, 5.0, 6.4, 20.0)
        self.pitch_pid = self.make_pid('pitch', 12.8, 8.0, 9.5, 20.0)
        self.yaw_pid = self.make_pid('yaw', 12.7, 5.0, 9.0, 10.0)

        self.pids = (
            self.pos_x_pid, self.pos_y_pid, self.depth_pid,
            self.roll_pid, self.pitch_pid, self.yaw_pid
        )

        # State
        self.odom = None
        self.last_odom_time = None
        self.last_time = self.get_clock().now()

        self.enabled = False
        self.latched = False

        self.target = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
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

        # Must match the latched (TRANSIENT_LOCAL) publisher in the teleop node
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.station_keep_sub = self.create_subscription(
            Bool,
            self.get_parameter('station_keep_topic').value,
            self.station_keep_callback,
            latched_qos
        )

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Bluerov2 station keep started (idle until enabled)')

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
        self.odom = msg
        self.last_odom_time = self.get_clock().now()

    def station_keep_callback(self, msg):
        if msg.data == self.enabled:
            return

        self.enabled = msg.data
        self.latched = False

        for pid in self.pids:
            pid.reset()

        if self.enabled:
            self.get_logger().info('Station keeping ENABLED')
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

    def latch_target(self, s):
        self.target = {
            'x': s['x'],
            'y': s['y'],
            'z': s['z'],
            'roll': s['roll'] if self.latch_roll_pitch else 0.0,
            'pitch': s['pitch'] if self.latch_roll_pitch else 0.0,
            'yaw': s['yaw'],
        }
        self.latched = True

        self.get_logger().info(
            'Target latched: '
            f"x={s['x']:.2f} y={s['y']:.2f} z={s['z']:.2f} "
            f"roll={math.degrees(self.target['roll']):.1f} "
            f"pitch={math.degrees(self.target['pitch']):.1f} "
            f"yaw={math.degrees(s['yaw']):.1f} deg"
        )

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    def control_loop(self):
        # Idle: publish nothing so the stabilized node owns the wrench topic
        if not self.enabled:
            return

        now = self.get_clock().now()
        wrench = Wrench()

        # Never drive on missing or stale odometry
        odom_ok = (
            self.odom is not None
            and (now - self.last_odom_time).nanoseconds * 1e-9 < self.odom_timeout
        )

        if not odom_ok:
            for pid in self.pids:
                pid.reset()
            self.wrench_pub.publish(wrench)
            self.get_logger().warn(
                'Odometry missing or stale, publishing zero wrench',
                throttle_duration_sec=2.0
            )
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.1:
            dt = 1.0 / self.control_rate

        s = self.read_state()

        # Latch the current pose on the first valid tick after enabling
        if not self.latched:
            self.latch_target(s)

        t = self.target
        R = s['R']

        # --- x / y position, error rotated from world into the body frame ---
        ex_w = t['x'] - s['x']
        ey_w = t['y'] - s['y']

        ex_b = R[0][0] * ex_w + R[1][0] * ey_w
        ey_b = R[0][1] * ex_w + R[1][1] * ey_w

        wrench.force.x = self.pos_x_pid.update(ex_b, s['vx'], dt)
        wrench.force.y = self.pos_y_pid.update(ey_b, s['vy'], dt)

        # --- Depth ---
        wrench.force.z = self.depth_pid.update(
            t['z'] - s['z'], s['world_vz'], dt
        )

        # --- Attitude ---
        wrench.torque.x = self.roll_pid.update(
            wrap_angle(t['roll'] - s['roll']), s['roll_rate'], dt
        )
        wrench.torque.y = self.pitch_pid.update(
            wrap_angle(t['pitch'] - s['pitch']), s['pitch_rate'], dt
        )
        wrench.torque.z = self.yaw_pid.update(
            wrap_angle(t['yaw'] - s['yaw']), s['yaw_rate'], dt
        )

        self.wrench_pub.publish(wrench)

        self.get_logger().debug(
            f'err body x={ex_b:.3f} y={ey_b:.3f} z={t["z"] - s["z"]:.3f} '
            f'yaw={math.degrees(wrap_angle(t["yaw"] - s["yaw"])):.1f} deg',
            throttle_duration_sec=1.0
        )


def main(args=None):

    rclpy.init(args=args)

    node = HeavyStationKeep()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()