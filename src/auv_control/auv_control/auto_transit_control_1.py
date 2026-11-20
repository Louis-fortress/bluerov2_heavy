#!/usr/bin/env python3
"""
Six-DOF sequenced setpoint controller for BlueROV2 Heavy.

Pilot interface (geometry_msgs/Twist):
  linear.x/y/z  -> relative world-frame X/Y/Z offsets [m] from the pose
                   captured when the node starts (or when reset is received).
  angular.x/y/z -> absolute world-frame roll/pitch/yaw setpoints [rad].

The node publishes geometry_msgs/Wrench directly. Do not run another
controller that publishes to the same wrench topic at the same time.
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Vector3
from auv_control_msgs.msg import SixDofSetpoint

def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def clamp(value, limit):
    return max(-limit, min(limit, value))


class PID:
    """PID using measured rate for derivative and conditional anti-windup."""

    def __init__(self, kp, ki, kd, output_limit, integral_limit=None):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = abs(float(output_limit))
        self.integral_limit = (
            None if integral_limit is None else abs(float(integral_limit))
        )
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error, measured_rate, dt):
        p_term = self.kp * error
        d_term = -self.kd * measured_rate
        candidate_integral = self.integral + self.ki * error * dt

        if self.integral_limit is not None:
            candidate_integral = clamp(candidate_integral, self.integral_limit)

        unconstrained = p_term + candidate_integral + d_term
        output = clamp(unconstrained, self.output_limit)

        # If saturated and the error would drive the output further into
        # saturation, discard this cycle's integration.
        if abs(unconstrained) > self.output_limit and error * unconstrained > 0.0:
            candidate_integral = self.integral
            unconstrained = p_term + candidate_integral + d_term
            output = clamp(unconstrained, self.output_limit)

        self.integral = candidate_integral
        return output


class HeavySequencedTransitControl(Node):
    def __init__(self):
        super().__init__("heavy_sequenced_transit_control")

        # ROS interfaces
        self.declare_parameter('odom_topic', '/bluerov2_heavy/odometry/filtered')
        self.declare_parameter('pilot_input_topic', '/bluerov2_heavy/auto_transit/pilot_input')
        self.declare_parameter('reset_topic', '/bluerov2_heavy/auto_transit/reset')
        self.declare_parameter('wrench_topic', '/bluerov2_heavy/wrench')
        self.declare_parameter('state_topic', '/bluerov2_heavy/auto_transit/state')

        # Timing and sequencing tolerances
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('xy_tolerance', 0.75)
        self.declare_parameter('heave_tolerance', 0.02)
        self.declare_parameter('yaw_tolerance', math.radians(0.01))
        self.declare_parameter('roll_tolerance', math.radians(1.0))
        self.declare_parameter('pitch_tolerance', math.radians(1.0))

        #Wrench saturation limits
        self.declare_parameter('surge_limit', 40.0)
        self.declare_parameter('sway_limit', 40.0)
        self.declare_parameter('heave_limit', 40.0)
        self.declare_parameter('roll_limit', 20.0)
        self.declare_parameter('pitch_limit', 20.0)
        self.declare_parameter('yaw_limit', 20.0)

        # BlueROV2 Heavy gains supplied by the user.
        gain_defaults = {
            'surge_kp': 27.4217864991176,
            'surge_ki': 0.0,
            'surge_kd': 6.06583586546836,
            'sway_kp': 45.0271671594788,
            'sway_ki': 0.0,
            'sway_kd': 7.00598822741625,
            'heave_kp': 154.528346711312,
            'heave_ki': 159.195792680064,
            'heave_kd': 33.3457824384944,
            'roll_kp': 23.3551541950938,
            'roll_ki': 17.3175804133597,
            'roll_kd': 7.87441453217977,
            'pitch_kp': 27.732451499144,
            'pitch_ki': 17.1569223510377,
            'pitch_kd': 11.2066845443694,
            'yaw_kp': 18.8547805356319,
            'yaw_ki': 22.4343207253557,
            'yaw_kd': 3.96159475249835,
        }

        for name, value in gain_defaults.items():
            self.declare_parameter(name, value)

        # Integral-state limits explicitly supplied for heave and yaw.
        self.declare_parameter('heave_integral_limit', 5.0)
        self.declare_parameter('yaw_integral_limit', 5.0)

        self.declare_parameter('attitude_input_limit_deg', 60.0)
        self.attitude_input_limit = math.radians(
            float(self.get_parameter('attitude_input_limit_deg').value)
        )

        self.odom_topic = self.get_parameter('odom_topic').value
        self.pilot_input_topic = self.get_parameter('pilot_input_topic').value
        self.reset_topic = self.get_parameter('reset_topic').value
        self.wrench_topic = self.get_parameter('wrench_topic').value
        self.state_topic = self.get_parameter('state_topic').value

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        self.xy_tolerance = float(self.get_parameter('xy_tolerance').value)
        self.heave_tolerance = float(self.get_parameter('heave_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.roll_tolerance = float(self.get_parameter('roll_tolerance').value)
        self.pitch_tolerance = float(self.get_parameter('pitch_tolerance').value)

        self.surge_limit = float(self.get_parameter('surge_limit').value)
        self.sway_limit = float(self.get_parameter('sway_limit').value)
        self.heave_limit = float(self.get_parameter('heave_limit').value)
        self.roll_limit = float(self.get_parameter('roll_limit').value)
        self.pitch_limit = float(self.get_parameter('pitch_limit').value)
        self.yaw_limit = float(self.get_parameter('yaw_limit').value)

        def gain(name):
            return float(self.get_parameter(name).value)

        self.surge_pid = PID(gain('surge_kp'), gain('surge_ki'), gain('surge_kd'), self.surge_limit)
        self.sway_pid = PID(gain('sway_kp'), gain('sway_ki'), gain('sway_kd'), self.sway_limit)
        self.heave_pid = PID(
            gain('heave_kp'), gain('heave_ki'), gain('heave_kd'),
            self.heave_limit, self.get_parameter('heave_integral_limit').value
        )
        self.roll_pid = PID(gain('roll_kp'), gain('roll_ki'), gain('roll_kd'), self.roll_limit)
        self.pitch_pid = PID(gain('pitch_kp'), gain('pitch_ki'), gain('pitch_kd'), self.pitch_limit)
        self.yaw_pid = PID(
            gain('yaw_kp'), gain('yaw_ki'), gain('yaw_kd'),
            self.yaw_limit, self.get_parameter('yaw_integral_limit').value
        )

        # Latest measurements and pilot command.
        self.odom = None
        self.last_odom_time = None
        self.initial_pose = None
        self.hold_heave = None
        self.hold_yaw = None
        self.pilot = SixDofSetpoint()
        self.stage = 'WAITING_FOR_ODOM'
        self.last_time = self.get_clock().now()
        self.last_stale_warning = None

        self.wrench_pub = self.create_publisher(Wrench, self.wrench_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.pilot_sub = self.create_subscription(SixDofSetpoint, self.pilot_input_topic, self.pilot_callback, 10)
        self.reset_sub = self.create_subscription(Bool, self.reset_topic, self.reset_callback, 10)

        #Visualization topics
        ns = '/bluerov2_heavy/auto_transit'
        self.position_pub = self.create_publisher(Vector3, f'{ns}/position', 10)  # x, y, depth
        self.position_target_pub = self.create_publisher(Vector3, f'{ns}/position_target', 10)  # controller
        self.position_error_pub = self.create_publisher(Vector3, f'{ns}/position_error', 10)
        self.pilot_position_target_pub = self.create_publisher(Vector3, f'{ns}/pilot_position_target', 10)
        self.pilot_position_error_pub = self.create_publisher(Vector3, f'{ns}/pilot_position_error', 10)
        self.attitude_pub = self.create_publisher(Vector3, f'{ns}/attitude_deg', 10)  # roll, pitch, yaw
        self.attitude_target_pub = self.create_publisher(Vector3, f'{ns}/attitude_target_deg', 10)  # controller
        self.attitude_error_pub = self.create_publisher(Vector3, f'{ns}/attitude_error_deg', 10)
        self.pilot_attitude_target_pub = self.create_publisher(Vector3, f'{ns}/pilot_attitude_target_deg', 10)
        self.pilot_attitude_error_pub = self.create_publisher(Vector3, f'{ns}/pilot_attitude_error_deg', 10)
        self.attitude_limit_pub = self.create_publisher(Vector3, f'{ns}/attitude_limit_deg', 10)
        self.attitude_margin_pub = self.create_publisher(Vector3, f'{ns}/attitude_limit_margin_deg', 10)
        self.velocity_pub = self.create_publisher(Vector3, f'{ns}/velocity', 10)  # body vx, vy, world vz
        self.horizontal_error_pub = self.create_publisher(Float64, f'{ns}/horizontal_error', 10)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)
        self.get_logger().info('BlueROV2 Heavy sequenced transit controller started')
        self.get_logger().info(f'Pilot input: {self.pilot_input_topic} (geometry_msgs/Twist)')

    def reset_all_pids(self):
        for pid in (
            self.surge_pid, self.sway_pid, self.heave_pid,
            self.roll_pid, self.pitch_pid, self.yaw_pid
        ):
            pid.reset()

    def publish_stage(self, stage):
        if stage == self.stage:
            return
        self.stage = stage
        msg = String()
        msg.data = stage
        self.state_pub.publish(msg)
        self.get_logger().info(f'Auto-transit stage: {stage}')
        self.reset_all_pids()

    def odom_callback(self, msg):
        self.odom = msg
        self.last_odom_time = self.get_clock().now()
        if self.initial_pose is None:
            s = self.read_state()
            self.initial_pose = {
                'x': s['x'], 'y': s['y'], 'z': s['z'],
                'yaw': s['yaw'],
            }
            self.hold_heave = s['z']
            self.hold_yaw = s['yaw']
            self.get_logger().info(
                'Initial pose captured: '
                f"X={s['x']:.3f}, Y={s['y']:.3f}, Z={s['z']:.3f}, "
                f"Yaw={math.degrees(s['yaw']):.2f} deg"
            )
            self.publish_stage('HORIZONTAL')

    def pilot_callback(self, msg):
        values = (
            msg.surge, msg.sway, msg.heave, msg.roll,
            msg.pitch, msg.yaw
        )
        if not all(math.isfinite(v) for v in values):
            self.get_logger().error("Pilot command rejected: all six input must be finite")
            return

        if abs(msg.roll) > self.attitude_input_limit or abs(msg.pitch) > self.attitude_input_limit:
            self.get_logger().error(
                f"Pilot command rejected: roll={math.degrees(msg.roll):.1f} deg, "
                f"pitch={math.degrees(msg.pitch):.1f} deg exceeds "
                f"+/-{math.degrees(self.attitude_input_limit):.0f} deg"
            )
            return
    
        self.pilot = msg

    def reset_callback(self, msg):
        if not msg.data:
            return
        if self.odom is None or self.last_odom_time is None:
            self.get_logger().error('Cannot reset initial pose: odometry is unavailable')
            return
        age = (self.get_clock().now() - self.last_odom_time).nanoseconds * 1e-9
        if age > self.odom_timeout:
            self.get_logger().error('Cannot reset initial pose: odometry is stale')
            return
        s = self.read_state()
        self.initial_pose = {'x': s['x'], 'y': s['y'], 'z': s['z'], 'yaw': s['yaw']}
        self.hold_heave = s['z']
        self.hold_yaw = s['yaw']
        self.pilot = SixDofSetpoint()
        # Preserve current attitude on rebase; translational offsets become zero.
        self.pilot.roll = s['roll']
        self.pilot.pitch = s['pitch']
        self.pilot.yaw = s['yaw']
        self.reset_all_pids()
        self.publish_stage('HORIZONTAL')
        self.get_logger().warn('Initial pose recaptured and pilot offsets reset to zero')

    def read_state(self):
        pose = self.odom.pose.pose
        twist = self.odom.twist.twist
        q = pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # Rotation matrix mapping body-frame vectors into the odometry world frame.
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        roll = math.atan2(r21, r22)
        pitch = -math.asin(clamp(r20, 1.0))
        yaw = math.atan2(r10, r00)

        #Odometry.twist.twist is expressed in child_frame_id/base_link.
        vx = twist.linear.x
        vy = twist.linear.y
        vz = twist.linear.z
        p = twist.angular.x
        q_rate = twist.angular.y
        r = twist.angular.z

        cos_pitch = math.cos(pitch)
        if abs(cos_pitch) < 1e-3:
            cos_pitch = math.copysign(1e-3, cos_pitch if cos_pitch != 0.0 else 1.0)
        sin_roll = math.sin(roll)
        cos_roll = math.cos(roll)
        tan_pitch = math.tan(pitch)

        roll_rate = p + sin_roll * tan_pitch * q_rate + cos_roll * tan_pitch * r
        pitch_rate = cos_roll * q_rate - sin_roll * r
        yaw_rate = (sin_roll * q_rate + cos_roll * r) / cos_pitch
        world_vz = r20 * vx + r21 * vy + r22 * vz

        return {
            'R': ((r00, r01, r02), (r10, r11, r12), (r20, r21, r22)),
            'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
            'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'vx': vx, 'vy': vy, 'vz': vz, 'world_vz': world_vz,
            'roll_rate': roll_rate, 'pitch_rate': pitch_rate, 'yaw_rate': yaw_rate,
        }

    def world_xy_error_to_body(self, s, target_x, target_y):
        ex_world = target_x - s['x']
        ey_world = target_y - s['y']
        r = s['R']
        # R transpose maps world-frame position error into body coordinates.
        ex_body = r[0][0] * ex_world + r[1][0] * ey_world
        ey_body = r[0][1] * ex_world + r[1][1] * ey_world
        return ex_body, ey_body

    def make_wrench(self, s, dt, setpoints):
        target_x, target_y, target_z, target_roll, target_pitch, target_yaw = setpoints
        ex_body, ey_body = self.world_xy_error_to_body(s, target_x, target_y)

        wrench = Wrench()
        wrench.force.x = self.surge_pid.update(ex_body, s['vx'], dt)
        wrench.force.y = self.sway_pid.update(ey_body, s['vy'], dt)
        wrench.force.z = self.heave_pid.update(target_z - s['z'], s['world_vz'], dt)
        wrench.torque.x = self.roll_pid.update(
            wrap_angle(target_roll - s['roll']), s['roll_rate'], dt
        )
        wrench.torque.y = self.pitch_pid.update(
            wrap_angle(target_pitch - s['pitch']), s['pitch_rate'], dt
        )
        wrench.torque.z = self.yaw_pid.update(
            wrap_angle(target_yaw - s['yaw']), s['yaw_rate'], dt
        )
        return wrench

    def publish_debug(self, s, pilot_targets, controller_targets):
        # State, targets and errors for plotting. Errors are target - current, depth positive down.
        px, py, pz, proll, ppitch, pyaw = pilot_targets
        cx, cy, cz, croll, cpitch, cyaw = controller_targets

        depth = -s['z']

        self.position_pub.publish(Vector3(x=s['x'], y=s['y'], z=depth))

        # What the PIDs are tracking this cycle
        self.position_target_pub.publish(Vector3(x=cx, y=cy, z=-cz))
        self.position_error_pub.publish(Vector3(
            x=cx - s['x'], y=cy - s['y'], z=(-cz) - depth
        ))

        # What the pilot asked for
        self.pilot_position_target_pub.publish(Vector3(x=px, y=py, z=-pz))
        self.pilot_position_error_pub.publish(Vector3(
            x=px - s['x'], y=py - s['y'], z=(-pz) - depth
        ))

        self.attitude_pub.publish(Vector3(
            x=math.degrees(s['roll']), y=math.degrees(s['pitch']), z=math.degrees(s['yaw'])
        ))

        self.attitude_target_pub.publish(Vector3(
            x=math.degrees(croll), y=math.degrees(cpitch), z=math.degrees(cyaw)
        ))
        self.attitude_error_pub.publish(Vector3(
            x=math.degrees(wrap_angle(croll - s['roll'])),
            y=math.degrees(wrap_angle(cpitch - s['pitch'])),
            z=math.degrees(wrap_angle(cyaw - s['yaw'])),
        ))

        self.pilot_attitude_target_pub.publish(Vector3(
            x=math.degrees(proll), y=math.degrees(ppitch), z=math.degrees(pyaw)
        ))
        self.pilot_attitude_error_pub.publish(Vector3(
            x=math.degrees(wrap_angle(proll - s['roll'])),
            y=math.degrees(wrap_angle(ppitch - s['pitch'])),
            z=math.degrees(wrap_angle(pyaw - s['yaw'])),
        ))

        # Pilot input limit (positive side; the negative side is the mirror)
        self.attitude_limit_pub.publish(Vector3(
            x=math.degrees(self.attitude_input_limit),
            y=math.degrees(self.attitude_input_limit),
            z=0.0,
        ))
        self.attitude_margin_pub.publish(Vector3(
            x=math.degrees(self.attitude_input_limit - abs(s['roll'])),
            y=math.degrees(self.attitude_input_limit - abs(s['pitch'])),
            z=0.0,
        ))

        self.velocity_pub.publish(Vector3(x=s['vx'], y=s['vy'], z=s['world_vz']))
        self.horizontal_error_pub.publish(Float64(
            data=math.hypot(px - s['x'], py - s['y'])
        ))

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.1:
            dt = 1.0 / self.control_rate

        if self.odom is None or self.initial_pose is None or self.last_odom_time is None:
            return

        odom_age = (now - self.last_odom_time).nanoseconds * 1e-9
        if odom_age > self.odom_timeout:
            # Feedback is no longer trustworthy. Zero is published as a watchdog
            self.wrench_pub.publish(Wrench())
            if self.last_stale_warning is None or (now - self.last_stale_warning).nanoseconds * 1e-9 > 2.0:
                self.get_logger().error('Odometry stale: publishing zero wrench; check the vehicle failsafe')
                self.last_stale_warning = now
            return

        s = self.read_state()
        initial = self.initial_pose

        #Relative translational inputs are converted to world-frame position targets.
        target_x = initial['x'] + self.pilot.surge
        target_y = initial['y'] + self.pilot.sway
        target_z = initial['z'] + self.pilot.heave
        # Attitude inputs are absolute world-frame Euler-angle targets.
        target_roll = self.pilot.roll                                              
        target_pitch = self.pilot.pitch
        target_yaw = self.pilot.yaw

        dx = target_x - s['x']
        dy = target_y - s['y']
        horizontal_error = math.hypot(dx, dy)
        heave_error = target_z - s['z']
        yaw_error = wrap_angle(target_yaw - s['yaw'])
        roll_error = wrap_angle(target_roll - s['roll'])
        pitch_error = wrap_angle(target_pitch - s['pitch'])

        # Priority sequencing: horizontal -> heave -> yaw -> simultaneous roll/pitch.
        if horizontal_error > self.xy_tolerance:
            stage = 'HORIZONTAL'
            controller_z = self.hold_heave
            controller_roll = 0.0
            controller_pitch = 0.0
            controller_yaw = self.hold_yaw

        elif abs(heave_error) > self.heave_tolerance:
            stage = 'HEAVE'
            controller_z = target_z
            controller_roll = 0.0
            controller_pitch = 0.0
            controller_yaw = self.hold_yaw
            # Keep the most recently achieved heave as the hold reference if
            # horizontal correction becomes necessary again.
            self.hold_heave = s['z']

        elif abs(yaw_error) > self.yaw_tolerance:
            stage = 'YAW'
            controller_z = target_z
            controller_roll = 0.0
            controller_pitch = 0.0
            controller_yaw = target_yaw
            # Preserve the current yaw if an earlier sequence stage must resume.
            self.hold_yaw = s['yaw']

        else:
            if (abs(roll_error) > self.roll_tolerance or abs(pitch_error) > self.pitch_tolerance):
                stage = 'ROLL_PITCH'
            else:
                stage = 'HOLD'
            controller_z = target_z
            controller_roll = target_roll
            controller_pitch = target_pitch
            controller_yaw = target_yaw

        self.publish_stage(stage)
        setpoints = (target_x, target_y, controller_z, controller_roll,
                     controller_pitch, controller_yaw)
        wrench = self.make_wrench(s, dt, setpoints)
        self.wrench_pub.publish(wrench)
        self.publish_debug(s, (target_x, target_y, target_z, target_roll, target_pitch, target_yaw), 
                           setpoints)


def main(args=None):
    rclpy.init(args=args)
    node = HeavySequencedTransitControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.wrench_pub.publish(Wrench())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()