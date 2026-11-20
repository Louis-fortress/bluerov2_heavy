#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Point

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


class AutoTransitControl(Node):
    def __init__(self):
        super().__init__("auto_transit_control")

        #Topics
        self.declare_parameter("odom_topic", "/bluerov2_heavy/odometry/filtered")
        self.declare_parameter("wrench_topic", "/bluerov2_heavy/wrench")
        self.declare_parameter("target_topic", "/bluerov2_heavy/auto_transit/target")
        self.declare_parameter("cancel_topic", "/bluerov2_heavy/auto_transit/cancel")
        self.declare_parameter("state_topic", "/bluerov2_heavy/auto_transit/state")

        #Tolerance
        self.declare_parameter("xy_tolerance", 0.10)
        self.declare_parameter("z_tolerance", 0.10)
        self.declare_parameter("heading_tolerance", math.radians(3.0))

        #Max wrench outputs on each axis
        self.declare_parameter('surge_limit', 40.0)
        self.declare_parameter('sway_limit', 40.0)
        self.declare_parameter('heave_limit', 40.0)
        self.declare_parameter('yaw_limit', 20.0)

        #Loop handling
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('odom_timeout', 0.5)

        self.state_topic = self.get_parameter('state_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.cancel_topic = self.get_parameter('cancel_topic').value

        self.control_rate = self.get_parameter('control_rate').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        self.xy_tolerance = self.get_parameter('xy_tolerance').value
        self.z_tolerance = self.get_parameter('z_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value

        self.surge_limit = self.get_parameter('surge_limit').value
        self.sway_limit = self.get_parameter('sway_limit').value
        self.heave_limit = self.get_parameter('heave_limit').value
        self.yaw_limit = self.get_parameter('yaw_limit').value

        # self.depth_pid = self.make_pid('depth', 11.9403874387169, 4.07948623930136, 3.24177393275929, self.heave_limit)
        self.depth_pid = self.make_pid('depth', 154.528346711312, 159.195792680064, 33.3457824384944, self.heave_limit)
        self.roll_pid = self.make_pid('roll', 16.0, 2.0, 17.0, 20.0)
        self.pitch_pid = self.make_pid('pitch', 26.0, 3.0, 26.0, 20.0)
        self.yaw_pid = self.make_pid('yaw', 15.8, 1.5, 19.0, self.yaw_limit)

        #Position / Velocity Controller
        self.pos_x_pid = self.make_pid('positon_x_pid', 8.0, 0.5, 25.0, self.surge_limit)
        self.pos_y_pid = self.make_pid('positon_y_pid', 8.0, 0.5, 25.0, self.sway_limit)
        self.vel_x_pid = self.make_pid('velocity_x_pid', 30.0, 3.0, 0.0, self.surge_limit)
        self.vel_y_pid = self.make_pid('velocity_y_pid', 30.0, 3.0, 0.0, self.sway_limit)

        #navigation states
        self.odom = None
        self.last_odom_time = None
        self.target = None
        self.state = 'IDLE'
        self.target_yaw = None
        self.transit_depth = None
        self.last_time = self.get_clock().now()

        # ROS infrastructure
        self.wrench_pub = self.create_publisher(
            Wrench,
            self.get_parameter('wrench_topic').value,
            10
        )

        self.state_pub = self.create_publisher(
            String, self.state_topic, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10
        )

        self.target_sub = self.create_subscription(
            Point, self.target_topic, self.target_callback, 10
        )

        self.cancel_sub = self.create_subscription(
            Bool, self.cancel_topic, self.cancel_callback, 10
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate, self.control_loop
        )

        self.get_logger().info(
            'Auto Transit Controller started'
        )

    def make_pid(self, name, kp, ki, kd, limit):
        parameters = {
            f'{name}_kp': kp,
            f'{name}_ki': ki,
            f'{name}_kd': kd,
            f'{name}_limit': limit
        }

        for param_name, default_value in parameters.items():
            if not self.has_parameter(param_name):
                self.declare_parameter(param_name, default_value)

        return PID(
            self.get_parameter(f'{name}_kp').value,
            self.get_parameter(f'{name}_ki').value,
            self.get_parameter(f'{name}_kd').value,
            self.get_parameter(f'{name}_limit').value
            )

    def reset_all_pids(self):
        for pid in (self.depth_pid, self.yaw_pid, self.pos_x_pid, self.pos_y_pid, self.vel_x_pid, self.vel_y_pid):
            pid.reset()
    
    def odom_callback(self, msg):
        self.odom = msg
        self.last_odom_time = self.get_clock().now()

    def target_callback(self, msg):
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            self.get_logger().error('Target rejected: coordinates must be finite')
            return

        self.target = [msg.x, msg.y, msg.z]

        self.target_yaw = None
        self.transit_depth = None
        self.reset_all_pids()
        self.set_state('ALIGN')
        self.get_logger().info(
            f'New target: X={msg.x:.2f}, '
            f'Y={msg.y:.2f}, Z={msg.z:.2f}'
        )

    def cancel_callback(self, msg):
        if not msg.data:
            return

        if self.odom is None or self.last_odom_time is None:
            self.get_logger().error('Cannot cancel into HOLD: no odometry available')
            return

        now = self.get_clock().now()
        odom_age = (now - self.last_odom_time).nanoseconds * 1e-9
        if odom_age > self.odom_timeout:
            self.get_logger().error('Cannot capture HOLD position: odometry is stale')
            return

        #Replace navigation target with current position
        s = self.read_state()
        self.target = [
            s['x'],
            s['y'],
            s['z']
        ]
        self.target_yaw = s['yaw']
        self.transit_depth = s['z']
        self.reset_all_pids()
        self.set_state('HOLD')
        self.get_logger().warn(
        f'Auto transit cancelled. Holding current position: '
        f'X={s["x"]:.2f}, '
        f'Y={s["y"]:.2f}, '
        f'Z={s["z"]:.2f}, '
        f'Yaw={math.degrees(s["yaw"]):.2f} deg'
        )

    def set_state(self, new_state):
        if self.state == new_state:
            return

        self.state = new_state
        self.get_logger().info(f'Auto transit state: {self.state}')

        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
        self.reset_all_pids()

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

    #World position error -> body position error
    def world_to_body_error(self, s):
        ex_w = self.target[0] - s['x']
        ey_w = self.target[1] - s['y']

        R = s['R']
        ex_b = (
            R[0][0] * ex_w + R[1][0] * ey_w
        )
        ey_b = (
            R[0][1] * ex_w + R[1][1] * ey_w
        )

        return ex_b, ey_b

    #Calculating desired heading
    def calculate_target_heading(self, s):
        dx = self.target[0] - s['x']
        dy = self.target[1] - s['y']

        return math.atan2(dy, dx)

    #Publish zero wrench
    def publish_zero_wrench(self):
        self.wrench_pub.publish(Wrench())

    #main controller
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.1:
            dt = 1.0/self.control_rate

        if self.odom is None:
            self.publish_zero_wrench()
            return

        odom_age = (now - self.last_odom_time).nanoseconds * 1e-9
        if odom_age > self.odom_timeout:
            self.publish_zero_wrench()
            return 

        s = self.read_state()
        wrench = Wrench()

        #IDLE
        if self.state == 'IDLE':
            self.target = [s['x'], s['y'], s['z']]
            self.target_yaw = s['yaw']
            self.transit_depth = s['z']
            self.set_state('HOLD')
            self.get_logger().info(
                f'Holding current pose: X={s["x"]:.2f}, Y={s["y"]:.2f}, '
                f'Z={s["z"]:.2f}, Yaw={math.degrees(s["yaw"]):.2f} deg'
            )

        if self.target is None:
            self.publish_zero_wrench()
            return

        if self.target_yaw is None:
            dx = self.target[0] - s['x']
            dy = self.target[1] - s['y']

            horizontal_distance = math.hypot(dx, dy)
            if horizontal_distance <= self.xy_tolerance:
                self.target_yaw = s['yaw']
            else:
                self.target_yaw = self.calculate_target_heading(s)
            self.transit_depth = s['z']
            if horizontal_distance <= self.xy_tolerance:
                self.set_state('DEPTH')

            self.get_logger().info(
                f'Desired heading: '
                f'{math.degrees(self.target_yaw):.2f} degrees'
            )

        #Position errors
        dx = self.target[0] - s['x']
        dy = self.target[1] - s['y']
        dz = self.target[2] - s['z']
        horizontal_distance = math.hypot(dx, dy)

        ex_b, ey_b = self.world_to_body_error(s)
        heading_error = wrap_angle(self.target_yaw - s['yaw'])
        #Align
        if self.state == 'ALIGN':
            #Rotate towards the horizontal target bearing
            wrench.torque.z = self.yaw_pid.update(heading_error, s['yaw_rate'], dt)
            #Break translational velocity while rotating
            wrench.force.x = self.vel_x_pid.update(-s['vx'], 0.0, dt)
            wrench.force.y = self.vel_y_pid.update(-s['vy'], 0.0, dt)
            #Maintain the initial depth
            wrench.force.z = self.depth_pid.update(self.transit_depth - s['z'], s['world_vz'], dt)

            if abs(heading_error) <= self.heading_tolerance:
                self.set_state('TRANSLATE')

        elif self.state == 'TRANSLATE':
            #Maintain heading
            wrench.torque.z = self.yaw_pid.update(heading_error, s['yaw_rate'], dt)
            #Move towards the XY target
            wrench.force.x = self.pos_x_pid.update(ex_b, s['vx'], dt)
            wrench.force.y = self.pos_y_pid.update(ey_b, s['vy'], dt)
            #Maintain initial depth
            wrench.force.z = self.depth_pid.update(
                self.transit_depth - s['z'],
                s['world_vz'],
                dt
            )

            if horizontal_distance <= self.xy_tolerance:
                self.set_state('DEPTH')
                self.get_logger().info(
                    'Horizontal target reached.'
                    'Starting vertical navigation'
                )

        elif self.state == 'DEPTH':
            #Maintain horizontal position
            wrench.force.x = self.pos_x_pid.update(
                ex_b, s['vx'], dt
            )
            wrench.force.y = self.pos_y_pid.update(ey_b, s['vy'], dt)
            #Maintain desired heading
            wrench.torque.z = self.yaw_pid.update(heading_error, s['yaw_rate'], dt)
            #Move vertically towards actual target
            wrench.force.z = self.depth_pid.update(dz, s['world_vz'], dt)
            if abs(dz) <= self.z_tolerance:
                self.set_state('HOLD')
                self.get_logger().info('Target reached holding position.')

        elif self.state == 'HOLD':
            #Hold all 4 degrees of freedom
            wrench.force.x = self.pos_x_pid.update(ex_b, s['vx'], dt)
            wrench.force.y = self.pos_y_pid.update(ey_b, s['vy'], dt)
            wrench.force.z = self.depth_pid.update(dz, s['world_vz'], dt)
            wrench.torque.z = self.yaw_pid.update(heading_error, s['yaw_rate'], dt)

        self.wrench_pub.publish(wrench)

def main(args=None):
    rclpy.init(args=args)
    node = AutoTransitControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
