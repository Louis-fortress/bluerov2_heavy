#!/usr/bin/env python3
import math
import rclpy
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

def wrap_angle(a):
    """Wrap an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class StabilizedControl(Node):
    def __init__(self):
        super().__init__("stabilized_control")

        #Ros infrastructure
        self.cmd_wrench_pub = self.create_publisher(Wrench,
                                                       '/bluerov2/wrench', 10)

        # self.odom_sub = self.create_subscription(Odometry, '/bluerov2/odom', self.odom_callback,
        #                                          10)

        #EKF x, y position
        self.pos_sub = self.create_subscription(Odometry, '/bluerov2/odometry/filtered',
                                                self.position_callback, 10)

        #Depth
        self.depth_sub = self.create_subscription(PoseWithCovarianceStamped, '/bluerov2/depth_pose',
                                                  self.depth_callback, 10)

        #DVL
        self.dvl_sub = self.create_subscription(TwistWithCovarianceStamped, '/bluerov2/dvl_twist_cov',
                                                self.dvl_callback, 10)

        #IMU
        self.imu_sub = self.create_subscription(Imu, '/bluerov2/imu', self.imu_callback, 10)

        self.cmd_wrench_joy_sub = self.create_subscription(Wrench, '/bluerov2/cmd_wrench_joy',
                                                           self.wrench_joy_callback, 10)

        self.zupt_pub = self.create_publisher(TwistWithCovarianceStamped, '/bluerov2/zupt_vel', 10)

        self.timer = self.create_timer(0.01, self.control_loop)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.station_keep_sub = self.create_subscription(Bool, '/bluerov2/station_keep_enable',
                                            self.station_keep_callback, latched_qos)

        self.declare_parameter('depth_Kp', 100.0)
        self.declare_parameter('depth_Ki', 15.0)
        self.declare_parameter('depth_Kd', 0.0)
        self.declare_parameter('depth_force_limit', 50.0)
        self.declare_parameter('depth_deadband', 0.0)
        self.declare_parameter('integral_limit', 2.0)
        self.declare_parameter('heave_threshold', 0.05)

        self.declare_parameter('sk_use_velocity', True)
        self.declare_parameter('vel_Kp', 14.980)
        self.declare_parameter('vel_Ki', 1.498)
        self.declare_parameter('vel_integral_limit', 3.0)
        self.declare_parameter('vel_force_limit', 40.0)

        # Acceleration feedback: filtered body-frame acceleration estimated
        # from successive EKF velocity measurements (m/s^2).
        # The term -accel_gain * measured_acceleration adds damping to the
        # existing position/velocity cascade. Start conservatively.
        self.declare_parameter('accel_gain', 1.0)       # N per (m/s^2)
        self.declare_parameter('accel_filter_tau', 0.15) # seconds
        self.declare_parameter('accel_limit', 1.5)       # m/s^2
        self.declare_parameter('dvl_timeout', 0.5)
        self.declare_parameter('dvl_offset_x', 0.0)
        self.declare_parameter('dvl_offset_y', 0.0)
        self.declare_parameter('dvl_offset_z', -0.1)

        # Attitude parameters 
        self.declare_parameter('rp_Kp', 8.1157155128443)
        self.declare_parameter('rp_Ki', 17.6870803091602)
        self.declare_parameter('rp_Kd', 0.909928347386951)
        self.declare_parameter('yaw_Kp', 7.47734001156017)
        self.declare_parameter('yaw_Ki', 4.87669495949845)
        self.declare_parameter('yaw_Kd', 2.21182867790919)
        self.declare_parameter('rp_torque_limit', 8.0)
        self.declare_parameter('yaw_torque_limit', 8.0)
        self.declare_parameter('att_integral_limit', 0.5)
        self.declare_parameter('yaw_threshold', 0.05)
        self.declare_parameter('roll_threshold', 0.05)
        self.declare_parameter('roll_return_time', 1.0)

        #Station Keeping parameters
        self.declare_parameter('settle_speed', 0.05)
        self.declare_parameter('pos_Kp', 61.12263231561)
        self.declare_parameter('pos_Ki', 0.0)
        self.declare_parameter('pos_Kd', 14.9808045779026)
        self.declare_parameter('pos_force_limit', 40.0)
        self.declare_parameter('pos_integral_limit', 5.0)
        self.declare_parameter('translate_threshold', 0.05)

        self.declare_parameter('pos_to_vel_Kp', 0.25)      # (m/s) per m of error
        self.declare_parameter('pos_to_vel_Ki', 0.025)      # rejects steady current
        self.declare_parameter('sk_vel_max', 1.0)         # m/s cap on the velocity target
        self.declare_parameter('sk_pos_integral_limit', 1.0)

        # self.declare_parameter('depth_Kp', 33.7932352213167)
        # self.declare_parameter('depth_Ki', 10.144528030708)
        # self.declare_parameter('depth_Kd', 4.51105305064316)
        # self.declare_parameter('depth_force_limit', 50.0)
        # self.declare_parameter('depth_deadband', 0.0)
        # self.declare_parameter('integral_limit', 5.0)
        # self.declare_parameter('heave_threshold', 0.05)

        # self.declare_parameter('sk_use_velocity', True)
        # self.declare_parameter('vel_surge_Kp', 6.56012931550416)
        # self.declare_parameter('vel_surge_Ki', 2.49552167590994)
        # self.declare_parameter('vel_surge_Kd', 0.338763825263534)
        # self.declare_parameter('vel_sway_Kp', 1.65515200771336)
        # self.declare_parameter('vel_sway_Ki', 0.214845884204208)
        # self.declare_parameter('vel_sway_Kd', 0.861077785437516)
        # self.declare_parameter('vel_Kp', 14.980)
        # self.declare_parameter('vel_Ki', 1.498)
        # self.declare_parameter('vel_integral_limit', 1.0)
        # self.declare_parameter('vel_force_limit', 40.0)

        # # Acceleration feedback: filtered body-frame acceleration estimated
        # # from successive EKF velocity measurements (m/s^2).
        # # The term -accel_gain * measured_acceleration adds damping to the
        # # existing position/velocity cascade. Start conservatively.
        # self.declare_parameter('accel_gain', 0.715956910627207)       # N per (m/s^2)
        # self.declare_parameter('accel_filter_tau', 0.15) # seconds
        # self.declare_parameter('accel_limit', 1.5)       # m/s^2
        # self.declare_parameter('dvl_timeout', 0.5)
        # self.declare_parameter('dvl_offset_x', 0.0)
        # self.declare_parameter('dvl_offset_y', 0.0)
        # self.declare_parameter('dvl_offset_z', -0.1)

        # # Attitude parameters 
        # self.declare_parameter('rp_Kp', 35.5308948660839)
        # self.declare_parameter('rp_Ki', 59.0028243043992)
        # self.declare_parameter('rp_Kd', 5.3490850009866)
        # self.declare_parameter('yaw_Kp', 18.8547805356319)
        # self.declare_parameter('yaw_Ki', 22.4343207253557)
        # self.declare_parameter('yaw_Kd', 3.96159475249835)
        # self.declare_parameter('rp_torque_limit', 8.0)
        # self.declare_parameter('yaw_torque_limit', 8.0)
        # self.declare_parameter('att_integral_limit', 0.5)
        # self.declare_parameter('yaw_threshold', 0.05)
        # self.declare_parameter('roll_threshold', 0.05)
        # self.declare_parameter('roll_return_time', 1.0)

        # #Station Keeping parameters
        # self.declare_parameter('settle_speed', 0.05)
        # self.declare_parameter('pos_Kp', 61.12263231561)
        # self.declare_parameter('pos_Ki', 0.0)
        # self.declare_parameter('pos_Kd', 14.9808045779026)
        # self.declare_parameter('pos_force_limit', 40.0)
        # self.declare_parameter('pos_integral_limit', 5.0)
        # self.declare_parameter('translate_threshold', 0.05)

        # self.declare_parameter('pos_to_vel_surge_Kp', 8.07742976452443)      # (m/s) per m of error
        # self.declare_parameter('pos_to_vel_surge_Ki', 1.58225765817822)      # rejects steady current
        # self.declare_parameter('pos_to_vel_surge_Kd', 1.23511304043015)
        # self.declare_parameter('pos_to_vel_sway_Kp', 10.7121976349455)
        # self.declare_parameter('pos_to_vel_sway_Ki', 1.53274632582785)
        # self.declare_parameter('pos_to_vel_sway_Kd', 0.00535571563089068)
        # self.declare_parameter('sk_vel_max', 1.5)         # m/s cap on the velocity target
        # self.declare_parameter('sk_pos_integral_limit', 5.0)

        self.Kp = self.get_parameter('depth_Kp').value
        self.Ki = self.get_parameter('depth_Ki').value
        self.Kd = self.get_parameter('depth_Kd').value
        self.depth_force_limit = self.get_parameter('depth_force_limit').value
        self.depth_deadband = self.get_parameter('depth_deadband').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.heave_threshold = self.get_parameter('heave_threshold').value
        self.yaw_threshold = self.get_parameter('yaw_threshold').value
        self.roll_threshold = self.get_parameter('roll_threshold').value
        self.translate_threshold = self.get_parameter('translate_threshold').value
        self.roll_return_time = self.get_parameter('roll_return_time').value

        self.sk_use_velocity = self.get_parameter('sk_use_velocity').value
        self.vel_Kp = self.get_parameter('vel_Kp').value
        self.vel_Ki = self.get_parameter('vel_Ki').value
        # self.vel_surge_Kp = self.get_parameter('vel_surge_Kp').value
        # self.vel_surge_Ki = self.get_parameter('vel_surge_Ki').value
        # self.vel_surge_Kd = self.get_parameter('vel_surge_Kd').value
        # self.vel_sway_Kp = self.get_parameter('vel_sway_Kp').value
        # self.vel_sway_Ki = self.get_parameter('vel_sway_Ki').value
        # self.vel_sway_Kd = self.get_parameter('vel_sway_Kd').value
        self.vel_integral_limit = self.get_parameter('vel_integral_limit').value
        self.vel_force_limit = self.get_parameter('vel_force_limit').value
        self.accel_gain = self.get_parameter('accel_gain').value
        self.accel_filter_tau = self.get_parameter('accel_filter_tau').value
        self.accel_limit = self.get_parameter('accel_limit').value
        self.dvl_timeout = self.get_parameter('dvl_timeout').value
        self.dvl_offset = (self.get_parameter('dvl_offset_x').value,
                           self.get_parameter('dvl_offset_y').value,
                           self.get_parameter('dvl_offset_z').value)

        self.rp_Kp = self.get_parameter('rp_Kp').value
        self.rp_Ki = self.get_parameter('rp_Ki').value
        self.rp_Kd = self.get_parameter('rp_Kd').value
        self.yaw_Kp = self.get_parameter('yaw_Kp').value
        self.yaw_Ki = self.get_parameter('yaw_Ki').value
        self.yaw_Kd = self.get_parameter('yaw_Kd').value
        self.rp_torque_limit = self.get_parameter('rp_torque_limit').value
        self.yaw_torque_limit = self.get_parameter('yaw_torque_limit').value
        self.att_integral_limit = self.get_parameter('att_integral_limit').value

        self.pos_to_vel_Kp = self.get_parameter('pos_to_vel_Kp').value
        self.pos_to_vel_Ki = self.get_parameter('pos_to_vel_Ki').value
        # self.pos_to_vel_surge_Kp = self.get_parameter('pos_to_vel_surge_Kp').value
        # self.pos_to_vel_surge_Ki = self.get_parameter('pos_to_vel_surge_Ki').value
        # self.pos_to_vel_surge_Kd = self.get_parameter('pos_to_vel_surge_Kd').value
        # self.pos_to_vel_sway_Kp = self.get_parameter('pos_to_vel_sway_Kp').value
        # self.pos_to_vel_sway_Ki = self.get_parameter('pos_to_vel_sway_Ki').value
        # self.pos_to_vel_sway_Kd = self.get_parameter('pos_to_vel_sway_Kd').value
        self.sk_vel_max = self.get_parameter('sk_vel_max').value
        self.sk_pos_integral_limit = self.get_parameter('sk_pos_integral_limit').value

        self.pos_Kp = self.get_parameter('pos_Kp').value
        self.pos_Ki = self.get_parameter('pos_Ki').value
        self.pos_Kd = self.get_parameter('pos_Kd').value
        self.pos_force_limit = self.get_parameter('pos_force_limit').value
        self.pos_integral_limit = self.get_parameter('pos_integral_limit').value
        self.settle_speed = self.get_parameter('settle_speed').value
        
        # Depth state
        self.depth_hold_active = False
        self.current_depth = None
        self.target_depth = None
        self.depth_velocity = 0.0
        self.previous_heave = False
        self.last_error = 0.0
        self.previous_derivative = 0.0
        self.integral_error = 0.0
        self.last_time = self.get_clock().now()
        self.last_wrench = Wrench()

        # Attitude state
        self.roll = None
        self.pitch = None
        self.yaw = None
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0
        self.roll_integral = 0.0
        self.yaw_integral = 0.0
        self.yaw_hold_active = False
        self.roll_return_start_time = None
        self.roll_return_start_angle = 0.0
        self.target_yaw = None
        self.previous_yaw_input = False
        self.previous_roll_input = False

        #Station Keeping state
        self.station_keep_enabled = False
        self.pos_x = None
        self.pos_y = None

        # EKF odometry uses header.frame_id=odom and child_frame_id=base_link.
        # Per nav_msgs/Odometry, twist is expressed in child_frame_id, so these
        # velocities are body-frame velocities suitable for the inner loop.
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z_body = 0.0
        self.last_odom_time = None
        self.prev_odom_vel_x = None
        self.prev_odom_vel_y = None
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.q = None
        self.pos_latched = False
        self.target_x = None
        self.target_y = None
        self.pos_integral_x = 0.0
        self.pos_integral_y = 0.0
        self.vel_integral_x = 0.0
        self.vel_integral_y = 0.0
        self.last_dvl_time = None

        self.output_wrench = Wrench()

    def wrench_joy_callback(self, msg: Wrench):
        """Store the latest joystick wrench command."""
        self.last_wrench = msg

    def station_keep_callback(self, msg: Bool):
        """Enable or disable station keeping and reset its position-hold state."""
        if msg.data == self.station_keep_enabled:
            return

        self.station_keep_enabled = msg.data
        self.pos_latched = False
        self.pos_integral_x = 0.0
        self.pos_integral_y = 0.0
        self.vel_integral_x = 0.0
        self.vel_integral_y = 0.0
        self.prev_odom_vel_x = None
        self.prev_odom_vel_y = None
        self.accel_x = 0.0
        self.accel_y = 0.0

        if self.station_keep_enabled:
            # Start in braking mode. Capture the target only after motion settles,
            # so the target is not latched at a point the vehicle is still leaving.
            self.get_logger().info("Station keeping enabled: braking before position latch")
        else:
            self.get_logger().info("Station keeping disabled: returning to joystick control")

        zupt_msg = TwistWithCovarianceStamped()
        zupt_msg.header.frame_id = "/bluerov2/base_link"
        zupt_msg.twist.twist.linear.x = 0.0
        zupt_msg.twist.twist.linear.y = 0.0
        zupt_msg.twist.covariance[0] = 1.0e-9
        zupt_msg.twist.covariance[7] = 1.0e-9
        if self.station_keep_enabled:
            self.zupt_pub.publish(zupt_msg)

    def imu_callback(self, msg: Imu):
        """Update the orientation and body angular rates from the IMU"""
        o = msg.orientation
        self.q = [o.x, o.y, o.z, o.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(self.q)

        self.roll_rate = msg.angular_velocity.x
        self.pitch_rate = msg.angular_velocity.y
        self.yaw_rate = msg.angular_velocity.z

    def position_callback(self, msg: Odometry):
        """Update EKF pose and body-frame velocity.

        Expected frames:
          header.frame_id = bluerov2/odom
          child_frame_id = bluerov2/base_link

        nav_msgs/Odometry specifies twist in child_frame_id, so twist.linear.x/y
        are used directly as body-frame surge/sway velocities. This avoids
        treating the raw DVL frame (bluerov2/dvl) as if it were base_link.
        """
        now = self.get_clock().now()
        new_vel_x = msg.twist.twist.linear.x
        new_vel_y = msg.twist.twist.linear.y

        # Estimate acceleration only when a new EKF velocity sample arrives.
        # nav_msgs/Odometry twist is expressed in base_link. The yaw-rate terms
        # compensate for rotation of the body-frame velocity axes:
        # a_body = d(v_body)/dt + omega x v_body.
        if self.last_odom_time is not None and self.prev_odom_vel_x is not None:
            sample_dt = (now - self.last_odom_time).nanoseconds * 1.0e-9
            if 0.005 <= sample_dt <= 0.2:
                raw_ax = ((new_vel_x - self.prev_odom_vel_x) / sample_dt
                          - self.yaw_rate * new_vel_y)
                raw_ay = ((new_vel_y - self.prev_odom_vel_y) / sample_dt
                          + self.yaw_rate * new_vel_x)

                limit = self.accel_limit
                raw_ax = max(-limit, min(raw_ax, limit))
                raw_ay = max(-limit, min(raw_ay, limit))

                tau = max(0.0, self.accel_filter_tau)
                alpha = 1.0 if tau == 0.0 else sample_dt / (tau + sample_dt)
                self.accel_x += alpha * (raw_ax - self.accel_x)
                self.accel_y += alpha * (raw_ay - self.accel_y)
            else:
                # Avoid differentiating across a long gap or invalid interval.
                self.accel_x = 0.0
                self.accel_y = 0.0

        self.prev_odom_vel_x = new_vel_x
        self.prev_odom_vel_y = new_vel_y
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.vel_x = new_vel_x
        self.vel_y = new_vel_y
        self.vel_z_body = msg.twist.twist.linear.z
        self.last_odom_time = now

        # Convert child-frame (base_link) velocity to world vertical velocity
        # for the depth controller's damping term.
        if self.q is not None:
            q = self.q  # [x, y, z, w]
            vz_world = (2.0 * (q[0] * q[2] - q[3] * q[1]) * self.vel_x
                        + 2.0 * (q[1] * q[2] + q[3] * q[0]) * self.vel_y
                        + (1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1])) * self.vel_z_body)
            self.depth_velocity = -vz_world

    def depth_callback(self, msg: PoseWithCovarianceStamped):
        "Update the depth from the pressure sensor"
        current_z = msg.pose.pose.position.z
        self.current_depth = -current_z

    def dvl_callback(self, msg: TwistWithCovarianceStamped):
        """Track DVL reception; its dvl-frame x/y values are not used directly."""
        self.last_dvl_time = self.get_clock().now()

    # def odom_callback(self, msg: Odometry):
    #     """Update position, depth, orientation, and linear/angular velocities from odometry."""
    #     current_z = msg.pose.pose.position.z
    #     self.current_depth = -current_z
    #     self.pos_x = msg.pose.pose.position.x
    #     self.pos_y = msg.pose.pose.position.y

    #     #Quaternion to roll/pitch/yaw
    #     q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    #     self.roll, self.pitch, self.yaw = euler_from_quaternion(q)

    #     #Body angular rates (p, q, r)
    #     self.roll_rate = msg.twist.twist.angular.x
    #     self.pitch_rate = msg.twist.twist.angular.y
    #     self.yaw_rate = msg.twist.twist.angular.z

    #     #Body linear rates (u, v)
    #     v = msg.twist.twist.linear
    #     self.vel_x = v.x
    #     self.vel_y = v.y

    #     # Depth rate = -(world-frame vz). Rotate body-frame linear velocity into the world z axis
    #     vz_world = (2.0 * (q[0] * q[2] - q[3] * q[1]) * v.x
    #                 + 2.0 * (q[1] * q[2] + q[3] * q[0]) * v.y
    #                 + (1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1])) * v.z)
    #     self.depth_velocity = -vz_world

    def update_depth_controller(self, wrench, dt):
        """Apply manual heave input or capture and maintain the target depth."""
        current_heave = wrench.force.z
        heave_active = (abs(current_heave) > self.heave_threshold)

        if heave_active:
            self.depth_hold_active = False
            self.integral_error = 0.0
            self.previous_heave = True
            return current_heave

        if self.previous_heave or not self.depth_hold_active:
            if self.current_depth is not None:
                self.target_depth = self.current_depth
                self.integral_error = 0.0
                self.last_error = 0.0
                self.previous_derivative = 0.0
                self.depth_hold_active = True
                self.get_logger().info(
                    f"Depth hold enabled at {self.target_depth: .3f} m",
                throttle_duration_sec=0.5)

        self.previous_heave = False

        if self.depth_hold_active:
            return self.depth_pid(dt)

        return 0.0

    def update_yaw_controller(self, wrench, dt):
        """Apply manual yaw input or hold the current heading with the yaw PID."""
        current_yaw = wrench.torque.z
        yaw_active = (abs(current_yaw) > self.yaw_threshold)

        if yaw_active:
            self.yaw_hold_active = False
            self.yaw_integral = 0.0
            self.previous_yaw_input = True
            return current_yaw

        if self.previous_yaw_input or not self.yaw_hold_active:
            if self.yaw is not None:
                self.target_yaw = self.yaw
                self.yaw_integral = 0.0
                self.yaw_hold_active = True
                self.get_logger().info(
                    f"Yaw hold enabled at {self.target_yaw: .3f} rad",
                    throttle_duration_sec=0.5
                )

        self.previous_yaw_input = False
        if self.yaw_hold_active:
            return self.yaw_pid(dt)

        return 0.0

    def update_roll_controller(self, wrench, dt):
        """Apply manual roll input or return the vehicle smoothly to level."""
        roll_cmd = wrench.torque.x
        roll_active = (abs(roll_cmd) > self.roll_threshold)

        if roll_active:
            self.roll_integral = 0.0
            self.previous_roll_input = True
            return roll_cmd
        
        #Stick just released: start the return to level from the current roll
        if self.previous_roll_input and self.roll is not None:
            self.roll_return_start_angle = self.roll
            self.roll_return_start_time = self.get_clock().now()
            self.roll_integral = 0.0

        self.previous_roll_input = False
        return self.roll_pid(dt)

    def update_position_controller(self, wrench, dt):
        """Use joystick translation only when station keeping is disabled.

        When station keeping is enabled, joystick surge/sway commands are ignored
        completely. The controller brakes residual motion, latches one target,
        and then holds that target until station keeping is disabled.
        """
        if not self.station_keep_enabled:
            self.pos_latched = False
            self.pos_integral_x = 0.0
            self.pos_integral_y = 0.0
            self.vel_integral_x = 0.0
            self.vel_integral_y = 0.0
            return wrench.force.x, wrench.force.y

        if not self.sk_use_velocity:
            # Legacy non-cascaded position loop; still never forwards joystick input
            # while station keeping is enabled.
            if self.pos_x is None or self.pos_y is None or self.yaw is None:
                return 0.0, 0.0
            if not self.pos_latched:
                if math.hypot(self.vel_x, self.vel_y) > self.settle_speed:
                    fx = -self.vel_Kp * self.vel_x
                    fy = -self.vel_Kp * self.vel_y
                    return (max(-self.vel_force_limit, min(fx, self.vel_force_limit)),
                            max(-self.vel_force_limit, min(fy, self.vel_force_limit)))
                self.target_x, self.target_y = self.pos_x, self.pos_y
                self.pos_integral_x = self.pos_integral_y = 0.0
                self.pos_latched = True
            return self.position_pid(dt)

        return self.cascaded_hold(dt)

    def cascaded_hold(self, dt):
        """World position PI -> body velocity PI -> body-frame wrench."""
        now = self.get_clock().now()

        # Require fresh EKF odometry and valid attitude. Do not clear the latched
        # target on a brief dropout: otherwise the target can creep after recovery.
        odom_fresh = (
            self.last_odom_time is not None and
            (now - self.last_odom_time).nanoseconds * 1.0e-9 <= self.dvl_timeout
        )
        if (self.pos_x is None or self.pos_y is None or self.yaw is None or not odom_fresh):
            self.vel_integral_x = 0.0
            self.vel_integral_y = 0.0
            # No valid feedback means no trustworthy corrective wrench.
            return 0.0, 0.0

        speed = math.hypot(self.vel_x, self.vel_y)

        if not self.pos_latched:
            if speed > self.settle_speed:
                # Braking phase: negative feedback on body-frame velocity.
                fx = -self.vel_Kp * self.vel_x
                fy = -self.vel_Kp * self.vel_y
                fx = max(-self.vel_force_limit, min(fx, self.vel_force_limit))
                fy = max(-self.vel_force_limit, min(fy, self.vel_force_limit))
                return fx, fy

            # Capture the hold point once motion has settled.
            self.target_x = self.pos_x
            self.target_y = self.pos_y
            self.pos_integral_x = 0.0
            self.pos_integral_y = 0.0
            self.vel_integral_x = 0.0
            self.vel_integral_y = 0.0
            self.pos_latched = True
            self.get_logger().info(
                f"Position hold latched at x={self.target_x:.3f}, y={self.target_y:.3f}"
            )

        # Outer loop: position error in odom/world frame -> desired world velocity.
        ex_w = self.target_x - self.pos_x
        ey_w = self.target_y - self.pos_y
        lim = self.sk_pos_integral_limit
        self.pos_integral_x = max(-lim, min(self.pos_integral_x + ex_w * dt, lim))
        self.pos_integral_y = max(-lim, min(self.pos_integral_y + ey_w * dt, lim))

        vx_w = self.pos_to_vel_Kp * ex_w + self.pos_to_vel_Ki * self.pos_integral_x
        vy_w = self.pos_to_vel_Kp * ey_w + self.pos_to_vel_Ki * self.pos_integral_y

        # vx_w = self.pos_to_vel_surge_Kp * ex_w + self.pos_to_vel_surge_Ki * self.pos_integral_x
        # vy_w = self.pos_to_vel_sway_Kp * ey_w + self.pos_to_vel_sway_Ki * self.pos_integral_y
        
        # Limit commanded planar speed while preserving direction.
        speed_cmd = math.hypot(vx_w, vy_w)
        if speed_cmd > self.sk_vel_max > 0.0:
            scale = self.sk_vel_max / speed_cmd
            vx_w *= scale
            vy_w *= scale

        # Rotate world velocity target into base_link using yaw (ENU convention).
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        vx_b = c * vx_w + s * vy_w
        vy_b = -s * vx_w + c * vy_w

        # Inner loop: body-frame velocity PI. The integral term rejects constant
        # disturbances; clamp it and reset it when feedback is stale.
        self.vel_integral_x = max(
            -self.vel_integral_limit,
            min(self.vel_integral_x + (vx_b - self.vel_x) * dt, self.vel_integral_limit)
        )
        self.vel_integral_y = max(
            -self.vel_integral_limit,
            min(self.vel_integral_y + (vy_b - self.vel_y) * dt, self.vel_integral_limit)
        )

        fx = self.vel_Kp * (vx_b - self.vel_x) + self.vel_Ki * self.vel_integral_x
        fy = self.vel_Kp * (vy_b - self.vel_y) + self.vel_Ki * self.vel_integral_y

        # fx = self.vel_surge_Kp * (vx_b - self.vel_x) + self.vel_surge_Ki * self.vel_integral_x
        # fy = self.vel_sway_Kp * (vy_b - self.vel_y) + self.vel_sway_Ki * self.vel_integral_y

        # Acceleration feedback damping. Positive measured acceleration adds
        # an opposing force; the filtered estimate limits derivative noise.
        # Units: accel_gain [N/(m/s^2)] * acceleration [m/s^2] = force [N].
        fx -= self.accel_gain * self.accel_x
        fy -= self.accel_gain * self.accel_y

        fx = max(-self.vel_force_limit, min(fx, self.vel_force_limit))
        fy = max(-self.vel_force_limit, min(fy, self.vel_force_limit))

        self.get_logger().info(
            f"SK target=({self.target_x:.2f},{self.target_y:.2f}) "
            f"pos=({self.pos_x:.2f},{self.pos_y:.2f}) "
            f"err=({ex_w:.2f},{ey_w:.2f}) "
            f"body_vel=({self.vel_x:.2f},{self.vel_y:.2f}) "
            f"force=({fx:.1f},{fy:.1f})",
            throttle_duration_sec=0.5
        )
        return fx, fy

    def depth_pid(self, dt):
        """Calculate the depth-hold output using proportional, integral, and derivative terms."""
        if self.current_depth is None:
            return 0.0

        if self.target_depth is None:
            return 0.0

        error = (self.current_depth - self.target_depth)
        if abs(error) < self.depth_deadband:
            error = 0.0
        self.integral_error += (error * dt)
        self.integral_error = max(-self.integral_limit, min(self.integral_error, self.integral_limit))

        self.last_error = error

        derivative = self.depth_velocity

        output = (self.Kp * error) + (self.Ki*self.integral_error) + (self.Kd*derivative)
        #Saturate
        output = max(-self.depth_force_limit, min(output, self.depth_force_limit))

        self.get_logger().info(
            f"target={self.target_depth:.3f} "
            f"depth={self.current_depth:.3f} "
            f"error={error:.3f} "
            f"vel={self.depth_velocity:.3f} "
            f"Fz={output:.2f}",
            throttle_duration_sec=0.5
        )
        
        return output

    def yaw_pid(self, dt):
        """Calculate the yaw-hold torque using heading error and yaw rate."""
        if self.yaw is None or self.target_yaw is None:
            return 0.0

        error = wrap_angle(self.target_yaw - self.yaw)
        self.yaw_integral += error * dt
        self.yaw_integral = max(-self.att_integral_limit, min(self.yaw_integral, self.att_integral_limit))

        output = (self.yaw_Kp * error) + (self.yaw_Ki * self.yaw_integral) - (self.yaw_Kd * self.yaw_rate)
        output = max(-self.yaw_torque_limit, min(output, self.yaw_torque_limit))

        return output

    def roll_pid(self, dt):
        """Calculate roll-stabilizing torque while smoothly transitioning the roll target to zero."""
        if self.roll is None:
            return 0.0

        #Reference moves from the release angle to 0 over roll_return_time
        roll_ref = 0.0
        roll_ref_rate = 0.0
        if self.roll_return_start_time is not None and self.roll_return_time > 0.0:
            t = (self.get_clock().now() - self.roll_return_start_time).nanoseconds * 1.e-9
            if t < self.roll_return_time:
                s = t/self.roll_return_time
                blend = 10.0*s**3 - 15.0*s**4 + 6.0*s**5
                dblend = (30.0*s**2 - 60.0*s**3 + 30.0*s**4) / self.roll_return_time
                roll_ref = self.roll_return_start_angle * (1.0 - blend) #Scales release angle down to 0
                roll_ref_rate = -self.roll_return_start_angle * dblend

        error = roll_ref - self.roll
        self.roll_integral += error * dt
        self.roll_integral = max(-self.att_integral_limit, min(self.roll_integral, self.att_integral_limit))

        output = (self.rp_Kp * error) + (self.rp_Ki * self.roll_integral) + (self.rp_Kd * (roll_ref_rate - self.roll_rate))
        output = max(-self.rp_torque_limit, min(output, self.rp_torque_limit))

        return output

    def position_pid(self, dt):
        """Calculate station-keeping forces from position error and velocity feedback."""
        #Error and integral are kept ini the world frame so a constant current is cancelled
        ex_w = self.target_x - self.pos_x
        ey_w = self.target_y - self.pos_y
        self.pos_integral_x += ex_w * dt
        self.pos_integral_y += ey_w * dt
        lim = self.pos_integral_limit
        self.pos_integral_x = max(-lim, min(self.pos_integral_x, lim))
        self.pos_integral_y = max(-lim, min(self.pos_integral_y, lim))

        fx_w = (self.pos_Kp * ex_w) + (self.pos_Ki * self.pos_integral_x)
        fy_w = (self.pos_Kp * ey_w) + (self.pos_Ki * self.pos_integral_y)

        #Rotate the world-frame force into the body frame (roll/pitch at 0, only yaw used in transformation)
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        fx = c * fx_w + s * fy_w
        fy = -s * fx_w + c * fy_w

        #Damping on body velocity
        fx -= self.pos_Kd * self.vel_x
        fy -= self.pos_Kd * self.vel_y

        #Saturate
        fx = max(-self.pos_force_limit, min(fx, self.pos_force_limit))
        fy = max(-self.pos_force_limit, min(fy, self.pos_force_limit))

        self.get_logger().info(
            f"SK target=({self.target_x:.2f},{self.target_y:.2f}) "
            f"Current pos=({self.pos_x}, {self.pos_y})"
            f"err=({ex_w:.2f},{ey_w:.2f}) "
            f"Fx={fx:.1f} Fy={fy:.1f}",
            throttle_duration_sec=0.5
        )
        
        return fx, fy

    def control_loop(self):
        """Run the controllers, combine their outputs into a wrench, and publish it."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1.e-9
        self.last_time = now
        if dt <= 0.0:
            return

        if dt > 0.1:
            dt = 0.1

        output = Wrench()

        # Station-keeping is an exclusive mode: ignore every joystick axis while
        # enabled, not only surge/sway.
        command = Wrench() if self.station_keep_enabled else self.last_wrench

        output.force.x, output.force.y = self.update_position_controller(command, dt)
        output.torque.y = 0.0
        output.torque.x = self.update_roll_controller(command,dt)
        output.torque.z = self.update_yaw_controller(command, dt)
        output.force.z = self.update_depth_controller(command, dt)

        self.cmd_wrench_pub.publish(output)


def main():
    rclpy.init()
    node = StabilizedControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()