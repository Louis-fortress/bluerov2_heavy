#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Pose


class ManualControl(Node):
    def __init__(self):
        super().__init__("manual_control")

        #Ros infrastructure
        self.cmd_wrench_pub = self.create_publisher(Wrench,
                                                       '/bluerov2/wrench', 10)

        self.pose_sub = self.create_subscription(Pose, '/bluerov2/pose_gt', self.pose_callback,
                                                 10)

        self.cmd_wrench_joy_sub = self.create_subscription(Wrench, '/bluerov2/cmd_wrench_joy',
                                                           self.wrench_joy_callback, 10)

        self.timer = self.create_timer(0.01, self.control_loop)

        self.declare_parameter('depth_Kp', 100.0)
        self.declare_parameter('depth_Ki', 15.0)
        self.declare_parameter('depth_Kd', 0.0)
        self.declare_parameter('depth_force_limit', 50.0)
        self.declare_parameter('depth_deadband', 0.0)
        self.declare_parameter('integral_limit', 2.0)
        self.declare_parameter('heave_threshold', 0.05)

        self.Kp = self.get_parameter('depth_Kp').value
        self.Ki = self.get_parameter('depth_Ki').value
        self.Kd = self.get_parameter('depth_Kd').value
        self.depth_force_limit = self.get_parameter('depth_force_limit').value
        self.depth_deadband = self.get_parameter('depth_deadband').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.heave_threshold = self.get_parameter('heave_threshold').value

        self.depth_hold_active = False
        self.current_depth = None
        self.target_depth = None
        self.depth_velocity = 0.0
        self.previous_heave = False
        self.last_error = 0.0
        self.previous_derivative = 0.0
        self.integral_error = 0.0
        self.previous_z = None
        self.previous_pose_time = None
        self.last_time = self.get_clock().now()
        self.last_wrench = Wrench()

        self.output_wrench = Wrench()

    def wrench_joy_callback(self, msg: Wrench):
        # Store the latest joystick wrench so the control loop can use it
        self.last_wrench = msg

    def pose_callback(self, msg: Pose):
        # Convert the pose z to depth (depth = -z) and estimate the depth rate by differencing
        now = self.get_clock().now()
        current_z = msg.position.z
        self.current_depth = -current_z
        if self.previous_z is not None:
            dt = (now-self.previous_pose_time).nanoseconds * 1e-9
            if dt > 0.0:
                z_velocity = (current_z - self.previous_z)/dt
                self.depth_velocity = -z_velocity

        self.previous_z = current_z
        self.previous_pose_time = now

    def update_depth_controller(self, wrench, dt):
        # Pass the heave stick through while it is moved; latch the depth on release and hold it with the PID
        current_heave = wrench.force.z
        heave_active = (abs(current_heave) > self.heave_threshold)

        if heave_active:
            self.depth_hold_active = False
            self.integral_error = 0.0
            self.previous_heave = True
            return current_heave

        if self.previous_heave:
            if self.current_depth is not None:
                self.target_depth = self.current_depth
                self.integral_error = 0.0
                self.last_error = 0.0
                self.previous_derivative = 0.0
                self.depth_hold_active = True
                self.get_logger().info(
                    f"Depth hold enabled at {self.target_depth: .3f} m"
                )

        self.previous_heave = False

        if self.depth_hold_active:
            return self.depth_pid(dt)

        return 0.0

    def control_loop(self):
        # 100 Hz loop: pass the joystick wrench through, replace heave with the depth controller output, publish
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        #Prevent a large timestep after a pause
        if dt > 0.1:
            dt = 0.1

        output = Wrench()

        output.force.x = self.last_wrench.force.x
        output.force.y = self.last_wrench.force.y
        output.torque.x = self.last_wrench.torque.x
        output.torque.y = self.last_wrench.torque.y
        output.torque.z = self.last_wrench.torque.z

        output.force.z = self.update_depth_controller(self.last_wrench, dt)

        self.cmd_wrench_pub.publish(output)

    def depth_pid(self, dt):
        # PID on depth error (error > 0 means too deep, so the output pushes up); returns a saturated heave force
        if self.current_depth is None:
            return 0.0

        if self.target_depth is None:
            return 0.0

        error = (self.current_depth - self.target_depth)
        #Deadband check
        if abs(error) < self.depth_deadband:
            error = 0.0
        self.integral_error += (error * dt)
        self.integral_error = max(-self.integral_limit, min(self.integral_error, self.integral_limit))

        # if self.N > 0:
        #     derivative = (self.N * (error - self.last_error) + self.previous_derivative/(1.0+self.N*dt))
        #     self.previous_derivative = derivative
        # else:
        #     derivative = 0.0

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
            f"Fz={output:.2f}"
        )

        return output

    
def main():
    rclpy.init()
    node = ManualControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()        