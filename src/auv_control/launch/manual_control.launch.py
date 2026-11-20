import os
from simple_launch import SimpleLauncher
from ament_index_python import get_package_share_directory

def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)
    #thruster_config = os.path.join(get_package_share_directory('bluerov2_control'), 'config', 'thruster_manager.yaml')

    sl.declare_arg('namespace', default_value='bluerov2')

    # Hardware joystick driver
    sl.node(
        'joy',
        'joy_node',
        name='joy_node',
        parameters=[{'device_id': 0, 'deadzone': 0.05, 'autorepeat_rate': 20.0}],
    )

    with sl.group(ns=sl.arg('namespace')):
        # Teleop mapper: joystick -> /bluerov2/cmd_wrench_joy
        sl.node(
            'auv_control',
            'teleop_node.py',
            name='bluerov2_teleop',
            parameters=[{
                'joy_id': 0,
                'surge_axis': 4,
                'sway_axis': 3,
                'heave_axis': 1,
                'roll_axis': 2,
                'pitch_axis': 5,
                'yaw_axis': 0,
                'deadzone': 0.05,
                'surge_force': 10.0,
                'sway_force': 10.0,
                'heave_force': 20.0,
                'roll_torque': 5.0,
                'pitch_torque': 5.0,
                'yaw_torque': 5.0,
            }],
        )

        # Your custom manual control node
        sl.node(
            'auv_control',
            'manual_control.py',
            name='manual_control',
            parameters=[{
                'depth_Kp': 133.1173,
                'depth_Ki': 76.0998,
                'depth_Kd': 0.0,
                'depth_force_limit': 20.0,
                'depth_deadband': 0.03,
                'integral_limit': 2.0,
                'heave_threshold': 0.05,
            }],
        )

        sl.node('thruster_manager', 'thruster_manager_node', parameters=[sl.find('bluerov2_control', 'thruster_manager.yaml')])

    return sl.launch_description()