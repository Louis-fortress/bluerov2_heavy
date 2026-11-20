import os
from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)
    #thruster_config = os.path.join(get_package_share_directory('bluerov2_control'), 'config', 'thruster_manager.yaml')

    sl.declare_arg('namespace', default_value='bluerov2_heavy')

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
            'teleop_node_heavy.py',
            name='bluerov2_heavy_teleop',
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
            'stabilized_control_acceleration_feedback_heavy.py',
            name='stabilized_control_heavy',
            parameters=[{
                'use_sim_time': True,
            }],
        )

        sl.node('thruster_manager', 'thruster_manager_node', parameters=[sl.find('bluerov2_control', 'thruster_manager.yaml')])

    return sl.launch_description()