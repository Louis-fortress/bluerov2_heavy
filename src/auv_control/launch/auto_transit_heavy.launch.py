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

        # Your custom manual control node
        sl.node(
            'auv_control',
            'auto_transit_control.py',
            name='auto_transit_control',
            parameters=[{
                'use_sim_time': True,
            }],
        )

        sl.node('thruster_manager', 'thruster_manager_node', parameters=[sl.find('bluerov2_control', 'thruster_manager.yaml')])

    return sl.launch_description()