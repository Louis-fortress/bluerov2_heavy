from simple_launch import SimpleLauncher, GazeboBridge

sl = SimpleLauncher(use_sim_time= True)

sl.declare_arg('namespace', default_value='bluerov2_heavy')
sl.declare_arg('ground_truth', default_value=True)
sl.declare_arg('sliders', default_value=False)
sl.declare_arg('gazebo_world_name', 'none')

# initial pose
sl.declare_gazebo_axes(x=1., y=0., z=2., roll=0.,pitch=0., yaw=0.)


def launch_setup():

    ns = sl.arg('namespace')
    
    if sl.arg('gazebo_world_name') != 'none':
        GazeboBridge.set_world_name(sl.arg('gazebo_world_name'))

    # robot state publisher
    sl.include('bluerov2_description', 'bluerov2_heavy_state_publisher_launch.py',
                launch_arguments={'namespace': ns, 'use_sim_time': sl.sim_time})

    with sl.group(ns=ns):
                        
        # URDF spawner to Gazebo, defaults to relative robot_description topic
        sl.spawn_gz_model(ns, spawn_args = sl.gazebo_axes_args())
            
        # ROS-Gz bridges
        bridges = []
        gz_js_topic = GazeboBridge.model_prefix(ns) + '/joint_state'
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))

        #pressure
        bridges.append(GazeboBridge(f'{ns}/pressure',
                                    'pressure', 'sensor_msgs/FluidPressure', GazeboBridge.gz2ros))

        #Pressure->depth
        sl.node('sensor_pkgs', 'pressure_to_depth.py',
                parameters={'frame_id': ns + '/odom'})

        #Dvl
        bridges.append(GazeboBridge(f'{ns}/dvl', 'dvl_twist', 'geometry_msgs/Twist', GazeboBridge.gz2ros))
        bridges.append(GazeboBridge(f'{ns}/dvl/altitude', 'dvl_altitude', 'std_msgs/Float64', GazeboBridge.gz2ros))

        #Dvl covariance output
        sl.node('sensor_pkgs', 'dvl_twist_cov.py', parameters={'frame_id': ns + '/dvl'})
        
        # pose ground truth
        bridges.append(GazeboBridge(f'/model/{ns}/pose',
                                        'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros))
        
        # ground truth if requested
        if sl.arg('ground_truth'):
            bridges.append(GazeboBridge(f'/model/{ns}/odometry',
                                        'odom', 'nav_msgs/Odometry', GazeboBridge.gz2ros,
                                        'gz.msgs.Odometry'))
            sl.node('pose_to_tf',parameters={'child_frame': ns + '/base_link'},
                    output='screen')
        else:
            # otherwise publish ground truth as another link to get, well, ground truth
            sl.node('pose_to_tf',parameters={'child_frame': ns+'/base_link_gt'})

        # imu
        bridges.append(GazeboBridge(f'{ns}/imu',
                        'imu', 'sensor_msgs/Imu', GazeboBridge.gz2ros))

        #robot localization
        sl.node('robot_localization', 'ekf_node', name='ekf_filter_node',
                parameters=[sl.find('sensor_pkgs', 'ekf_heavy.yaml')],
                output='screen')

        # # sonar (as laser scan for now...)
        # bridges.append(GazeboBridge(f'{ns}/sonar', 'sonar', 'sensor_msgs/LaserScan', GazeboBridge.gz2ros))
        # bridges.append(GazeboBridge(f'{ns}/sonar/points', 'cloud', 'sensor_msgs/PointCloud2', GazeboBridge.gz2ros))

        # # camera
        # if sl.arg('camera'):
        #     bridges.append(GazeboBridge(f'{ns}/image', 'image', 'sensor_msgs/Image', GazeboBridge.gz2ros))
        
        # # tilt control
        # bridges.append(GazeboBridge(f'/model/{ns}/joint/tilt/0/cmd_pos',
        #                                 'cmd_tilt', 'std_msgs/Float64', GazeboBridge.ros2gz))
        
        # thrusters
        for thr in range(1, 9):
            thruster = f'thruster{thr}'
            gz_thr_topic = f'/{ns}/{thruster}/cmd'
            bridges.append(GazeboBridge(gz_thr_topic, f'cmd_{thruster}', 'std_msgs/Float64', GazeboBridge.ros2gz))

        sl.create_gz_bridge(bridges)

        if sl.arg('sliders'):
            sl.node('slider_publisher', arguments=[sl.find('bluerov2_description', 'manual_heavy.yaml')])
    
    return sl.launch_description()
    
    
generate_launch_description = sl.launch_description(launch_setup)