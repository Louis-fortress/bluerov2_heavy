"""
Launches Gazebo Fortress (gz sim) and spawns BlueRov2_fortress.sdf into it.

Assumes:
  - The freebuoyancy_gz package (plugin) has been built and its .so
    installed to install/freebuoyancy_gz/lib/freebuoyancy_gz -- adjust
    PLUGIN_PACKAGE below if you named the package differently.
  - The SDF file and its mesh (model://BlueRov2/meshes/BlueRov2.stl) live
    somewhere under a package's share/ or a models/ directory you control
    -- SDF_PATH and the GZ_SIM_RESOURCE_PATH below are the two lines you
    need to point at your actual layout.
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# TODO: set this to wherever BlueRov2_fortress.sdf actually lives on disk.
SDF_PATH = os.path.join(
    os.path.expanduser('~'), 'central_ws', 'src', 'bluerov2','bluerov2_description',
    'urdf', 'bluerov2_freebuoyancy.sdf')

mesh_path = os.path.join(
    os.path.expanduser('~'), 'central_ws', 'src', 'bluerov2', 'bluerov2_description',
    'model'
)

# TODO: name of the colcon package that builds/installs the plugin .so
PLUGIN_PACKAGE = 'freebuoyancy_gz'

def generate_launch_description():
    # Points gz-sim at the built plugin so <plugin filename=
    # "libfreebuoyancy-system.so"> in the SDF actually resolves.
    plugin_lib_dir = os.path.join(
        get_package_prefix(PLUGIN_PACKAGE), 'lib', PLUGIN_PACKAGE)
    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=plugin_lib_dir)

    # Points gz-sim at the directory containing the BlueRov2/meshes/ folder
    # so the model://BlueRov2/meshes/BlueRov2.stl URI in the SDF resolves.
    # TODO: adjust to wherever your BlueRov2/ model folder actually sits.
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value='/home/louis/central_ws/src/bluerov2/bluerov2_description/model')

    # Starts Gazebo Fortress with an empty world.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items())

    # Spawns the SDF into the running simulation.
    # spawn_vehicle = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-file', SDF_PATH,
    #         '-name', 'bluerov2',
    #         '-z', '0.0',   # spawn a couple meters underwater
    #     ],
    #     output='screen')

    spawn_vehicle = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-file',
        SDF_PATH,
        '-name',
        'bluerov2',
        '-x',
        '0',
        '-y',
        '0',
        '-z',
        '1',
    ],
    output='screen'
    )

    return LaunchDescription([
        set_plugin_path,
        set_resource_path,
        gz_sim,
        spawn_vehicle,
    ])
