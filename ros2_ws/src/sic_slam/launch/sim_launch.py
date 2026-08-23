import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true = server only (-s), no GUI window')
    gz_flag = PythonExpression(
        ['"-r -s" if "', LaunchConfiguration('headless'), '" == "true" else "-r"'])

    pkg_share = get_package_share_directory('sic_slam')
    # bluerov2_sim's model.sdf lives here (an ArduPilot-plugin-free copy of
    # cavex_tracked_vehicle's own bluerov2/model.sdf -- thrusters driven
    # directly via cmd_thrust, no ArduSub SITL needed for this pipeline).
    # It has no meshes of its own: its model://bluerov2/meshes/*.dae URIs
    # resolve against cavex_tracked_vehicle's already-vendored bluerov2
    # mesh directory via GZ_SIM_RESOURCE_PATH below, so the ~17 MB of
    # meshes isn't duplicated a second time in this package.
    cavex_vehicle_share = get_package_share_directory('cavex_tracked_vehicle')
    world_file = os.path.join(pkg_share, 'worlds', 'sic_slam_tank.world')
    model_file = os.path.join(pkg_share, 'models', 'bluerov2_sim', 'model.sdf')
    bridge_yaml = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    checkpoint_path = os.path.join(pkg_share, 'models', 'uuv_controller.pth')

    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, 'models') + ':' +
        os.path.join(cavex_vehicle_share, 'models'))

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [gz_flag, ' ', world_file]}.items(),
    )

    spawn_bluerov2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'sic_slam_tank', '-name', 'bluerov2',
                   '-file', model_file, '-x', '0', '-y', '0', '-z', '-2'],
        output='screen',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen',
    )

    ping360_sim = Node(
        package='sic_slam',
        executable='ping360_sim_node.py',
        name='ping360_sim_node',
        output='screen',
    )

    perception_bridge = Node(
        package='sic_slam',
        executable='sic_slam_perception_bridge.py',
        name='sic_slam_perception_bridge',
        output='screen',
        parameters=[{'checkpoint_path': checkpoint_path}],
    )

    graph_backend = Node(
        package='sic_slam',
        executable='sic_slam_graph_backend.py',
        name='sic_slam_graph_backend',
        output='screen',
    )

    flight_logger = Node(
        package='sic_slam',
        executable='sic_slam_flight_logger.py',
        name='sic_slam_flight_logger',
        output='screen',
    )

    return LaunchDescription([
        headless_arg,
        gz_resource_path,
        gz_sim,
        spawn_bluerov2,
        gz_bridge,
        ping360_sim,
        perception_bridge,
        graph_backend,
        flight_logger,
    ])
