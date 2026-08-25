import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='true = server only (-s), no GUI window')
    gz_flag = PythonExpression(
        ['"-r -s" if "', LaunchConfiguration('headless'), '" == "true" else "-r"'])

    # Current + turbidity, real cavex_sonar physics (see ping360_sim_node.py
    # and package.xml for why cavex_sonar is reused rather than duplicated).
    current_vx_arg = DeclareLaunchArgument('current_vx', default_value='0.0')
    absorption_arg = DeclareLaunchArgument(
        'absorption_db_per_m', default_value='0.4',
        description='Turbidity proxy: higher = murkier water, shorter sonar range')
    clutter_arg = DeclareLaunchArgument('clutter_probability', default_value='0.0')
    beam_count_arg = DeclareLaunchArgument('beam_count', default_value='64')
    log_training_data_arg = DeclareLaunchArgument('log_training_data', default_value='false')
    training_data_path_arg = DeclareLaunchArgument(
        'training_data_path', default_value='sic_slam_training_data.csv')
    enable_current_factor_arg = DeclareLaunchArgument(
        'enable_current_factor', default_value='true')
    enable_perception_arg = DeclareLaunchArgument(
        'enable_perception', default_value='true',
        description='false = no perception bridge at all (not just untrained) -- '
                    'graph backend gets zero landmark corrections, pure IMU+CurrentFactor '
                    'dead-reckoning, for a true no-CNN baseline')

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

    # Real acoustic physics (absorption/backscatter/clutter/current), reused
    # unmodified from cavex_sonar -- see package.xml/ping360_sim_node.py.
    sonar_node = Node(
        package='cavex_sonar',
        executable='sonar_node',
        name='sonar_node',
        output='screen',
        parameters=[{
            'seed': 42,
            'beam_count': ParameterValue(LaunchConfiguration('beam_count'), value_type=int),
            'absorption_db_per_m': ParameterValue(LaunchConfiguration('absorption_db_per_m'), value_type=float),
            'clutter_probability': ParameterValue(LaunchConfiguration('clutter_probability'), value_type=float),
        }],
    )

    current_field = Node(
        package='cavex_sonar',
        executable='current_field_node.py',
        name='current_field_node',
        output='screen',
        parameters=[{
            'profile': 'constant',
            'vx': ParameterValue(LaunchConfiguration('current_vx'), value_type=float),
        }],
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
        parameters=[{
            'checkpoint_path': checkpoint_path,
            'num_samples': ParameterValue(LaunchConfiguration('beam_count'), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration('enable_perception')),
    )

    training_data_logger = Node(
        package='sic_slam',
        executable='training_data_logger.py',
        name='training_data_logger',
        output='screen',
        parameters=[{'output_csv_path': LaunchConfiguration('training_data_path')}],
        condition=IfCondition(LaunchConfiguration('log_training_data')),
    )

    graph_backend = Node(
        package='sic_slam',
        executable='sic_slam_graph_backend.py',
        name='sic_slam_graph_backend',
        output='screen',
        parameters=[{
            'enable_current_factor': ParameterValue(LaunchConfiguration('enable_current_factor'), value_type=bool),
        }],
    )

    flight_logger = Node(
        package='sic_slam',
        executable='sic_slam_flight_logger.py',
        name='sic_slam_flight_logger',
        output='screen',
    )

    return LaunchDescription([
        headless_arg,
        current_vx_arg,
        absorption_arg,
        clutter_arg,
        beam_count_arg,
        log_training_data_arg,
        training_data_path_arg,
        enable_current_factor_arg,
        enable_perception_arg,
        gz_resource_path,
        gz_sim,
        spawn_bluerov2,
        gz_bridge,
        sonar_node,
        current_field,
        ping360_sim,
        perception_bridge,
        graph_backend,
        flight_logger,
        training_data_logger,
    ])
