import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_cavex = get_package_share_directory('cavex_slam_nav')

    world_file = os.path.join(pkg_cavex, 'worlds', 'cavex_world.world')
    urdf_file = os.path.join(pkg_cavex, 'urdf', 'cavex_walker.urdf.xacro')
    gait_yaml = os.path.join(pkg_cavex, 'config', 'cavex_walker_gait.yaml')
    joints_map_yaml = os.path.join(pkg_cavex, 'config', 'cavex_walker_joints_map.yaml')

    # Structure copied from gazebo_sim.launch.py (Task 5 Step 1), same
    # world reused (dry-cave section has room at this spawn point), same
    # -s/-r server-only gz_args pattern (no gzclient GUI here either).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-s -r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
            'use_sim_time': True,
        }]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-name', 'cavex_walker',
                   '-topic', 'robot_description',
                   # Clear ground in the dry-cave section; z=0.6 sits the
                   # base_footprint above the floor (walker's standing
                   # height per Task 4's nominal_height=0.48 + margin).
                   '-x', '-30', '-y', '0', '-z', '0.6'],
        output='screen'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # Topic names re-verified empirically for this robot (Step 4) --
        # this session's established finding is that gz-sim's <topic>
        # overrides don't reliably follow the /world/.../sensor/... scoped
        # convention, same as cavex_robot's bridge required fixing earlier.
        # camera/imu/clock/pose matched the naive names exactly (confirmed
        # via `gz topic -i -t <name>` showing the expected gz.msgs type),
        # but the lidar did NOT: gz-sim's gpu_lidar sensor publishes a
        # gz.msgs.LaserScan on the bare <topic> name (here, /lidar/points)
        # and auto-appends "/points" for the actual gz.msgs.PointCloudPacked
        # point cloud -- confirmed via `gz topic -i -t /lidar/points` (shows
        # LaserScan) vs `gz topic -i -t /lidar/points/points` (shows
        # PointCloudPacked, matching what this task's interface promises).
        arguments=[
            '/lidar/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/color@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Ground-truth pose (Step 3) -- PosePublisher's
            # use_pose_vector_msg=true publishes gz.msgs.Pose_V, which maps
            # to geometry_msgs/PoseArray, not nav_msgs/Odometry. Task 10
            # republishes this as Odometry for ate_evaluator_node.py.
            '/model/cavex_walker/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/lidar/points/points', '/lidar/points'),
            ('/camera/color', '/camera/color/image_raw'),
            ('/camera/camera_info', '/camera/color/camera_info'),
        ],
        output='screen'
    )

    # champ_base's gait-control node (not covered by the plan's own 4
    # steps -- without it nothing ever drives /cmd_vel into leg
    # trajectories; the URDF/ros2_control wiring from Task 4 only enables
    # joint control, it doesn't generate the walking gait itself).
    #
    # Real executable name confirmed via `ros2 pkg executables champ_base`
    # (re-checked this task, matches Task 1's finding): quadruped_controller_node.
    # It subscribes to Twist on "cmd_vel/smooth" (confirmed again via
    # `ros2 topic list`/grep on quadruped_controller.cpp -- champ_base ships
    # no velocity-smoother node upstream of it), remapped below to the
    # project-standard /cmd_vel. It needs "urdf" (the robot_description
    # string, loaded via champ::URDF::loadFromString) plus links_map/
    # joints_map (cavex_walker_joints_map.yaml) and the gait/gazebo/
    # joint_controller_topic params (cavex_walker_gait.yaml, which already
    # sets gazebo: true and the corrected joint_controller_topic).
    quadruped_controller_node = Node(
        package='champ_base',
        executable='quadruped_controller_node',
        name='quadruped_controller_node',
        output='screen',
        parameters=[
            gait_yaml,
            joints_map_yaml,
            {
                'use_sim_time': True,
                'urdf': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
            },
        ],
        remappings=[('cmd_vel/smooth', '/cmd_vel')],
    )

    # ros2_control's controllers (joint_state_broadcaster,
    # joint_group_position_controller -- see cavex_walker_ros2_control.yaml)
    # are declared to the controller_manager the gz_ros2_control plugin
    # starts on model spawn, but nothing loads/activates them by itself --
    # confirmed empirically with a single, uncontaminated launch (no
    # spawners at all): controller_manager logs "Resource Manager has been
    # successfully initialized. Starting Controller Manager services..."
    # and then just sits there; `ros2 control list_controllers` reports
    # "No controllers are currently loaded!" indefinitely, and no /clock or
    # any other topic ever produces data. (An earlier pass here mistakenly
    # concluded activation was automatic, from a launch log where two
    # overlapping `ros2 launch` invocations were running at once -- one
    # instance's spawner really did load+activate the controllers, and the
    # other, redundant instance's spawner then failed with "can not be
    # configured from 'active' state", which looked like unprompted
    # auto-activation but wasn't. Re-tested clean, single-instance, twice.)
    # So: real explicit spawners, sequenced via OnProcessExit (spawn_entity's
    # "create" process exits once the entity is spawned, which is roughly
    # when the controller_manager service becomes available) -- the
    # standard ros2_control launch idiom.
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_joint_group_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_position_controller'],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge,
        quadruped_controller_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_group_position_controller],
            )
        ),
    ])
