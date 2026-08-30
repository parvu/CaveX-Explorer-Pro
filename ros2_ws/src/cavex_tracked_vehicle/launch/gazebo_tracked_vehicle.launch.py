import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Task 4/5/6's real, confirmed spawned model name -- NOT cavex_tracked_vehicle
# (the brief's Interfaces section names the package, not the spawned model;
# using the package name for model-scoped topics was the brief's own mistake,
# corrected here and in the verification commands below).
VEHICLE_MODEL_NAME = 'cavex_tracked_blueboat'


def generate_launch_description():
    pkg_cavex_tracked = get_package_share_directory('cavex_tracked_vehicle')
    pkg_cavex_slam = get_package_share_directory('cavex_slam_nav')

    world_file = os.path.join(pkg_cavex_slam, 'worlds', 'cavex_world.world')
    urdf_stub_file = os.path.join(pkg_cavex_tracked, 'urdf', 'cavex_tracked_vehicle.urdf')
    ros2_control_yaml = os.path.join(pkg_cavex_tracked, 'config', 'cavex_tracked_vehicle_ros2_control.yaml')
    combined_bridge_yaml = os.path.join(pkg_cavex_tracked, 'config', 'gazebo_tracked_vehicle_bridge.yaml')
    core_bridge_yaml = os.path.join(pkg_cavex_tracked, 'config', 'gazebo_tracked_vehicle_bridge_core.yaml')

    # sensors:=false (manual-drive runs) swaps the parameter_bridge to a
    # config with no /imu, /lidar, /camera entries. With those sensors at
    # always_on=0 in model.sdf.tracked, no subscriber = the gz-sim-sensors
    # system never renders them -- measured live 2026-08-29 as the dominant
    # gz sim CPU cost on this world. Leave true for the SLAM stack, which
    # needs the point cloud / RGB-D / imu.
    declare_sensors = DeclareLaunchArgument(
        'sensors', default_value='true',
        description='Bridge + render the camera/lidar/imu sensors. Set false '
                    'for manual-drive runs to recover real-time factor.')
    sensors = LaunchConfiguration('sensors')
    sdf_template_file = os.path.join(pkg_cavex_tracked, 'models', 'blueboat', 'model.sdf.tracked')

    # model.sdf.tracked's gz_ros2_control <parameters> tag can't use xacro's
    # $(find pkg) (this is native SDF, no xacro preprocessing step -- Task 5
    # confirmed the literal, unexpanded string aborts gz sim at model load),
    # so the checked-in file ships a __ROS2_CONTROL_PARAMS_PATH__ placeholder
    # token instead of a worktree-absolute literal. Substitute a real
    # FindPackageShare-resolved path here at launch time and spawn the
    # generated copy -- not the checked-in template -- so the template stays
    # portable across machines/worktrees/CI. A plain string .replace() is the
    # right amount of code for one token; no templating library needed.
    with open(sdf_template_file) as f:
        sdf_content = f.read().replace('__ROS2_CONTROL_PARAMS_PATH__', ros2_control_yaml)
    generated_sdf_file = '/tmp/cavex_tracked_blueboat.generated.sdf'
    with open(generated_sdf_file, 'w') as f:
        f.write(sdf_content)

    # GZ_SIM_RESOURCE_PATH for the `gz sim` subprocess, set as a launch
    # action (not a sourced .sh) so it reaches the subprocess reliably.
    # Needs BOTH project model dirs explicitly: cavex_tracked_vehicle's
    # installed `models` dir (this exact dir, not its parent) for
    # model.sdf.tracked's `model://blueboat/...` + `model://x500/...` mesh
    # URIs, and cavex_slam_nav's `models` dir for cavex_world.world's
    # `<include><uri>model://cave_world`. ArduPilot removed 2026-08-29 --
    # the ardupilot_gazebo build/models/worlds fragments went with it, as
    # did GZ_SIM_SYSTEM_PLUGIN_PATH (only ArduPilotPlugin.so needed it).
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_cavex_tracked, 'models') + ':' +
        os.path.join(pkg_cavex_slam, 'models') + ':' +
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    # Structure copied from cavex_slam_nav/launch/gazebo_walker.launch.py
    # (real, proven pattern): gz_sim server-only, same world (dry-cave section
    # has room at this spawn point; Task 8 replaces the placeholder geometry).
    #
    # Real fix (memory/CPU optimization): this comment already said
    # "server-only" but gz_args never actually included -s -- the full
    # GUI-attached process (Ogre2 rendering, this environment's own
    # GALLIUM_DRIVER=d3d12 software translation path) was launching by
    # default every single time. Measured live this session: the GUI
    # process alone was consuming 7.5GB+ RSS and contributing directly to
    # repeated CPU-saturation stalls (system load observed as high as
    # 39 on an 8-core box with it running). Headless is now the real
    # default, matching the comment's own stated intent; attach a GUI on
    # demand exactly as history.txt's own section 6 already documents:
    # `gz sim -g &` connects to this already-running headless server, no
    # separate world/relaunch needed.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s {world_file}'}.items(),
    )

    # gz_ros2_control's real, reproduced behavior (Task 5 finding, contradicts
    # gz_ros2_control's own upstream docs): GazeboSimROS2ControlPlugin.Configure()
    # still calls controller_manager::ControllerManager::robot_description_callback,
    # which blocks forever waiting for an external actor to publish a URDF (not
    # the SDF) with matching <ros2_control> joint/interface names on
    # /robot_description, transient-local -- even though model.sdf.tracked
    # already has its own SDF-embedded <ros2_control> block. Without this,
    # joint_state_broadcaster/track_retract_controller never initialize and the
    # retraction joints are silently never actuatable. This stub URDF (declares
    # only base_link + the two retraction joints + the matching <ros2_control>
    # block -- not a physical replica of the real hull) exists solely to
    # satisfy that wait; it is never spawned into the simulation.
    with open(urdf_stub_file) as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawns the real vehicle: native SDF (the generated/templated copy of
    # model.sdf.tracked, per the path fix above), not the robot_state_publisher
    # URDF stub above (unrelated -- that stub is launch-time-only).
    #
    # Spawn point history: Task 8's original probe-drop test at x=-60, y=0
    # (a 0.5m box dropped from z=12, settling at z~=0.25) was later found to
    # have been resting on cavex_world.world's flat ground_plane, not real
    # cave mesh collision -- the vendored mesh has a genuine, confirmed hole
    # in floor coverage at that exact spot (zero mesh vertices found within a
    # 6x6m window there). Re-derived from the mesh's own real vertex data
    # instead of another probe-drop test (see cavex_world.world's
    # cave_floor_patch comment for the full derivation): a supplementary
    # floor collision patch now covers x [-40,70] y [-12,12] at a real,
    # vertex-confirmed height CAVE_FLOOR_Z=5.9. Spawn moved to x=-35 (inside
    # that coverage, west of all four dry-section obstacles), z = 5.9 + 0.75
    # clearance = 6.65.
    # Moved for the cave_world 2x mesh scale (real request, models/cave_world/
    # model.sdf's own comment has the full story): scaling happens around the
    # include's local origin, not around this spawn point, so the real
    # corridor that used to be here moved. Re-derived precisely, not guessed
    # -- inverted this exact SDF pose's rotation+translation to get the old
    # spawn point's LOCAL mesh coordinate, then re-applied the same
    # transform with scale=2 (round-trip-verified against the documented
    # x=-37 floor point first). That's a global rigid+uniform-scale
    # transform, so it's topology-preserving: the old spawn point was real,
    # empirically-verified open air, and its image under this transform is
    # therefore also real open air, not a guess. New position: old (-35, 0)
    # + delta (-53.78, -31.4) = (-88.78, -31.4). z kept at 6.65 unchanged --
    # the transform pose's own z (5.9826) sits almost exactly at floor
    # height, so points near the floor barely move in z when scaled around
    # it (computed real floor at the new location: ~5.98, vs 5.9 before).
    # See cave_floor_patch_scaled below for the supplementary floor
    # collision added at this new location (the original patch's coverage,
    # x[-40,70] y[-12,12], does not reach here).
    # 2026-08-30: HOME position (-90, -35). Real request (moved from -40 -> -35).
    # Solid cave floor (mesh-verified z ~= 5.9, ceiling ~17, dense -- open
    # corridor). z = 5.9 + 0.75 clearance = 6.65. The cave mesh is trimmed to
    # the play box (tools/trim_cave_mesh.py) and its open corridor mouths are
    # sealed by the cap_plate_1..9 box models in cavex_world.world, so the
    # vehicle can no longer drive off the mesh into the void.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-file', generated_sdf_file,
                   '-name', VEHICLE_MODEL_NAME,
                   '-x', '-90', '-y', '-35', '-z', '6.65'],
        output='screen',
    )

    # BlueROV2 (real request, 2026-08-26: "make bluerov2 static in
    # reference with blueboat not the world") is no longer spawned as a
    # separate entity at all -- it is now a fixed child link of the boat's
    # own model (see model.sdf.tracked's own bluerov2_link comment for the
    # full reasoning: a static body ignores every force/joint constraint
    # a physics engine can apply, so the old DetachableJoint-lock +
    # motorized-tether design could no longer keep a separate static ROV
    # attached to a moving boat). No separate spawn_bluerov2 node,
    # spawn_bluerov2_retry safety net, or ROV lock/unlock mechanic exists
    # any more -- see perception branch for the full, functional,
    # independently-tethered BlueROV2.

    # x500 quadcopter is no longer a separate spawned model. Real request
    # 2026-08-28: it is now a fixed DECOR child link (x500_link) of
    # model.sdf.tracked -- visuals + ~2.06kg mass only, no DetachableJoint.
    # See that file's own x500_link comment. This removes spawn_x500_cargo,
    # its BEFORE-the-boat spawn ordering, and the /cavex/x500_release/detach
    # path entirely.

    # gz_bridge: ONE combined parameter_bridge process (real, structural fix,
    # not a Task 7/12 code bug -- see gazebo_tracked_vehicle_bridge.yaml's own
    # header comment for the full live-diagnosed root cause). This used to be
    # two separate `parameter_bridge` processes (one for sensors via CLI args,
    # one for track_cmd_vel via its own config file); ros_gz_bridge's
    # parameter_bridge auto-bridges /clock by default on EVERY instance it
    # runs, so running two processes structurally created two independent,
    # competing /clock publishers -- confirmed live via `ros2 topic info
    # /clock -v` showing two distinct `ros_gz_bridge` publisher GIDs -- which
    # produced real timing jitter that broke icp_odometry's frame-to-frame
    # registration entirely (ratio stuck at 0.000000 even with substantial
    # real vehicle motion) and, downstream, RTAB-Map's WM staying at 0
    # forever. Merged into a single config-file-driven bridge so there is
    # only ever one /clock relay. Topic names re-verified empirically
    # (Step 2, and again live for this fix) rather than assumed, per this
    # project's established gz-sim topic-naming gotcha (<topic> overrides /
    # sensor scoping don't reliably follow the naive convention) -- see
    # gazebo_tracked_vehicle_bridge.yaml for the real, confirmed topic names.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={combined_bridge_yaml}'],
        output='screen',
        condition=IfCondition(sensors),
    )
    gz_bridge_core = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={core_bridge_yaml}'],
        output='screen',
        condition=UnlessCondition(sensors),
    )


    track_retract_control = Node(
        package='cavex_tracked_vehicle',
        executable='track_retract_control.py',
        name='track_retract_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Republishes this vehicle's real ground-truth pose (via gz-transport
    # directly -- see that file's header) as /odom_ground_truth. Lives here,
    # not in tracked_vehicle_slam.launch.py, because every control node below
    # (vehicle_switch, boat_buoyancy, boat_thruster, skid_steer) is driven
    # entirely by its /odom_ground_truth callback -- without it the vehicle
    # simply does not respond to cmd_vel (found live 2026-08-28, "not
    # moving"). The full stack runs this launch too, so it's still exactly
    # one instance.
    tracked_vehicle_ground_truth_odom = Node(
        package='cavex_tracked_vehicle',
        executable='tracked_vehicle_ground_truth_odom.py',
        name='tracked_vehicle_ground_truth_odom',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Task 19: watches the tracked vehicle's real ground truth and triggers
    # track retraction on the water-boundary crossing. Real request,
    # 2026-08-26: no longer manages a separate BlueROV2's lock/unlock or
    # tether -- bluerov2 is now a fixed child link of the boat's own model
    # (see model.sdf.tracked's bluerov2_link comment), so there is no
    # separate ROV entity to release or tether. Consumes /odom_ground_truth
    # from tracked_vehicle_ground_truth_odom just above.
    vehicle_switch_node = Node(
        package='cavex_tracked_vehicle',
        executable='vehicle_switch_node.py',
        name='vehicle_switch_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Real request, 2026-08-26: consumer for cavex_world.world's new
    # ActionButtons/ManualControl GUI plugins (Track up/down, Rover
    # lock/unlock, D-pad + turn-left/right). Auto-launched here for the
    # same reason sic_slam's sim_launch.py auto-launches
    # manual_control_node.py -- clicking the GUI's buttons would
    # otherwise silently do nothing until an operator remembered to start
    # this separately. Publishes nothing on /cmd_vel while the Manual
    # toggle is off, same convention.
    manual_gui_bridge = Node(
        package='cavex_tracked_vehicle',
        executable='manual_gui_bridge.py',
        name='manual_gui_bridge',
        output='screen',
    )

    # Real request 2026-08-26: replaces cavex_world.world's Buoyancy plugin for this
    # vehicle (left disabled there -- see its own comment) with a manual, region- and
    # righting-aware lift. Same /odom_ground_truth dependency as
    # vehicle_switch_node -- idle with zero lift (not a crash) until
    # tracked_vehicle_ground_truth_odom above is publishing.
    boat_buoyancy_control = Node(
        package='cavex_tracked_vehicle',
        executable='boat_buoyancy_control.py',
        name='boat_buoyancy_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Real request 2026-08-26: mixes the boat's real drive command into thrust for the
    # twin motor/prop assemblies re-added to model.sdf.tracked. Same /odom_ground_truth
    # dependency/caveat as vehicle_switch_node and boat_buoyancy_control above.
    boat_thruster_control = Node(
        package='cavex_tracked_vehicle',
        executable='boat_thruster_control.py',
        name='boat_thruster_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Land locomotion. The engine is bullet-featherstone (for real cave-mesh
    # collision + RTF), under which gz-sim's TrackedVehicle/TrackController
    # plugins produce NO motion (they steer via track-link velocity + surface
    # friction, a dartsim-only path). This node drives base_link directly
    # from the same cmd_vel while /cavex/locomotion_mode is 'tracks'/
    # 'retracting' -- exact complement of boat_thruster_control's props gate.
    # Same /odom_ground_truth dependency as the nodes above.
    skid_steer_control = Node(
        package='cavex_tracked_vehicle',
        executable='skid_steer_control.py',
        name='skid_steer_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ros2_control's controllers are declared to the controller_manager the
    # gz_ros2_control plugin starts on model spawn, but nothing loads/activates
    # them by itself (same real, empirically-confirmed requirement as the
    # abandoned branch's gazebo_walker.launch.py -- controller_manager just
    # sits there waiting otherwise). Real explicit spawners, sequenced via
    # OnProcessExit (spawn_entity's "create" process exits once the entity is
    # spawned, which is roughly when the controller_manager service becomes
    # available) -- the standard ros2_control launch idiom, copied directly.
    # Real bug found live 2026-08-27 ("boat wont move"): both spawners were
    # reliably dying with "A controller named '...' was already loaded
    # inside the controller manager" -- root-caused, not guessed. The
    # spawner's default --controller-manager-timeout is 10 REAL (wall-clock)
    # seconds, but controller_manager runs on use_sim_time -- its own
    # request-processing cadence is gated by how fast sim time actually
    # advances. Confirmed live via /world/cavex_world/stats:
    # real_time_factor ~=0.31 (this world runs at under a third of real
    # speed under current load). The load_controller call was genuinely
    # succeeding server-side (confirmed in controller_manager's own log --
    # "Loading controller" printed once, immediately), just too slowly for
    # the spawner's wall-clock deadline to see the response in time; the
    # spawner's own retry then found it already loaded and died instead of
    # succeeding. Raised to 60s of real wall-clock budget, comfortably
    # covering a real_time_factor well below 0.31 without needing to touch
    # the physics/rendering load causing the slowdown itself.
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen',
    )

    load_track_retract_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['track_retract_controller', '--controller-manager-timeout', '60'],
        output='screen',
    )

    # bluerov2 and x500 no longer spawn separately -- both are fixed child
    # links of the boat's own model now (see model.sdf.tracked's
    # bluerov2_link / x500_link comments). The boat spawns directly once
    # gz_sim is up.
    return LaunchDescription([
        declare_sensors,
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge,
        gz_bridge_core,
        track_retract_control,
        tracked_vehicle_ground_truth_odom,
        vehicle_switch_node,
        manual_gui_bridge,
        boat_buoyancy_control,
        boat_thruster_control,
        skid_steer_control,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_track_retract_controller],
            )
        ),
    ])
