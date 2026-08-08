import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# This project's existing convention (see ardupilot_gazebo_env.sh, and
# model.sdf.tracked's own gz_ros2_control <parameters> comment pre-Task-7): a
# worktree-absolute path, not portable across machines/CI. Kept as one
# constant here rather than baked into multiple strings.
WORKTREE_ROOT = '/home/parvu/CaveX-Explorer-Pro/.worktrees/cavex-tracked-blueboat-ardupilot'

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

    # ardupilot_gazebo_env.sh's two env vars, set here as SetEnvironmentVariable
    # actions so they reach the `gz sim` subprocess the IncludeLaunchDescription
    # below launches (sourcing the .sh file before `ros2 launch` runs would
    # only affect this launch-file process's own env, not necessarily every
    # subprocess it spawns via `ros2 launch`'s process-execution machinery --
    # setting them as launch actions is the reliable path). Also prepends
    # cavex_tracked_vehicle's installed `models` dir to GZ_SIM_RESOURCE_PATH --
    # Task 5's report found this exact directory (not its parent) is required
    # for model.sdf.tracked's `models://blueboat/...` mesh URIs to resolve.
    #
    # Task 8 addition: pkg_cavex_slam's installed `models` dir, real, live
    # necessary fix found running this exact launch file after vendoring the
    # cave mesh -- cavex_world.world's new `<include><uri>model://cave_world`
    # failed with "Unable to find uri[model://cave_world]" even though
    # ardupilot_gazebo_env.sh (sourced manually) already covers this path,
    # because this launch file builds its own GZ_SIM_RESOURCE_PATH from
    # scratch (see comment above) rather than inheriting the sourced shell's
    # env for the `gz sim` subprocess -- this line was simply missing before
    # cave_world existed for it to need to resolve.
    set_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(WORKTREE_ROOT, 'ardupilot_gazebo', 'build') + ':' +
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_cavex_tracked, 'models') + ':' +
        os.path.join(pkg_cavex_slam, 'models') + ':' +
        os.path.join(WORKTREE_ROOT, 'ardupilot_gazebo', 'models') + ':' +
        os.path.join(WORKTREE_ROOT, 'ardupilot_gazebo', 'worlds') + ':' +
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    # Structure copied from cavex_slam_nav/launch/gazebo_walker.launch.py
    # (real, proven pattern): gz_sim server-only, same world (dry-cave section
    # has room at this spawn point; Task 8 replaces the placeholder geometry).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
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
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-file', generated_sdf_file,
                   '-name', VEHICLE_MODEL_NAME,
                   '-x', '-35', '-y', '0', '-z', '6.65'],
        output='screen',
    )

    # Carried BlueROV2 cargo (real request: stored on the tracked vehicle through
    # the dry section, released at the water boundary by vehicle_switch_node.py via
    # model.sdf.tracked's own DetachableJoint plugin, added alongside this). Spawned
    # BEFORE the tracked vehicle so the plugin's Configure() (which runs when the
    # tracked vehicle model loads) finds "bluerov2" already present -- untested
    # whether DetachableJoint retries a not-yet-existing child, so this avoids
    # relying on that.
    #
    # Position: moved forward to sit at the same (x, y) as the helipad (local
    # x=0.3, y=0 -- model.sdf.tracked's helipad_link, now raised on a support
    # post), so the ROV is mounted directly UNDER the drone launch pad rather
    # than amidships. Still mounted on the hull deck's top (base_link's own
    # local z=0 -- the hull collision extends only DOWN to z=-0.376, per
    # model.sdf.tracked's real bounding-box comments), same z-clearance
    # reasoning as before (deck-top + 0.03m). bluerov2's own main-body
    # collision box (models/bluerov2/model.sdf) is centered at its local
    # z=0.06 with half-height 0.0325, so its own bottom sits
    # 0.06-0.0325=+0.0275m ABOVE its origin (corrected in final review -- an
    # earlier version of this comment had the sign backwards, claiming the
    # bottom sits below the origin) -- mounting the origin at deck-top + 0.03m
    # puts the ROV's bottom at 0.03+0.0275=~0.058m above deck level, not the
    # ~0.0025m originally claimed. Real clearance from the hull's own solid
    # collision (the actual safety requirement) is larger than intended, not
    # smaller, so this sign error was benign for crash-avoidance -- the real,
    # documented gz-sim gotcha found in /usr/share/gz/gz-sim8/worlds/
    # detachable_joint.sdf's own comments ("this will cause Gazebo to crash because
    # B1 will be in collision with vehicle_blue") means any negative clearance
    # (fully embedding the ROV to make its own TOP, not bottom, flush with the
    # deck) is not safely achievable given the hull's collision has no real
    # gap/moon-pool there -- so "under the pad" means directly below its raised
    # platform, not embedded inside the hull. x = -35 + 0.3 = -34.7 (matching
    # helipad_link's local x offset). z = 6.65 (tracked vehicle spawn) + 0.03
    # = 6.68 (well below the pad deck, now raised to local z=0.35).
    #
    # Known caveat, not fully resolved: cavex_world.world's Buoyancy plugin applies
    # its water-density function by world-frame z alone (below z=7.9 -> density
    # 1000), not scoped to the water region's real x/y extent -- while carried
    # through the dry section (z~6.7, below that threshold), bluerov2
    # technically experiences simulated underwater buoyancy the whole time, not just
    # once released. Since it's rigidly attached to the much heavier tracked vehicle
    # via the DetachableJoint, this manifests as extra load on the joint rather than
    # anything visibly wrong, but it's not physically correct -- flagged here rather
    # than silently accepted.
    spawn_bluerov2_cargo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-file',
                   os.path.join(pkg_cavex_tracked, 'models', 'bluerov2', 'model.sdf'),
                   '-name', 'bluerov2',
                   '-x', '-34.7', '-y', '0', '-z', '6.68'],
        output='screen',
    )

    # Carried PX4 x500 quadcopter (real, vendored fuel.gazebosim.org/PX4/models/x500)
    # on model.sdf.tracked's real helipad_link (front of the hull, local x=0.3,
    # now raised on a support post to pad-deck z=0.35 so the BlueROV2 can be
    # mounted directly underneath it -- see helipad_link's own comment).
    # x500's own real landing-gear feet sit at local z~=-0.227 relative to its
    # own origin (models/x500/model.sdf's real collision box poses) --
    # mounting the origin at pad-deck(0.35) + 0.227 puts the feet right at the
    # helipad surface in principle, but this model spawns SECOND, still
    # BEFORE the tracked vehicle itself (OnProcessExit chain: bluerov2 ->
    # x500 -> tracked vehicle; corrected in final review -- an earlier version
    # of this comment wrongly said "THIRD, after ... the tracked vehicle's own
    # boot sequence"), and live-verified the DetachableJoint does NOT preserve
    # the exact spawn-time
    # offset -- it free-falls under gravity for however long elapses before the
    # parent model's plugin actually attaches it, losing real height in a way
    # that varied between two otherwise-identical launches this session (0.237m
    # -> 0.014m observed once, uncomfortably close to the hull's own collision
    # top). Given +0.3m of headroom (well beyond the observed worst-case loss)
    # empirically kept the settled offset safely positive across a retest.
    # World z = 6.65 (tracked vehicle spawn) + 0.35 (raised pad-deck) + 0.237
    # + 0.3 margin = 7.537; x = -35 + 0.3 = -34.7 (matching helipad_link's
    # local x offset). Same spawn-before-parent ordering requirement as
    # bluerov2 above, and the same placeholder scope: no PX4 SITL/
    # flight-control integration here, see model.sdf.tracked's own comment on
    # this drone's DetachableJoint block.
    spawn_x500_cargo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-file',
                   os.path.join(pkg_cavex_tracked, 'models', 'x500', 'model.sdf'),
                   '-name', 'x500',
                   '-x', '-34.7', '-y', '0', '-z', '7.537'],
        output='screen',
    )

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
    )

    # Real launch arg (confirmed via `ros2 launch ardupilot_sitl
    # sitl_dds_udp.launch.py --show-args`): 'command' selects the SITL binary,
    # 'ardurover' is a valid choice; 'model:=rover' + comma-joined
    # 'defaults:=rover.parm,dds_udp.parm' is ArduPilot's own multi-file
    # defaults syntax and is what actually turns DDS on (dds_udp.parm sets
    # DDS_ENABLE 1 / DDS_UDP_PORT 2019) -- same combination Task 3 verified
    # live end-to-end.
    pkg_ardupilot_sitl = get_package_share_directory('ardupilot_sitl')
    rover_defaults = os.path.join(pkg_ardupilot_sitl, 'config', 'default_params', 'rover.parm')
    dds_udp_defaults = os.path.join(pkg_ardupilot_sitl, 'config', 'default_params', 'dds_udp.parm')
    ardupilot_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ardupilot_sitl, 'launch', 'sitl_dds_udp.launch.py')
        ),
        launch_arguments={
            'command': 'ardurover',
            'model': 'rover',
            'defaults': f'{rover_defaults},{dds_udp_defaults}',
            'synthetic_clock': 'False',
            'use_sim_time': 'False',
        }.items(),
    )

    cmd_vel_to_ardupilot = Node(
        package='cavex_tracked_vehicle',
        executable='cmd_vel_to_ardupilot.py',
        name='cmd_vel_to_ardupilot',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    track_cmd_vel_bridge_node = Node(
        package='cavex_tracked_vehicle',
        executable='track_cmd_vel_bridge.py',
        name='track_cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    track_retract_control = Node(
        package='cavex_tracked_vehicle',
        executable='track_retract_control.py',
        name='track_retract_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Task 19: watches the tracked vehicle's real ground truth and triggers the
    # water-boundary handoff (track retraction + BlueROV2 release via the
    # DetachableJoint plugin above) -- needs tracked_vehicle_ground_truth_odom.py
    # (started by tracked_vehicle_slam.launch.py, not here) to actually be
    # publishing /odom_ground_truth for its trigger condition to ever fire.
    vehicle_switch_node = Node(
        package='cavex_tracked_vehicle',
        executable='vehicle_switch_node.py',
        name='vehicle_switch_node',
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
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_track_retract_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['track_retract_controller'],
        output='screen',
    )

    return LaunchDescription([
        set_plugin_path,
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_bluerov2_cargo,
        gz_bridge,
        ardupilot_sitl_launch,
        cmd_vel_to_ardupilot,
        track_cmd_vel_bridge_node,
        track_retract_control,
        vehicle_switch_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_bluerov2_cargo,
                on_exit=[spawn_x500_cargo],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_x500_cargo,
                on_exit=[spawn_entity],
            )
        ),
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
