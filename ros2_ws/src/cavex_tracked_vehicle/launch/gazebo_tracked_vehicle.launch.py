import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler,
                             SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# This project's existing convention (see ardupilot_gazebo_env.sh, and
# model.sdf.tracked's own gz_ros2_control <parameters> comment pre-Task-7): a
# repo-absolute path, not portable across machines/CI. Kept as one constant
# here rather than baked into multiple strings. (Fixed: used to point at the
# now-deleted cavex-tracked-blueboat-ardupilot worktree from before that
# branch was merged into main -- harmless in practice since
# ardupilot_gazebo_env.sh's own correct entries already precede this one on
# the resulting search path, but stale and worth keeping accurate.)
WORKTREE_ROOT = '/home/parvu/CaveX-Explorer-Pro'

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
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-file', generated_sdf_file,
                   '-name', VEHICLE_MODEL_NAME,
                   '-x', '-88.78', '-y', '-31.4', '-z', '6.65'],
        output='screen',
    )

    # Tethered BlueROV2 (real request: no longer rigidly carried cargo --
    # replaced by motorized_tether_control.py, a real force-based constraint
    # via the gz-sim-apply-link-wrench-system world plugin, commanded through
    # /cavex/tether/payout_length_cmd; see model.sdf.tracked's
    # tether_anchor_link comment for why the old DetachableJoint carry was
    # replaced). Spawned AFTER the tracked vehicle -- see the spawn-order
    # comment above LaunchDescription().
    #
    # Position: near tether_anchor_link's own pose (local x=-0.2, moved
    # 0.3m forward of an earlier x=-0.5 stern placement), still clear of
    # the bow-mounted helipad at x=0.3. Base x = -35 + (-0.4) = -35.4, then
    # moved a further +0.3m forward per real request: -35.4 + 0.3 = -35.1
    # (+x is forward, this file's established convention). y = 0.
    #
    # z: base height 6.3155 (ROV top flush with the hull's own top, see the
    # raise-request comment below for the derivation), raised +0.3m to
    # 6.6155, then moved down 0.1m to 6.5155. A prior "move 2cm down" request
    # (intended result 6.4955) only got applied to THIS comment, not the
    # actual '-z' argument below -- a real bug, caught and fixed here along
    # with this turn's own further 2cm-down request: 6.5155 - 0.02 (missed
    # fix) - 0.02 (this request) = 6.4755. All x and z moves deliberately
    # NOT re-checked against the hull's own collision geometry (real
    # request: move the ROV "indifferent from collisions" for this
    # dry-cave-phase adjustment; the rigid DetachableJoint lock, not this
    # spawn placement or the tether, is what actually holds the ROV during
    # the dry section -- see vehicle_switch_node.py).
    # (This is still only accurate at the moment the hull finishes
    # settling -- the ROV is on a real motorized tether, not rigidly
    # attached, so real buoyancy is free to drift it away from this
    # afterward, same as the rest of this file's honest-physics
    # conventions; not fought with an artificial vertical lock.)
    #
    # Known caveat, not fully resolved: cavex_world.world's Buoyancy plugin applies
    # its water-density function by world-frame z alone (below z=7.9 -> density
    # 1000), not scoped to the water region's real x/y extent -- while under tow
    # through the dry section (z~6.6, below that threshold), bluerov2
    # technically experiences simulated underwater buoyancy the whole time, not just
    # once it actually reaches the water region. This was already true of the old
    # rigid-carry design too (flagged there for the same reason) -- not introduced
    # by the tether change, still not physically correct, still flagged rather
    # than silently accepted.
    # Shifted by the same delta as spawn_entity above (cave_world 2x scale),
    # preserving the exact same relative offset from the tracked vehicle.
    spawn_bluerov2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-file',
                   os.path.join(pkg_cavex_tracked, 'models', 'bluerov2', 'model.sdf'),
                   '-name', 'bluerov2',
                   '-x', '-88.88', '-y', '-31.4', '-z', '6.4755'],
        output='screen',
    )

    # Real request: "rov2 not spawen[ing] [always], try something else." The
    # spawn_bluerov2 node above only fires once, via OnProcessExit chained off
    # spawn_entity's own exit -- a single, unverified attempt. This is an
    # independent safety net (NOT chained to anything else's exit, so a
    # disrupted chain upstream can't block it either): it actively polls the
    # world's real pose stream for the boat to actually exist before trying
    # anything (removing the timing race a fixed delay only guesses at), does
    # nothing if bluerov2 already exists (the fast path above already worked),
    # and otherwise retries the real create service call for real -- not a
    # single silent attempt. See spawn_bluerov2_retry.py's own module
    # docstring for the full design. Started 3s in, comfortably after
    # spawn_x500_cargo/spawn_entity's own gz-transport connections are up.
    spawn_bluerov2_retry = TimerAction(
        period=5.0,
        actions=[Node(
            package='cavex_tracked_vehicle',
            executable='spawn_bluerov2_retry.py',
            name='spawn_bluerov2_retry',
            output='screen',
        )],
    )

    # Carried PX4 x500 quadcopter (real, vendored fuel.gazebosim.org/PX4/models/x500)
    # on model.sdf.tracked's real helipad_link (front of the hull, local x=0.3,
    # z=0.005 deck-top -- flush, no support post, see helipad_link's own
    # comment). helipad_deck_visual/collision is a 0.01-thick disc centered
    # on that link origin (no further local offset), so its own top surface
    # is at local z=0.005+0.005=0.01 -- world z = 6.65 (tracked vehicle
    # spawn) + 0.01 = 6.66.
    #
    # x500's own real landing-gear feet sit at local z~=-0.227 relative to
    # its own origin (models/x500/model.sdf's real collision box poses), so
    # spawning its origin at 6.66 + 0.227 = 6.887 puts the feet exactly at
    # the helipad surface -- per this session's request that the legs
    # actually sit on the pad, this is now spawned at that exact resting
    # height rather than dropped onto it from above. (An earlier version of
    # this spawn added +0.3m of drop margin to defend against the
    # DetachableJoint not preserving the exact spawn-time offset if the
    # model free-fell before the plugin attached it -- live-verified that
    # settle height varied 0.237m -> 0.014m of loss between otherwise-
    # identical launches. Spawning already at the target height removes the
    # free-fall these numbers were about in the first place: there's
    # nothing left to fall before the joint fixes it.)
    #
    # REAL BUG FOUND AND FIXED: commit e76a9205 (cave scaled 2x) updated the
    # boat's own spawn (-35,0 -> -88.78,-31.4) and bluerov2's spawn
    # (-35.1,0 -> -88.88,-31.4) to match, but never touched this spawn --
    # confirmed via `git show e76a9205` on this file. x500 was spawning at
    # the OLD pre-scale location, ~54m/~31m away from where the boat and its
    # helipad actually are post-scale -- nowhere near the helipad, most
    # likely landing in empty space or unrelated geometry in the now-2x cave
    # and just free-falling under gravity with nothing for the
    # DetachableJoint to actually hold. Same relative-offset math as before
    # (x = boat_x + 0.3, y = boat_y), just against the real current boat
    # spawn instead of the stale one: x = -88.78 + 0.3 = -88.48,
    # y = -31.4 (matching the boat, same convention bluerov2's own spawn
    # above already uses). z is unchanged -- it never depended on the cave
    # scale (see README's "Cave scaled 2x": vehicle-size/local-height
    # parameters were deliberately left unscaled).
    # Unlike bluerov2 above, x500 still has to spawn BEFORE the boat --
    # see the spawn-order comment above LaunchDescription(). Same
    # placeholder scope as before: no PX4 SITL/flight-control integration
    # here, see model.sdf.tracked's own comment on this drone's
    # DetachableJoint block.
    spawn_x500_cargo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-file',
                   os.path.join(pkg_cavex_tracked, 'models', 'x500', 'model.sdf'),
                   '-name', 'x500',
                   '-x', '-88.48', '-y', '-31.4', '-z', '6.887'],
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
    # Project-specific speed-controller tuning (CRUISE_SPEED/CRUISE_THROTTLE/
    # ATC_SPEED_P/I/D/MOT_SLEWRATE) for this heavy tracked vehicle -- a
    # THIRD defaults file, applied after rover.parm so its values win, kept
    # separate from the vendored ardupilot/ tree rather than editing that
    # shared upstream file directly. See that file's own header comment for
    # the real root cause this addresses (ArduPilot's own GUIDED velocity
    # loop under-throttling at this vehicle's low commanded speeds).
    speed_tuning_defaults = os.path.join(
        get_package_share_directory('cavex_tracked_vehicle'),
        'config', 'tracked_vehicle_speed_tuning.parm')
    ardupilot_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ardupilot_sitl, 'launch', 'sitl_dds_udp.launch.py')
        ),
        launch_arguments={
            'command': 'ardurover',
            'model': 'rover',
            'defaults': f'{rover_defaults},{dds_udp_defaults},{speed_tuning_defaults}',
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
    # water-boundary handoff (track retraction + paying the tether out to let
    # the BlueROV2 operate independently in the water region -- see
    # motorized_tether_control.py below, real bidirectional replacement for
    # the old one-way DetachableJoint release) -- needs
    # tracked_vehicle_ground_truth_odom.py (started by
    # tracked_vehicle_slam.launch.py, not here) to actually be publishing
    # /odom_ground_truth for its trigger condition to ever fire.
    vehicle_switch_node = Node(
        package='cavex_tracked_vehicle',
        executable='vehicle_switch_node.py',
        name='vehicle_switch_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Motorized tether: real force-based constraint (gz-sim-apply-link-wrench
    # -system, world plugin in cavex_world.world) replacing the old rigid
    # DetachableJoint carry. Subscribes both models' real world poses
    # directly via gz-transport (same proven pattern as
    # tracked_vehicle_ground_truth_odom.py, not the ROS bridge -- PoseArray
    # drops the per-pose name field needed to tell the two models apart).
    motorized_tether_control = Node(
        package='cavex_tracked_vehicle',
        executable='motorized_tether_control.py',
        name='motorized_tether_control',
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

    # Spawn order: x500 -> boat -> bluerov2 (kept from an earlier session,
    # not reverted by this round's tether-mechanism revert). x500 spawns
    # before the boat because it's still rigidly DetachableJoint-attached,
    # whose Configure() (running when the boat's own model loads) needs
    # the child model to already exist. bluerov2 has no such constraint
    # (no DetachableJoint of its own) so it spawns after the boat, letting
    # its spawn pose be computed relative to a real hull that already
    # exists.
    return LaunchDescription([
        set_plugin_path,
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_x500_cargo,
        gz_bridge,
        ardupilot_sitl_launch,
        cmd_vel_to_ardupilot,
        track_cmd_vel_bridge_node,
        track_retract_control,
        vehicle_switch_node,
        motorized_tether_control,
        manual_gui_bridge,
        spawn_bluerov2_retry,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_x500_cargo,
                on_exit=[spawn_entity],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[spawn_bluerov2, load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_track_retract_controller],
            )
        ),
    ])
