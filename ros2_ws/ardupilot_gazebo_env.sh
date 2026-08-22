#!/bin/bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/models:/home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=/home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_slam_nav/models:$GZ_SIM_RESOURCE_PATH
# Real gap found and fixed: cavex_tracked_vehicle/models (bluerov2, blueboat,
# x500) was never on this path, only cavex_slam_nav/models (the cave mesh).
# `ros2 launch cavex_tracked_vehicle ...` never hit this because ROS2 launch
# injects its own package-share resource path automatically -- but every
# manual `gz sim` + `gz service create` spawn (used throughout this
# project's standalone/headless testing) never got that injection. Effect:
# `model://bluerov2/meshes/*.dae` never resolved, so the ROV spawned with
# a real, moving physics body and ZERO visible geometry (confirmed via gz
# sim's own log: "Failed to load geometry for visual: base_link_visual" and
# the same for both thruster prop visuals) -- invisible in the GUI the
# whole time, silently, because collision shapes are separate primitive
# geometry unaffected by mesh-loading failures, so physics/pose telemetry
# was always genuinely correct even while nothing rendered.
export GZ_SIM_RESOURCE_PATH=/home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_tracked_vehicle/models:$GZ_SIM_RESOURCE_PATH

# micro_ros_agent fix (Task 10 follow-up): micro_ros_msgs isn't a real apt/rosdep package on
# ROS2 Jazzy, so this environment vendors it as a prebuilt extract at .local-deps/ros-extract.
# micro_ros_agent's own binary RUNPATH already points there and resolves its direct dependency
# (libmicro_ros_msgs__rosidl_typesupport_cpp.so) fine -- but that library in turn NEEDs
# libmicro_ros_msgs__rosidl_generator_c.so, and per ELF semantics DT_RUNPATH on the top-level
# executable does NOT propagate to transitive (dependency-of-a-dependency) lookups. Both .so
# files sit in the exact same directory (confirmed with readelf -d), but generator_c.so has no
# RPATH/RUNPATH of its own and isn't found unless this directory is in LD_LIBRARY_PATH directly.
export LD_LIBRARY_PATH=/home/parvu/CaveX-Explorer-Pro/ros2_ws/install/micro_ros_msgs/lib:$LD_LIBRARY_PATH

# mavproxy.py PATH fix (real root cause of the long-standing "/ap/arm_motors and
# /ap/mode_switch appear in the ROS2 graph but every call hangs forever" symptom
# investigated at length across Tasks 11/12 and a dedicated session). Root cause,
# confirmed live: ardupilot_sitl's launch file spawns `mavproxy.py` as a companion
# process that connects to ArduPilot's primary MAVLink serial port (SERIAL0, TCP
# 5760) -- ardurover's own "Waiting for connection ...." message (from
# AP_HAL_SITL/UARTDriver.cpp, NOT a Gazebo/FDM message despite its position in the
# log right before Gazebo-related lines) is this SERIAL0 wait, not a physics-link
# wait. Confirmed via source: this project's "rover"/"copter" SITL model
# (SIM_Rover/etc.) inherits ArduPilot's own self-contained `Aircraft` base class
# and does NOT need Gazebo's ArduPilotPlugin FDM socket at all for its own basic
# operation -- ArduPilotPlugin's fdm_port_in exists for compatibility but is
# unused by this project's real control path (Task 4/5's own comments already
# document that actual vehicle actuation flows through the DDS cmd_vel bridge,
# bypassing ArduPilotPlugin's servo channels entirely). When mavproxy.py isn't on
# PATH (real, installed at ~/.local/bin/mavproxy.py, but not reliably resolved by
# every shell that invokes `ros2 launch`, e.g. non-login/non-interactive shells
# that don't source ~/.bashrc), `exec mavproxy.py` fails silently (exit 127, "not
# found") with no serial client ever connecting -- ardurover then never gets far
# enough in its own startup for AP_DDS to complete its ping-based initialization
# ("AP: DDS: No ping response, exiting"), so micro_ros_agent's statically-advertised
# ROS2 services (/ap/arm_motors, /ap/mode_switch) are graph-visible but have no live
# server behind them, hanging every call indefinitely. Exporting PATH here removes
# the dependency on whichever shell happens to invoke `ros2 launch` already having
# ~/.local/bin set correctly.
export PATH=$HOME/.local/bin:$PATH

# microxrceddsgen (vendored Micro-XRCE-DDS-Gen at repo root) -- ardupilot_sitl's
# CMake configure step shells out to this tool by bare name and fails
# ("Could not find the program ['microxrceddsgen']") if it isn't on PATH.
export PATH=/home/parvu/CaveX-Explorer-Pro/Micro-XRCE-DDS-Gen/scripts:$PATH

# WSL GPU passthrough for Gazebo's Ogre2 renderer. Without any override,
# Mesa's EGL device enumeration only ever finds EGL_MESA_device_software
# (llvmpipe -- confirmed via ~/.gz/rendering/ogre2.log's own GL_RENDERER
# line reading "llvmpipe (LLVM ..., 256 bits)"), even though /dev/dxg
# (WSL's real DirectX-GPU passthrough device) is present and working --
# this environment has no /dev/dri/renderD* node, which is the path Mesa's
# EGL device enumeration normally uses to prefer a hardware driver over
# software. Setting GALLIUM_DRIVER=d3d12 and MESA_LOADER_DRIVER_OVERRIDE=d3d12
# does force Mesa's real d3d12 Gallium driver (already installed:
# /usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so) -- confirmed live on a small
# bundled demo world (ardupilot_gazebo/worlds/iris_runway.sdf): GL_RENDERER
# then reads "D3D12 (NVIDIA GeForce GTX 1050 Ti)" and `nvidia-smi` shows
# real non-zero GPU utilization/VRAM while `gz sim` runs, instead of
# 0%/0MiB.
#
# Re-enabled again, per explicit request -- known intermittent (crashed
# twice, ran clean twice, no fully confirmed root cause -- see this
# session's git history for the "system overload" theory that didn't
# fully hold up). If it crashes again: check `uptime`/`ps aux` for
# runaway processes first (confirmed contributor at least once), and if
# none are found, that's real evidence for a genuine D3D12/WSL timing
# issue under this specific heavy world, not yet root-caused.
export GALLIUM_DRIVER=d3d12
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
