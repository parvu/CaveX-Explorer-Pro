#!/bin/bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/parvu/CaveX-Explorer-Pro/.worktrees/cavex-tracked-blueboat-ardupilot/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/home/parvu/CaveX-Explorer-Pro/.worktrees/cavex-tracked-blueboat-ardupilot/ardupilot_gazebo/models:/home/parvu/CaveX-Explorer-Pro/.worktrees/cavex-tracked-blueboat-ardupilot/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=/home/parvu/CaveX-Explorer-Pro/.worktrees/cavex-tracked-blueboat-ardupilot/ros2_ws/src/cavex_slam_nav/models:$GZ_SIM_RESOURCE_PATH

# micro_ros_agent fix (Task 10 follow-up): micro_ros_msgs isn't a real apt/rosdep package on
# ROS2 Jazzy, so this environment vendors it as a prebuilt extract at .local-deps/ros-extract.
# micro_ros_agent's own binary RUNPATH already points there and resolves its direct dependency
# (libmicro_ros_msgs__rosidl_typesupport_cpp.so) fine -- but that library in turn NEEDs
# libmicro_ros_msgs__rosidl_generator_c.so, and per ELF semantics DT_RUNPATH on the top-level
# executable does NOT propagate to transitive (dependency-of-a-dependency) lookups. Both .so
# files sit in the exact same directory (confirmed with readelf -d), but generator_c.so has no
# RPATH/RUNPATH of its own and isn't found unless this directory is in LD_LIBRARY_PATH directly.
export LD_LIBRARY_PATH=/home/parvu/CaveX-Explorer-Pro/.worktrees/cavex-tracked-blueboat-ardupilot/.local-deps/ros-extract/opt/ros/jazzy/lib:$LD_LIBRARY_PATH
