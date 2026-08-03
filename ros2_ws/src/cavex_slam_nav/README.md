# CaveX SLAM & Navigation Workspace

## SLAM/PTAM Alternative Analysis
While PTAM (Parallel Tracking and Mapping) is a lightweight visual SLAM method for tracking points, it is outdated and lacks robust loop closure, dense 3D reconstruction, and multi-sensor fusion. 

For the CaveX hybrid drone navigating complex environments (dry caves, flooded zones, and vertical shafts), **RTAB-Map (Real-Time Appearance-Based Mapping)** is the vastly superior alternative:

### Why RTAB-Map over PTAM/ORB-SLAM?
1. **Multi-Modal Sensor Fusion**: RTAB-Map natively fuses RGB-D cameras (Stereo), 3D LiDAR (for the vertical shaft), and odometry (Spot's kinematics), whereas PTAM is strictly mono/stereo visual.
2. **Dense 3D Volumetric Mapping**: RTAB-Map outputs point clouds, octomaps, and meshes seamlessly which matches our UI's requirement for 3D Shaded Mesh Reconstruction and PCD exporting.
3. **Robust Loop Closure**: By using a memory management approach, RTAB-Map performs large-scale mapping without memory exhaustion, perfect for long cave explorations.
4. **ROS 2 & Nav2 Native**: RTAB-Map is officially supported in ROS 2 Humble/Iron and provides a direct 2D/3D costmap for Nav2.

## Build & Run Instructions
See the top-level [README.md](../../../README.md) for full instructions
(Gazebo simulation, driving the robot, ATE evaluation, web dashboard). Quick
build:
```bash
cd ros2_ws   # repo root
colcon build --symlink-install --packages-select cavex_slam_nav
source install/setup.bash
ros2 launch cavex_slam_nav gazebo_sim.launch.py   # terminal 1
ros2 launch cavex_slam_nav rtabmap_nav.launch.py  # terminal 2
```
