cmake_minimum_required(VERSION 3.8)
project(cavex_tracked_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ardupilot_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files, URDFs, Worlds, config, and models
install(DIRECTORY
  launch
  urdf
  worlds
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python nodes
install(PROGRAMS
  cavex_tracked_vehicle/track_retract_control.py
  cavex_tracked_vehicle/cmd_vel_to_ardupilot.py
  cavex_tracked_vehicle/track_cmd_vel_bridge.py
  cavex_tracked_vehicle/tracked_vehicle_ground_truth_odom.py
  cavex_tracked_vehicle/run_tracked_vehicle_ate_eval.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
