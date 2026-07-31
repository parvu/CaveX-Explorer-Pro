cmake_minimum_required(VERSION 3.8)
project(cavex_slam_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files, URDFs, Worlds, and Models
install(DIRECTORY
  launch
  urdf
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python nodes
install(PROGRAMS
  cavex_slam_nav/drone_commander.py
  cavex_slam_nav/base_station_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
