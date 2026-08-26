cmake_minimum_required(VERSION 3.8)
project(cavex_gtsam_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(gtsam_slam_core
  src/scan_registration.cpp
)
target_link_libraries(gtsam_slam_core Eigen3::Eigen)
target_include_directories(gtsam_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS gtsam_slam_core
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

add_executable(gtsam_slam_node src/gtsam_slam_node.cpp)
target_link_libraries(gtsam_slam_node gtsam_slam_core gtsam)
ament_target_dependencies(gtsam_slam_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)
install(TARGETS gtsam_slam_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_registration test/test_scan_registration.cpp)
  target_link_libraries(test_scan_registration gtsam_slam_core)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
