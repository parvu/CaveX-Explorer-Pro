cmake_minimum_required(VERSION 3.8)
project(cavex_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common search kdtree segmentation)

add_executable(sic_slam_node
  src/sic_slam_node.cpp
  src/instance_clustering.cpp
)
target_include_directories(sic_slam_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(sic_slam_node ${PCL_LIBRARIES})
ament_target_dependencies(sic_slam_node
  rclcpp sensor_msgs vision_msgs geometry_msgs tf2_ros tf2_eigen
  cv_bridge image_geometry pcl_conversions)

install(TARGETS sic_slam_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
