cmake_minimum_required(VERSION 3.8)
project(cavex_sonar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Host-agnostic acoustic physics. Deliberately has NO ROS and NO Gazebo
# dependencies so it can be unit-tested off-simulator and relocated into a
# gz-sim system plugin later without touching its physics or its tests.
add_library(sonar_acoustics src/sonar_acoustics.cpp)
target_include_directories(sonar_acoustics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS sonar_acoustics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sonar_acoustics test/test_sonar_acoustics.cpp)
  target_link_libraries(test_sonar_acoustics sonar_acoustics)
endif()

ament_package()
