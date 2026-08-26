cmake_minimum_required(VERSION 3.10)
project(cavex_tracked_vehicle_gui)

find_package(ament_cmake REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)

cmake_policy(SET CMP0100 NEW)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_add_resources(ACTIONBUTTONS_RESOURCES src/ActionButtons.qrc)

# .hh listed explicitly (not just #included from the .cc): AUTOMOC needs
# it on the target's own source list to moc it -- see sic_slam_gui's own
# InfoLabel.cc comment for the real link-error failure mode this avoids.
add_library(ActionButtons SHARED
  src/ActionButtons.cc
  src/ActionButtons.hh
  ${ACTIONBUTTONS_RESOURCES}
)
target_link_libraries(ActionButtons PRIVATE
  gz-gui8::gz-gui8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  Qt5::Core
  Qt5::Quick
  Qt5::QuickControls2
)

qt5_add_resources(MANUALCONTROL_RESOURCES src/ManualControl.qrc)

add_library(ManualControl SHARED
  src/ManualControl.cc
  src/ManualControl.hh
  ${MANUALCONTROL_RESOURCES}
)
target_link_libraries(ManualControl PRIVATE
  gz-gui8::gz-gui8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  Qt5::Core
  Qt5::Quick
  Qt5::QuickControls2
)

# Installed flat into lib/cavex_tracked_vehicle_gui, added to
# GZ_GUI_PLUGIN_PATH by gazebo_tracked_vehicle.launch.py (the actual
# consumer -- this package only builds the plugins, that launch file
# wires them into cavex_world.world's <gui> section).
install(TARGETS ActionButtons ManualControl DESTINATION lib/${PROJECT_NAME})

ament_package()
