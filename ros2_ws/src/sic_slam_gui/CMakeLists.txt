cmake_minimum_required(VERSION 3.10)
project(sic_slam_gui)

find_package(ament_cmake REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)

cmake_policy(SET CMP0100 NEW)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

qt5_add_resources(INFOLABEL_RESOURCES src/InfoLabel.qrc)

# InfoLabel.hh listed explicitly (not just #included from the .cc): this
# Qt/CMake version's AUTOMOC otherwise silently skips it entirely --
# confirmed live (mocs_compilation.cpp came out empty, "no files found
# that require moc"), which is what caused the InfoLabel vtable/signal
# link errors below to be chased first as a red herring.
add_library(InfoLabel SHARED
  src/InfoLabel.cc
  src/InfoLabel.hh
  ${INFOLABEL_RESOURCES}
)
target_link_libraries(InfoLabel PRIVATE
  gz-gui8::gz-gui8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  Qt5::Core
  Qt5::Quick
  Qt5::QuickControls2
)

# Installed flat into lib/sic_slam_gui, added to GZ_GUI_PLUGIN_PATH by
# sic_slam's sim_launch.py (the actual consumer -- this package only
# builds the plugin, sic_slam wires it into a world's <gui> section).
install(TARGETS InfoLabel DESTINATION lib/${PROJECT_NAME})

ament_package()
