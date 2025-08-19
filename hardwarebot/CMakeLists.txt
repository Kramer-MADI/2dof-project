cmake_minimum_required(VERSION 3.5)
project(hardwarebot)

# ─────────── language levels ───────────
set(CMAKE_C_STANDARD  99  CACHE STRING "" FORCE)
set(CMAKE_CXX_STANDARD 14 CACHE STRING "" FORCE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─────────── dependencies ───────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib          REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)         # ← NEW (Asio needs Boost::system)

# ─────────── ros2_control hardware plugin ───────────
add_library(hardwarebot_system SHARED
  hardware/hardwarebot_system.cpp
)

target_include_directories(hardwarebot_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware>
  $<INSTALL_INTERFACE:include/hardware>
  ${Boost_INCLUDE_DIRS}                                 # ← NEW
)

ament_target_dependencies(hardwarebot_system
  rclcpp
  hardware_interface
  pluginlib
)

target_link_libraries(hardwarebot_system
  Boost::system                                         # ← NEW
)

# Register the plugin with pluginlib
pluginlib_export_plugin_description_file(
  hardware_interface hardware/hardwarebot_system_plugin.xml
)

# ─────────── installation ───────────
install(TARGETS hardwarebot_system
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY hardware/
  DESTINATION include/hardware
  FILES_MATCHING PATTERN "*.hpp"
)

# meshes, launch files, xacros, …
install(
  DIRECTORY config description launch meshes moveit worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()