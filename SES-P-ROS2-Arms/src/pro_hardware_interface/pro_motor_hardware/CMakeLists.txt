cmake_minimum_required(VERSION 3.8)
project(pro_motor_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# Find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add the hardware interface library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/pro_motor_hardware.cpp
  src/pro.c
)

# Include directories
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

# Link dependencies using ament-style
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface pro_motor_hardware.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES pro_motor_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)
# Export package
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
