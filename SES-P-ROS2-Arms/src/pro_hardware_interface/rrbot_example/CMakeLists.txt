cmake_minimum_required(VERSION 3.8)
project(rrbot_example)

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

install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/rrbot_example
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/rrbot_example
)

# Export package
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
