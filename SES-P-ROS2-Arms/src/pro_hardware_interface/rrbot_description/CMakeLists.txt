cmake_minimum_required(VERSION 3.5)
project(rrbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY rrbot/urdf rrbot/rviz
  DESTINATION share/${PROJECT_NAME}/rrbot
)

ament_package()
