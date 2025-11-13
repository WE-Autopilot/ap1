cmake_minimum_required(VERSION 3.8)
project(ap1_launch)

find_package(ament_cmake REQUIRED)

# Install the launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
