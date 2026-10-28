cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# Enforce strict C++ compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp vision_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp vision_msgs)

# Tell CMake where to install the compiled binaries so ROS 2 can find them
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# This must always be the last line in a ROS 2 CMakeLists.txt
ament_package()