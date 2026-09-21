cmake_minimum_required(VERSION 3.8)
project(ap1_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ap1_msgs REQUIRED)

add_executable(control_node 
    src/main.cpp
    src/control_node.cpp
    src/ackermann_controller.cpp
)
add_executable(synthetic_actuation_feedback_node
    src/synthetic_actuation_feedback_node.cpp
)
target_include_directories(control_node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_include_directories(synthetic_actuation_feedback_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  control_node 
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "ap1_msgs"
)
ament_target_dependencies(
  synthetic_actuation_feedback_node
  "rclcpp"
  "ap1_msgs"
)

install(TARGETS 
  control_node 
  synthetic_actuation_feedback_node
  DESTINATION lib/${PROJECT_NAME}
)

# copy the config dir over as well
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # no auto linting for now
endif()

ament_package()
