cmake_minimum_required(VERSION 3.8)
project(ap1_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

# If you have post-processing nodes (e.g. voxel filtering, manifest generation)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/map_postprocessing_node.cpp")
  add_executable(map_postprocessing_node src/map_postprocessing_node.cpp)
  target_include_directories(map_postprocessing_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(map_postprocessing_node
    rclcpp
    sensor_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
  )
  target_link_libraries(map_postprocessing_node ${PCL_LIBRARIES})
  install(TARGETS map_postprocessing_node DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()