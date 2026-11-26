cmake_minimum_required(VERSION 3.8)
project(ap1_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ap1_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

add_executable(planner_node
  src/behaviours.cpp
  src/event_generator.cpp
  src/frames.cpp
  src/fsm.cpp
  src/main.cpp
  src/math_utils.cpp
  src/planner_node.cpp
  src/waypoint_utils.cpp
  util/osm_parser.cpp
)

target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/util>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(planner_node PUBLIC cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  planner_node
  rclcpp
  std_msgs
  geometry_msgs
  ap1_msgs
)
target_link_libraries(planner_node
  tinyxml2::tinyxml2
)

install(TARGETS
  planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# copy the config dir over as well
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # disable linting for now bc uncrustify style checks conflict with my clang configs
  # use the clang-format file to format your code
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planning_dummy test/test_planning_dummy.cpp)

  ament_add_gtest(test_osm_parser
    test/test_osm_parser.cpp
    util/osm_parser.cpp
  )
  target_compile_features(test_osm_parser PRIVATE cxx_std_17)
  target_include_directories(test_osm_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/util
  )
  target_link_libraries(test_osm_parser
    tinyxml2::tinyxml2
  )
  target_compile_definitions(test_osm_parser PRIVATE
    TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures"
  )
endif()

ament_package()