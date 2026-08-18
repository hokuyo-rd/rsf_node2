# rsf_node2 - ROS 2 driver for the Hokuyo RSF-X001 localization sensor.
# SPDX-License-Identifier: Apache-2.0
#
# Two build modes:
#
#   * inside a ROS 2 workspace, this builds rsf_library plus the ROS 2 node;
#   * outside one (a plain Visual Studio or GCC build), ament_cmake is absent
#     and only rsf_library and its samples are built:
#
#       cmake -S . -B build -G "Visual Studio 17 2022" -A x64
#       cmake --build build --config Release

cmake_minimum_required(VERSION 3.14)
project(rsf_node2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ******************** Core library ********************
#
# The protocol implementation is ROS free and lives in its own subdirectory, so
# it can also be used from a ROS 1 node or a non-ROS application.

set(RSF_BUILD_EXAMPLES ON CACHE BOOL "" FORCE)
set(RSF_INSTALL OFF CACHE BOOL "" FORCE)

find_package(ament_cmake QUIET)
if(NOT ament_cmake_FOUND)
  message(STATUS
    "ament_cmake was not found: building rsf_library and its samples only, without the ROS node. "
    "Source a ROS 2 setup script to build the full package.")
  enable_testing()
  add_subdirectory(rsf_library)
  return()
endif()

set(RSF_BUILD_TESTS OFF CACHE BOOL "" FORCE)  # driven by BUILD_TESTING below
add_subdirectory(rsf_library)

# ******************** ROS 2 dependencies ********************

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(ROS_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nmea_msgs
  diagnostic_msgs
  tf2_ros)

# ******************** Driver node ********************

add_executable(rsf_node src/rsf_node.cpp)
target_include_directories(rsf_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rsf_node PRIVATE rsf_library)
ament_target_dependencies(rsf_node ${ROS_DEPENDENCIES})

# ******************** Command sender ********************

add_executable(send_command src/send_command.cpp)
target_include_directories(send_command PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(send_command PRIVATE rsf_library)
ament_target_dependencies(send_command rclcpp std_msgs)

# ******************** Install ********************

install(TARGETS rsf_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# rsf_library/include/rsf/ and include/rsf_ros/ are both exported: a ROS 1
# package can reuse them unchanged.
install(DIRECTORY rsf_library/include/ DESTINATION include)
install(DIRECTORY include/ DESTINATION include)

install(TARGETS
  rsf_node
  send_command
  rsf_driver
  rsf_mock_sensor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include)
ament_export_libraries(rsf_library)

# ******************** Tests ********************

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # The protocol tests, built from the subdirectory sources.
  add_executable(rsf_library_tests rsf_library/tests/test_rsf_library.cpp)
  target_link_libraries(rsf_library_tests PRIVATE rsf_library)
  add_test(NAME rsf_library_tests COMMAND rsf_library_tests)

  # The message conversions, checked against stand-in message structs so that
  # no ROS message packages are needed and both the ROS 1 and ROS 2 timestamp
  # layouts get exercised.
  add_executable(test_conversions test/test_conversions.cpp)
  target_include_directories(test_conversions PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_conversions PRIVATE rsf_library)
  add_test(NAME test_conversions COMMAND test_conversions)
endif()

ament_package()
