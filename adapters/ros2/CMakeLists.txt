cmake_minimum_required(VERSION 3.16)
project(evergreenslam_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# core has no install or export rules, so it is built in place rather than found. Its tests need
# GTest, which the ROS image has no reason to carry.
# Resolve this directory before walking up, in two steps: colcon symlinks the package into
# <ws>/src, and get_filename_component collapses ".." lexically before resolving any symlink, so
# a one-step "${CMAKE_CURRENT_LIST_DIR}/../.." REALPATH lands in the workspace, not the repo.
get_filename_component(EVERGREENSLAM_PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(EVERGREENSLAM_ROOT "${EVERGREENSLAM_PACKAGE_DIR}/../.." REALPATH)
if(NOT EXISTS "${EVERGREENSLAM_ROOT}/core/CMakeLists.txt")
  message(FATAL_ERROR "core not found at ${EVERGREENSLAM_ROOT}/core; set EVERGREENSLAM_ROOT")
endif()
set(EVERGREENSLAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${EVERGREENSLAM_ROOT}/core ${CMAKE_CURRENT_BINARY_DIR}/core)

option(EVERGREENSLAM_WITH_WEBUI "Build the browser debug viewer" ON)
if(EVERGREENSLAM_WITH_WEBUI)
  add_subdirectory(${EVERGREENSLAM_ROOT}/apps/webui ${CMAKE_CURRENT_BINARY_DIR}/webui)
endif()

set(ROS_DEPENDENCIES rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_msgs tf2_ros rosbag2_cpp)

add_library(evergreenslam_ros_common src/laser_scan_converter.cc)
target_include_directories(evergreenslam_ros_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(evergreenslam_ros_common PUBLIC evergreenslam)
ament_target_dependencies(evergreenslam_ros_common PUBLIC ${ROS_DEPENDENCIES})

add_executable(laser_odometry_node src/laser_odometry_node.cc)
target_link_libraries(laser_odometry_node PRIVATE evergreenslam_ros_common)
ament_target_dependencies(laser_odometry_node PUBLIC ${ROS_DEPENDENCIES})

add_executable(bag_laser_odometry src/bag_laser_odometry_main.cc)
target_link_libraries(bag_laser_odometry PRIVATE evergreenslam_ros_common)
ament_target_dependencies(bag_laser_odometry PUBLIC ${ROS_DEPENDENCIES})

if(EVERGREENSLAM_WITH_WEBUI)
  foreach(target laser_odometry_node bag_laser_odometry)
    target_link_libraries(${target} PRIVATE evergreenslam_webui)
    target_compile_definitions(${target} PRIVATE EVERGREENSLAM_WITH_WEBUI)
  endforeach()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(laser_scan_converter_test test/laser_scan_converter_test.cc)
  target_link_libraries(laser_scan_converter_test evergreenslam_ros_common)
endif()

install(TARGETS laser_odometry_node bag_laser_odometry
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY ${EVERGREENSLAM_ROOT}/configs
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})
if(EVERGREENSLAM_WITH_WEBUI)
  install(DIRECTORY ${EVERGREENSLAM_ROOT}/apps/webui/www
          DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
