cmake_minimum_required(VERSION 3.16)
# C as well: conda-forge Ceres pulls SuiteSparse_config, whose config demands OpenMP for C.
project(evergreenslam_ros LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# RoboStack's rclcpp/tf2_ros export the message packages' Python typesupport libraries in their
# link interface; those resolve Python symbols lazily and abort at dyld time in a non-Python
# process. Nothing here references them, so let the linker drop them (conda-forge's own default).
if(APPLE)
  add_link_options(-Wl,-dead_strip_dylibs)
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
find_package(visualization_msgs REQUIRED)
find_package(evergreenslam_msgs REQUIRED)

# core has no install or export rules, so it is built in place rather than found. Its tests need
# GTest, which the ROS image has no reason to carry.
# Resolve this directory before walking up, in two steps: colcon symlinks the package into
# <ws>/src, and get_filename_component collapses ".." lexically before resolving any symlink, so
# a one-step "${CMAKE_CURRENT_LIST_DIR}/../../.." REALPATH lands in the workspace, not the repo.
get_filename_component(EVERGREENSLAM_PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(EVERGREENSLAM_ROOT "${EVERGREENSLAM_PACKAGE_DIR}/../../.." REALPATH)
if(NOT EXISTS "${EVERGREENSLAM_ROOT}/core/CMakeLists.txt")
  message(FATAL_ERROR "core not found at ${EVERGREENSLAM_ROOT}/core; set EVERGREENSLAM_ROOT")
endif()
set(EVERGREENSLAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${EVERGREENSLAM_ROOT}/core ${CMAKE_CURRENT_BINARY_DIR}/core)

option(EVERGREENSLAM_WITH_WEBUI "Build the browser debug viewer" ON)
if(EVERGREENSLAM_WITH_WEBUI)
  add_subdirectory(${EVERGREENSLAM_ROOT}/apps/webui ${CMAKE_CURRENT_BINARY_DIR}/webui)
endif()

option(EVERGREENSLAM_WITH_AGENT "Build the agent HTTP service (adapters/agent/API.md)" ON)
if(EVERGREENSLAM_WITH_AGENT)
  add_subdirectory(${EVERGREENSLAM_ROOT}/adapters/agent/service ${CMAKE_CURRENT_BINARY_DIR}/agent)
endif()

# Per-package C++ typesupport targets rather than ament_target_dependencies or ${pkg}_TARGETS:
# both also link the packages' Python typesupport libraries, which on RoboStack macOS fail to
# load outside a Python interpreter (unresolved _PyExc_* symbols at dyld time).
set(ROS_TARGETS rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros rosbag2_cpp::rosbag2_cpp)
foreach(pkg sensor_msgs nav_msgs geometry_msgs tf2_msgs visualization_msgs evergreenslam_msgs)
  list(APPEND ROS_TARGETS ${pkg}::${pkg}__rosidl_typesupport_cpp)
endforeach()

add_library(evergreenslam_ros_common src/laser_scan_converter.cc src/pose_graph_publisher.cc)
target_include_directories(evergreenslam_ros_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(evergreenslam_ros_common PUBLIC evergreenslam ${ROS_TARGETS})

add_executable(evergreenslam_node src/evergreenslam_node.cc)
target_link_libraries(evergreenslam_node PRIVATE evergreenslam_ros_common)

add_executable(evergreenslam_bag src/evergreenslam_bag_main.cc)
target_link_libraries(evergreenslam_bag PRIVATE evergreenslam_ros_common)

if(EVERGREENSLAM_WITH_WEBUI)
  foreach(target evergreenslam_node evergreenslam_bag)
    target_link_libraries(${target} PRIVATE evergreenslam_webui)
    target_compile_definitions(${target} PRIVATE EVERGREENSLAM_WITH_WEBUI)
  endforeach()
endif()

if(EVERGREENSLAM_WITH_AGENT)
  add_library(evergreenslam_agent_host src/agent_host.cc)
  target_include_directories(evergreenslam_agent_host PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(evergreenslam_agent_host PUBLIC evergreenslam_agent)
  foreach(target evergreenslam_node evergreenslam_bag)
    target_link_libraries(${target} PRIVATE evergreenslam_agent_host)
    target_compile_definitions(${target} PRIVATE EVERGREENSLAM_WITH_AGENT)
  endforeach()
endif()

# Plain GTest rather than ament_add_gtest: the latter compiles ament's vendored gtest sources
# against whatever gtest headers are on the include path, which breaks with a newer system gtest
# (RoboStack ships 1.17). Both the Docker image and the pixi env carry a GTest CMake package.
if(BUILD_TESTING)
  enable_testing()
  find_package(GTest REQUIRED)
  add_executable(laser_scan_converter_test test/laser_scan_converter_test.cc)
  target_link_libraries(laser_scan_converter_test evergreenslam_ros_common GTest::gtest_main)
  add_test(NAME laser_scan_converter_test COMMAND laser_scan_converter_test)
endif()

install(TARGETS evergreenslam_node evergreenslam_bag
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
