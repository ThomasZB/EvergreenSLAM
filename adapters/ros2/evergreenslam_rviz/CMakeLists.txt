cmake_minimum_required(VERSION 3.16)
project(evergreenslam_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra)

# RoboStack's rclcpp and rviz export the message packages' Python typesupport libraries in their
# link interface; those resolve Python symbols lazily and abort at dyld time in a non-Python
# process. Nothing here references them, so let the linker drop them.
if(APPLE)
  add_link_options(-Wl,-dead_strip_dylibs)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(evergreenslam_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Eigen3 REQUIRED)

# core/src is header-only here: utils/transform/transform.h and nothing else. Resolve this
# directory before walking up, in two steps: colcon symlinks the package into <ws>/src, and
# get_filename_component collapses ".." lexically before resolving any symlink.
get_filename_component(EVERGREENSLAM_PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(EVERGREENSLAM_ROOT "${EVERGREENSLAM_PACKAGE_DIR}/../../.." REALPATH)
if(NOT EXISTS "${EVERGREENSLAM_ROOT}/core/src/utils/transform/transform.h")
  message(FATAL_ERROR "core not found at ${EVERGREENSLAM_ROOT}/core; set EVERGREENSLAM_ROOT")
endif()

# The plugin and the msgs it links are installed outside every ROS prefix (build-pixi/install),
# so the loader has to be told where the typesupport library went.
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
# Link with the install rpath from the start: rewriting it at install time trips over the rpath
# the conda toolchain already adds when the install prefix is the env itself.
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

add_library(${PROJECT_NAME} SHARED
  src/drawable_submap.cc
  src/ogre_slice.cc
  src/submaps_display.cc
  include/evergreenslam_rviz/drawable_submap.h
  include/evergreenslam_rviz/submaps_display.h)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EVERGREENSLAM_ROOT}/core/src)

# Per-package C++ typesupport target rather than ament_target_dependencies or ${pkg}_TARGETS: both
# also link the packages' Python typesupport libraries, which on RoboStack macOS fail to load
# outside a Python interpreter (unresolved _PyExc_* symbols at dyld time).
target_link_libraries(${PROJECT_NAME} PRIVATE
  evergreenslam_msgs::evergreenslam_msgs__rosidl_typesupport_cpp
  geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  pluginlib::pluginlib
  Qt5::Widgets
  Eigen3::Eigen)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY ogre_media DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
