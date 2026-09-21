cmake_minimum_required(VERSION 3.16)
project(evergreenslam_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Installed outside every ROS prefix (build-pixi/install), so the generated libraries and the
# Python extension modules need the install lib directory on their own rpath.
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
# Link with the install rpath from the start: rewriting it at install time trips over the rpath
# the conda toolchain already adds when the install prefix is the env itself.
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/SubmapEntry.msg
  msg/SubmapList.msg
  msg/SubmapTexture.msg
  srv/SubmapQuery.srv
  DEPENDENCIES std_msgs geometry_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
