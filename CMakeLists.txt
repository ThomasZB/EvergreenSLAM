cmake_minimum_required(VERSION 3.16)
project(EvergreenSLAM LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Must be called from the top level directory, otherwise ctest sees no tests
# from the subdirectories.
enable_testing()

option(EVERGREENSLAM_WITH_WEBUI "Build the browser debug viewer" ON)

# core/ is the portable library and knows nothing about ROS or any adapter.
# Everything else (adapters/, apps/) is added here as it appears.
add_subdirectory(core)

if(EVERGREENSLAM_WITH_WEBUI)
  add_subdirectory(apps/webui)
endif()
