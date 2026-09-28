# Standalone build of the agent layer for its own tests:
#   cmake -S adapters/agent -B build-agent && cmake --build build-agent && ctest --test-dir build-agent
# Hosts (evergreenslam_ros) add service/ directly next to core instead of this file.
cmake_minimum_required(VERSION 3.16)
# C as well: core's Ceres pulls SuiteSparse_config, whose config demands OpenMP for C.
project(evergreenslam_agent LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

if(NOT TARGET evergreenslam)
  set(EVERGREENSLAM_BUILD_TESTS OFF)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core ${CMAKE_CURRENT_BINARY_DIR}/core)
endif()

add_subdirectory(service)

find_package(GTest REQUIRED)
enable_testing()

# service/test/ holds the service's tests; test/ mirrors service/ for the renderer's.
file(GLOB_RECURSE EVERGREENSLAM_AGENT_TEST_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc"
     "${CMAKE_CURRENT_SOURCE_DIR}/service/test/*.cc")
list(FILTER EVERGREENSLAM_AGENT_TEST_SOURCES EXCLUDE REGEX "/fake_host_main\\.cc$")
foreach(test_source ${EVERGREENSLAM_AGENT_TEST_SOURCES})
  file(RELATIVE_PATH test_relative "${CMAKE_CURRENT_SOURCE_DIR}" ${test_source})
  string(REGEX REPLACE "\\.cc$" "" test_name ${test_relative})
  string(REPLACE "/" "_" test_name ${test_name})
  add_executable(${test_name} ${test_source})
  # core/test for the simulated-laser scenarios the service tests drive.
  target_include_directories(${test_name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../core/test")
  target_include_directories(${test_name} SYSTEM PRIVATE
                             "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party")
  target_link_libraries(${test_name} PRIVATE evergreenslam_agent GTest::gtest_main)
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# The egs integration test (egs/tests/test_integration.py) spawns this with EGS_FAKE_HOST set.
add_executable(fake_host service/test/fake_host_main.cc)
target_include_directories(fake_host PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../core/test")
target_include_directories(fake_host SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party")
target_link_libraries(fake_host PRIVATE evergreenslam_agent GTest::gtest)
