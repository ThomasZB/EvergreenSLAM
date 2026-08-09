cmake_minimum_required(VERSION 3.16)
project(evergreenslam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(EVERGREENSLAM_BUILD_TESTS "Build the unit tests" ON)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)

# Apple clang ships without the OpenMP runtime, so point the module at the
# Homebrew libomp before giving up on it. real_time_correlative_scan_matcher.cc
# includes <omp.h> unconditionally, so this is required, not an optimisation.
if(APPLE)
  find_program(BREW_EXECUTABLE brew)
  if(BREW_EXECUTABLE)
    execute_process(COMMAND ${BREW_EXECUTABLE} --prefix libomp
                    OUTPUT_VARIABLE LIBOMP_PREFIX
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
  endif()
  if(LIBOMP_PREFIX AND EXISTS "${LIBOMP_PREFIX}/lib/libomp.dylib")
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I${LIBOMP_PREFIX}/include")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY "${LIBOMP_PREFIX}/lib/libomp.dylib")
  endif()
endif()
find_package(OpenMP REQUIRED)

file(GLOB_RECURSE EVERGREENSLAM_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")

add_library(evergreenslam STATIC ${EVERGREENSLAM_SOURCES})
target_include_directories(evergreenslam PUBLIC
                           "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(evergreenslam PUBLIC
                      Eigen3::Eigen
                      Ceres::ceres
                      glog::glog
                      yaml-cpp::yaml-cpp
                      OpenMP::OpenMP_CXX)
target_compile_options(evergreenslam PRIVATE -Wall -Wextra -Wno-unused-parameter)

if(EVERGREENSLAM_BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()

  file(GLOB_RECURSE EVERGREENSLAM_TEST_SOURCES CONFIGURE_DEPENDS
       "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc")

  # One binary per test file: a crash in one module cannot take the rest of
  # the suite with it, and the failing target names the module directly.
  foreach(test_source ${EVERGREENSLAM_TEST_SOURCES})
    file(RELATIVE_PATH test_relative "${CMAKE_CURRENT_SOURCE_DIR}/test"
         ${test_source})
    string(REGEX REPLACE "\\.cc$" "" test_name ${test_relative})
    string(REPLACE "/" "_" test_name ${test_name})
    add_executable(${test_name} ${test_source})
    target_link_libraries(${test_name} PRIVATE evergreenslam GTest::gtest_main)
    target_compile_definitions(${test_name} PRIVATE
        EVERGREENSLAM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../configs")
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
