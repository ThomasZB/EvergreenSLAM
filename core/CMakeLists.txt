cmake_minimum_required(VERSION 3.16)
# C as well: conda-forge Ceres pulls SuiteSparse_config, whose config demands OpenMP for C.
project(evergreenslam LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(EVERGREENSLAM_BUILD_TESTS "Build the unit tests" ON)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Protobuf REQUIRED)

# Apple clang ships without the OpenMP runtime, so point the module at the
# Homebrew libomp before giving up on it. Conda-forge clang (pixi) links its own
# llvm-openmp and must not be steered at Homebrew's. real_time_correlative_scan_matcher.cc
# includes <omp.h> unconditionally, so this is required, not an optimisation.
if(APPLE AND CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  find_program(BREW_EXECUTABLE brew)
  if(BREW_EXECUTABLE)
    execute_process(COMMAND ${BREW_EXECUTABLE} --prefix libomp
                    OUTPUT_VARIABLE LIBOMP_PREFIX
                    OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
  endif()
  if(LIBOMP_PREFIX AND EXISTS "${LIBOMP_PREFIX}/lib/libomp.dylib")
    foreach(lang C CXX)
      set(OpenMP_${lang}_FLAGS "-Xpreprocessor -fopenmp -I${LIBOMP_PREFIX}/include")
      set(OpenMP_${lang}_LIB_NAMES "omp")
    endforeach()
    set(OpenMP_omp_LIBRARY "${LIBOMP_PREFIX}/lib/libomp.dylib")
  endif()
endif()
find_package(OpenMP REQUIRED)

file(GLOB_RECURSE EVERGREENSLAM_SOURCES CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")

# protoc by hand rather than protobuf_generate_cpp: the module's output layout depends on the
# protobuf version, and the generated header has to be reachable as
# "lifelong/map_manager/proto/map.pb.h" like every other include in the tree.
set(EVERGREENSLAM_PROTO_DIR "${CMAKE_CURRENT_BINARY_DIR}/proto_gen")
set(EVERGREENSLAM_PROTO_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/lifelong/map_manager/proto/map.proto")
set(EVERGREENSLAM_PROTO_SRC
    "${EVERGREENSLAM_PROTO_DIR}/lifelong/map_manager/proto/map.pb.cc")
set(EVERGREENSLAM_PROTO_HDR
    "${EVERGREENSLAM_PROTO_DIR}/lifelong/map_manager/proto/map.pb.h")
add_custom_command(
  OUTPUT ${EVERGREENSLAM_PROTO_SRC} ${EVERGREENSLAM_PROTO_HDR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${EVERGREENSLAM_PROTO_DIR}
  COMMAND protobuf::protoc --cpp_out=${EVERGREENSLAM_PROTO_DIR}
          -I "${CMAKE_CURRENT_SOURCE_DIR}/src" ${EVERGREENSLAM_PROTO_FILE}
  DEPENDS ${EVERGREENSLAM_PROTO_FILE} protobuf::protoc
  COMMENT "protoc lifelong/map_manager/proto/map.proto"
  VERBATIM)

add_library(evergreenslam STATIC ${EVERGREENSLAM_SOURCES} ${EVERGREENSLAM_PROTO_SRC})
target_include_directories(evergreenslam PUBLIC
                           "${CMAKE_CURRENT_SOURCE_DIR}/src")
# PRIVATE: the generated header is an implementation detail of map_manager (decision 3), so
# nothing outside this library may reach it.
target_include_directories(evergreenslam PRIVATE "${EVERGREENSLAM_PROTO_DIR}")
target_link_libraries(evergreenslam PUBLIC
                      Eigen3::Eigen
                      Ceres::ceres
                      glog::glog
                      yaml-cpp::yaml-cpp
                      OpenMP::OpenMP_CXX)
target_link_libraries(evergreenslam PRIVATE protobuf::libprotobuf)
target_compile_options(evergreenslam PRIVATE -Wall -Wextra -Wno-unused-parameter)
set_source_files_properties(${EVERGREENSLAM_PROTO_SRC} PROPERTIES COMPILE_OPTIONS "-w")

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
