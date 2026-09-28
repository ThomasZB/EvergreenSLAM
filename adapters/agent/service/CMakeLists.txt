# Static agent service. A parent adds this directory after the evergreenslam target exists.
get_filename_component(EVERGREENSLAM_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# memory/README.md is the contract the agent reads; compiled in so it cannot drift from the source.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/../memory_README.md" EVERGREENSLAM_MEMORY_README)
configure_file(memory_readme.cc.in "${CMAKE_CURRENT_BINARY_DIR}/memory_readme.cc" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${CMAKE_CURRENT_SOURCE_DIR}/../memory_README.md")

add_library(evergreenslam_agent STATIC
            agent_service.cc
            fs_endpoints.cc
            fs_sandbox.cc
            graph_reads.cc
            gray_png.cc
            here_tracker.cc
            http_reply.cc
            json_writer.cc
            map_endpoints.cc
            maps_endpoints.cc
            place_endpoints.cc
            place_store.cc
            plan_report.cc
            service_context.cc
            session_endpoints.cc
            snapshot_exporter.cc
            "${CMAKE_CURRENT_BINARY_DIR}/memory_readme.cc")
# Includes read "service/agent_service.h", relative to adapters/agent.
target_include_directories(evergreenslam_agent PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
# SYSTEM so the vendored headers' warnings are not ours to fix.
target_include_directories(evergreenslam_agent SYSTEM PRIVATE
                           "${EVERGREENSLAM_REPO_ROOT}/third_party")
target_link_libraries(evergreenslam_agent PUBLIC evergreenslam yaml-cpp::yaml-cpp Threads::Threads)
target_compile_options(evergreenslam_agent PRIVATE -Wall -Wextra -Wno-unused-parameter)

# GET /view renders through service/render when that directory is present.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/render/CMakeLists.txt")
  add_subdirectory(render)
  target_link_libraries(evergreenslam_agent PUBLIC evergreenslam_agent_render)
endif()
