# Static agent service. A parent adds this directory after the evergreenslam target exists.
get_filename_component(EVERGREENSLAM_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# memory/README.md is the contract the agent reads; compiled in so it cannot drift from the source.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/../memory_README.md" EVERGREENSLAM_MEMORY_README)
configure_file(memory/memory_readme.cc.in "${CMAKE_CURRENT_BINARY_DIR}/memory_readme.cc" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${CMAKE_CURRENT_SOURCE_DIR}/../memory_README.md")

add_library(evergreenslam_agent STATIC
            agent_service.cc
            endpoints/fs_endpoints.cc
            endpoints/maps_endpoints.cc
            endpoints/place_endpoints.cc
            endpoints/session_endpoints.cc
            endpoints/view_endpoints.cc
            endpoints/zones_endpoints.cc
            graph/graph_reads.cc
            graph/gray_png.cc
            graph/here_tracker.cc
            graph/match_score_average.cc
            graph/plan_report.cc
            graph/snapshot_exporter.cc
            http/http_reply.cc
            http/json_writer.cc
            http/service_context.cc
            memory/fs_sandbox.cc
            memory/place_store.cc
            memory/zone_store.cc
            "${CMAKE_CURRENT_BINARY_DIR}/memory_readme.cc")
# Includes read "service/http/endpoints.h", relative to adapters/agent.
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
