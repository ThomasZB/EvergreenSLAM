# Static renderer for GET /view. A parent adds this directory after the evergreenslam target exists.
get_filename_component(EVERGREENSLAM_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)

add_library(evergreenslam_agent_render STATIC
            bitmap_font.cc
            canvas.cc
            grid_overlay.cc
            png_writer.cc
            renderer.cc
            view_frame.cc)
# Includes read "service/render/renderer.h", relative to adapters/agent.
target_include_directories(evergreenslam_agent_render PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../..")
# SYSTEM so the vendored header's warnings are not ours to fix.
target_include_directories(evergreenslam_agent_render SYSTEM PRIVATE
                           "${EVERGREENSLAM_REPO_ROOT}/third_party")
target_link_libraries(evergreenslam_agent_render PUBLIC evergreenslam)
target_compile_options(evergreenslam_agent_render PRIVATE -Wall -Wextra -Wno-unused-parameter)
