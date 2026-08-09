find_package(Threads REQUIRED)

# Where index.html is read from at run time. The in tree path is the default because this is a
# debug tool; WebDebugSink::Option::www_dir overrides it for an installed layout.
set(EVERGREENSLAM_WEBUI_WWW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/www"
    CACHE PATH "Directory served by the webui")

add_library(evergreenslam_webui STATIC src/web_debug_sink.cc)
target_include_directories(evergreenslam_webui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
# SYSTEM so the vendored header's warnings are not ours to fix.
target_include_directories(evergreenslam_webui SYSTEM PRIVATE
                           "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
target_compile_definitions(evergreenslam_webui PUBLIC
                           EVERGREENSLAM_WEBUI_WWW_DIR="${EVERGREENSLAM_WEBUI_WWW_DIR}")
target_link_libraries(evergreenslam_webui PUBLIC evergreenslam Threads::Threads)
target_compile_options(evergreenslam_webui PRIVATE -Wall -Wextra -Wno-unused-parameter)
