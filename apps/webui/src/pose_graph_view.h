/**
 * @file pose_graph_view.h
 * @author hang chen (chen@hang.plus)
 * @brief The lifelong backend's state as one JSON document for the browser.
 * @version 0.1
 * @date 2026-08-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_APPS_WEBUI_POSE_GRAPH_VIEW_H_
#define EVERGREENSLAM_APPS_WEBUI_POSE_GRAPH_VIEW_H_

#include <string>

namespace evergreenslam::lifelong {
class PoseGraph;
}  // namespace evergreenslam::lifelong

namespace evergreenslam::webui {

// Reads the graph, so it must run on a backend task; WebDebugSink::PublishPoseGraph arranges that.
std::string SerializePoseGraph(const lifelong::PoseGraph& graph);

}  // namespace evergreenslam::webui

#endif  // EVERGREENSLAM_APPS_WEBUI_POSE_GRAPH_VIEW_H_
