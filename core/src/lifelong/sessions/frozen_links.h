/**
 * @file frozen_links.h
 * @author hang chen (chen@hang.plus)
 * @brief How many constraints tie each submap of a session to the frozen layer.
 * @version 0.1
 * @date 2026-09-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_SESSIONS_FROZEN_LINKS_H_
#define EVERGREENSLAM_LIFELONG_SESSIONS_FROZEN_LINKS_H_

#include <map>

#include "lifelong/pose_graph_data.h"

namespace evergreenslam::lifelong {

// A frozen link is a constraint with exactly one endpoint in a frozen session whose other
// endpoint is the submap itself or one of its nodes; a PRIOR counts too. Every submap of the
// session gets an entry, possibly 0.
std::map<SubmapId, int> CountFrozenLinks(const PoseGraphData& graph, SessionId id);

// Graph-side on purpose: Optimization::IsSessionAnchoredToFrozen reads the gauge set of the last
// solve, which knows nothing of a session or an edge added since.
bool HasFrozenLink(const PoseGraphData& graph, SessionId id);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_SESSIONS_FROZEN_LINKS_H_
