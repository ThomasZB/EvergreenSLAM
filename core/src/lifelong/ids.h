/**
 * @file ids.h
 * @author hang chen (chen@hang.plus)
 * @brief Node, submap and session identifiers shared by the graph and session data.
 * @version 0.1
 * @date 2026-08-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_IDS_H_
#define EVERGREENSLAM_LIFELONG_IDS_H_

#include <tuple>

namespace evergreenslam::lifelong {

// Ids carry their session: the {session, index} composition is what makes them non-reusable.
struct SubmapId {
  int session_id = 0;
  int submap_index = 0;

  bool operator==(const SubmapId& other) const {
    return session_id == other.session_id && submap_index == other.submap_index;
  }
  bool operator<(const SubmapId& other) const {
    return std::tie(session_id, submap_index) < std::tie(other.session_id, other.submap_index);
  }
};

struct NodeId {
  int session_id = 0;
  int node_index = 0;

  bool operator==(const NodeId& other) const {
    return session_id == other.session_id && node_index == other.node_index;
  }
  bool operator<(const NodeId& other) const {
    return std::tie(session_id, node_index) < std::tie(other.session_id, other.node_index);
  }
};

struct SessionId {
  int session_index = 0;

  bool operator==(const SessionId& other) const { return session_index == other.session_index; }
  bool operator!=(const SessionId& other) const { return !(*this == other); }
  bool operator<(const SessionId& other) const { return session_index < other.session_index; }
};

inline SessionId SessionOf(const NodeId& id) { return SessionId{id.session_id}; }
inline SessionId SessionOf(const SubmapId& id) { return SessionId{id.session_id}; }

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_IDS_H_
