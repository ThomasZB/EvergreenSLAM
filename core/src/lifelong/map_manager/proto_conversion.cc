/**
 * @file proto_conversion.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/map_manager/proto_conversion.h"

#include <glog/logging.h>

#include <string>
#include <utility>

#include "mapping/grid_mapping/probability_grid.h"
#include "mapping/grid_mapping/probability_values.h"
#include "utils/transform/transform.h"

namespace evergreenslam::lifelong {
namespace {

namespace transform = utils::transform;

proto::SessionState ToProtoState(SessionState state) {
  return state == SessionState::FROZEN ? proto::FROZEN : proto::ACTIVE;
}

proto::Anchor::State ToProtoState(AnchorState state) {
  switch (state) {
    case AnchorState::BOUND:
      return proto::Anchor::BOUND;
    case AnchorState::REBOUND:
      return proto::Anchor::REBOUND;
    case AnchorState::ORPHAN:
      return proto::Anchor::ORPHAN;
  }
  LOG(FATAL) << "unhandled anchor state";
  return proto::Anchor::ORPHAN;
}

AnchorState FromProtoState(proto::Anchor::State state) {
  switch (state) {
    case proto::Anchor::BOUND:
      return AnchorState::BOUND;
    case proto::Anchor::REBOUND:
      return AnchorState::REBOUND;
    case proto::Anchor::ORPHAN:
      return AnchorState::ORPHAN;
    default:
      LOG(FATAL) << "unknown anchor state " << static_cast<int>(state);
  }
  return AnchorState::ORPHAN;
}

proto::Anchor::OrphanReason ToProtoReason(OrphanReason reason) {
  switch (reason) {
    case OrphanReason::NONE:
      return proto::Anchor::NONE;
    case OrphanReason::TRIMMED_NO_SUCCESSOR:
      return proto::Anchor::TRIMMED_NO_SUCCESSOR;
    case OrphanReason::SESSION_REMOVED:
      return proto::Anchor::SESSION_REMOVED;
    case OrphanReason::SUBMAP_MISSING:
      return proto::Anchor::SUBMAP_MISSING;
  }
  LOG(FATAL) << "unhandled orphan reason";
  return proto::Anchor::NONE;
}

OrphanReason FromProtoReason(proto::Anchor::OrphanReason reason) {
  switch (reason) {
    case proto::Anchor::NONE:
      return OrphanReason::NONE;
    case proto::Anchor::TRIMMED_NO_SUCCESSOR:
      return OrphanReason::TRIMMED_NO_SUCCESSOR;
    case proto::Anchor::SESSION_REMOVED:
      return OrphanReason::SESSION_REMOVED;
    case proto::Anchor::SUBMAP_MISSING:
      return OrphanReason::SUBMAP_MISSING;
    default:
      LOG(FATAL) << "unknown orphan reason " << static_cast<int>(reason);
  }
  return OrphanReason::NONE;
}

}  // namespace

proto::Pose2d ToProto(const Eigen::Affine2d& pose) {
  proto::Pose2d result;
  result.set_x(pose.translation().x());
  result.set_y(pose.translation().y());
  result.set_theta(transform::GetYaw(pose));
  return result;
}

Eigen::Affine2d FromProto(const proto::Pose2d& proto) {
  return transform::FromXYTheta(proto.x(), proto.y(), proto.theta());
}

proto::Matrix3d ToProto(const Eigen::Matrix3d& matrix) {
  proto::Matrix3d result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result.add_data(matrix(row, col));
    }
  }
  return result;
}

Eigen::Matrix3d FromProto(const proto::Matrix3d& proto) {
  CHECK_EQ(proto.data_size(), 9) << "a 3x3 matrix is nine doubles, row major";
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      matrix(row, col) = proto.data(row * 3 + col);
    }
  }
  return matrix;
}

proto::PointCloud ToProto(const sensor::PointCloud& point_cloud) {
  proto::PointCloud result;
  result.mutable_xy()->Reserve(static_cast<int>(2 * point_cloud.size()));
  for (const sensor::Point2d& point : point_cloud) {
    result.add_xy(point.point.x());
    result.add_xy(point.point.y());
  }
  return result;
}

sensor::PointCloud FromProto(const proto::PointCloud& proto) {
  CHECK_EQ(proto.xy_size() % 2, 0) << "point coordinates come in pairs";
  sensor::PointCloud point_cloud;
  point_cloud.points().reserve(static_cast<size_t>(proto.xy_size() / 2));
  for (int i = 0; i + 1 < proto.xy_size(); i += 2) {
    point_cloud.push_back(sensor::Point2d{Eigen::Vector2d(proto.xy(i), proto.xy(i + 1))});
  }
  return point_cloud;
}

proto::Grid ToProto(const mapping::GridMapu8& grid) {
  proto::Grid result;
  result.set_width(grid.width());
  result.set_height(grid.height());
  result.set_resolution(grid.resolution());
  result.set_origin_x(grid.origin_x());
  result.set_origin_y(grid.origin_y());
  const std::vector<uint8_t>& cells = grid.data();
  CHECK_EQ(cells.size(), static_cast<size_t>(grid.width()) * grid.height());
  for (const uint8_t value : cells) {
    CHECK_LT(value, mapping::kUpdateMarker) << "a grid still carrying update markers is mid "
                                               "insertion and must not be serialized";
  }
  result.set_cells(std::string(reinterpret_cast<const char*>(cells.data()), cells.size()));
  return result;
}

mapping::GridMapu8 FromProto(const proto::Grid& proto) {
  CHECK_GE(proto.width(), 0);
  CHECK_GE(proto.height(), 0);
  CHECK_EQ(proto.cells().size(), static_cast<size_t>(proto.width()) * proto.height())
      << "cell count does not match the declared size";
  const std::string& cells = proto.cells();
  std::vector<uint8_t> data(cells.begin(), cells.end());
  return mapping::GridMapu8(std::move(data), proto.width(), proto.height(), proto.resolution(),
                            proto.origin_x(), proto.origin_y(), mapping::kUnknownValue);
}

proto::SubmapId ToProto(const SubmapId& id) {
  proto::SubmapId result;
  result.set_session_id(id.session_id);
  result.set_submap_index(id.submap_index);
  return result;
}

SubmapId FromProto(const proto::SubmapId& proto) {
  return SubmapId{proto.session_id(), proto.submap_index()};
}

proto::NodeId ToProto(const NodeId& id) {
  proto::NodeId result;
  result.set_session_id(id.session_id);
  result.set_node_index(id.node_index);
  return result;
}

NodeId FromProto(const proto::NodeId& proto) {
  return NodeId{proto.session_id(), proto.node_index()};
}

proto::VariableId ToProto(const VariableId& id) {
  proto::VariableId result;
  result.set_kind(id.kind == VariableId::Kind::SUBMAP ? proto::VariableId::SUBMAP
                                                      : proto::VariableId::NODE);
  result.set_session_id(id.session_id);
  result.set_index(id.index);
  return result;
}

VariableId FromProto(const proto::VariableId& proto) {
  VariableId id;
  id.kind =
      proto.kind() == proto::VariableId::SUBMAP ? VariableId::Kind::SUBMAP : VariableId::Kind::NODE;
  id.session_id = proto.session_id();
  id.index = proto.index();
  return id;
}

proto::Constraint ToProto(const Constraint& constraint) {
  proto::Constraint result;
  switch (constraint.type) {
    case Constraint::Type::INTRA_SUBMAP:
      result.set_type(proto::Constraint::INTRA_SUBMAP);
      break;
    case Constraint::Type::INTER_SUBMAP:
      result.set_type(proto::Constraint::INTER_SUBMAP);
      break;
    case Constraint::Type::PRIOR:
      result.set_type(proto::Constraint::PRIOR);
      break;
  }
  *result.mutable_from() = ToProto(constraint.from);
  if (constraint.to.has_value()) {
    *result.mutable_to() = ToProto(*constraint.to);
  }
  *result.mutable_relative_pose() = ToProto(constraint.relative_pose);
  *result.mutable_sqrt_information() = ToProto(constraint.sqrt_information);
  result.set_recovered(constraint.recovered);
  return result;
}

Constraint FromProto(const proto::Constraint& proto) {
  Constraint constraint;
  switch (proto.type()) {
    case proto::Constraint::INTER_SUBMAP:
      constraint.type = Constraint::Type::INTER_SUBMAP;
      break;
    case proto::Constraint::PRIOR:
      constraint.type = Constraint::Type::PRIOR;
      break;
    default:
      constraint.type = Constraint::Type::INTRA_SUBMAP;
      break;
  }
  constraint.from = FromProto(proto.from());
  if (proto.has_to()) {
    constraint.to = FromProto(proto.to());
  }
  constraint.relative_pose = FromProto(proto.relative_pose());
  constraint.sqrt_information = FromProto(proto.sqrt_information());
  constraint.recovered = proto.recovered();
  return constraint;
}

proto::Node ToProto(const Node& node, const std::vector<SubmapId>& containing_submap_ids) {
  proto::Node result;
  *result.mutable_id() = ToProto(node.id);
  result.set_time_nanos(TimeToProto(node.constant_data.time));
  *result.mutable_local_pose() = ToProto(node.constant_data.local_pose);
  *result.mutable_global_pose() = ToProto(node.global_pose);
  *result.mutable_point_cloud() = ToProto(node.constant_data.point_cloud);
  for (const SubmapId& id : containing_submap_ids) {
    *result.add_containing_submap_ids() = ToProto(id);
  }
  return result;
}

Node FromProto(const proto::Node& proto) {
  Node node;
  node.id = FromProto(proto.id());
  node.constant_data.time = TimeFromProto(proto.time_nanos());
  node.constant_data.local_pose = FromProto(proto.local_pose());
  node.constant_data.point_cloud = FromProto(proto.point_cloud());
  node.global_pose = FromProto(proto.global_pose());
  return node;
}

std::vector<SubmapId> ContainingSubmapIdsFromProto(const proto::Node& proto) {
  std::vector<SubmapId> ids;
  ids.reserve(static_cast<size_t>(proto.containing_submap_ids_size()));
  for (const proto::SubmapId& id : proto.containing_submap_ids()) {
    ids.push_back(FromProto(id));
  }
  return ids;
}

proto::Submap ToProto(const SubmapRecord& record) {
  CHECK(record.submap != nullptr);
  proto::Submap result;
  *result.mutable_id() = ToProto(record.id);
  *result.mutable_local_pose() = ToProto(record.local_pose);
  *result.mutable_global_pose() = ToProto(record.global_pose);
  result.set_num_scans(record.submap->num_scans());
  result.set_finished(record.submap->finished());
  // SnapshotCopy, not Snapshot: the frontend thread may still be inserting into this submap.
  *result.mutable_grid() = ToProto(record.submap->SnapshotCopy());
  return result;
}

SubmapRecord FromProto(const proto::Submap& proto) {
  SubmapRecord record;
  record.id = FromProto(proto.id());
  record.local_pose = FromProto(proto.local_pose());
  record.global_pose = FromProto(proto.global_pose());
  // A loaded submap never re-enters ingest, so local_index is moot.
  record.submap = std::make_shared<const mapping::Submap>(
      record.id.submap_index, record.local_pose,
      mapping::ProbabilityGrid::FromSnapshot(FromProto(proto.grid())), proto.num_scans(),
      proto.finished());
  return record;
}

proto::Session ToProto(const SessionData& session) {
  proto::Session result;
  result.set_session_index(session.id.session_index);
  result.set_state(ToProtoState(session.state));
  result.set_start_time_nanos(TimeToProto(session.start_time));
  result.set_last_node_time_nanos(TimeToProto(session.last_node_time));
  *result.mutable_local_to_global() = ToProto(session.local_to_global);
  return result;
}

SessionData FromProto(const proto::Session& proto) {
  SessionData session;
  session.id = SessionId{proto.session_index()};
  session.state = proto.state() == proto::FROZEN ? SessionState::FROZEN : SessionState::ACTIVE;
  session.start_time = TimeFromProto(proto.start_time_nanos());
  session.last_node_time = TimeFromProto(proto.last_node_time_nanos());
  session.local_to_global = FromProto(proto.local_to_global());
  return session;
}

proto::Anchor ToProto(const Anchor& anchor) {
  proto::Anchor result;
  result.set_id(anchor.id);
  *result.mutable_submap_id() = ToProto(anchor.submap_id);
  *result.mutable_submap_from_anchor() = ToProto(anchor.submap_from_anchor);
  *result.mutable_node_id() = ToProto(anchor.node_id);
  result.set_saved_at_nanos(TimeToProto(anchor.saved_at));
  result.set_state(ToProtoState(anchor.state));
  result.set_orphan_reason(ToProtoReason(anchor.orphan_reason));
  if (anchor.scan.has_value()) {
    *result.mutable_scan() = ToProto(*anchor.scan);
  }
  return result;
}

Anchor FromProto(const proto::Anchor& proto) {
  Anchor anchor;
  anchor.id = proto.id();
  anchor.submap_id = FromProto(proto.submap_id());
  anchor.submap_from_anchor = FromProto(proto.submap_from_anchor());
  anchor.node_id = FromProto(proto.node_id());
  anchor.saved_at = TimeFromProto(proto.saved_at_nanos());
  anchor.state = FromProtoState(proto.state());
  anchor.orphan_reason = FromProtoReason(proto.orphan_reason());
  if (proto.has_scan()) {
    anchor.scan = FromProto(proto.scan());
  }
  return anchor;
}

common::Time TimeFromProto(int64_t nanos) { return common::FromUnixNanos(nanos); }

int64_t TimeToProto(common::Time time) { return common::ToUnixNanos(time); }

}  // namespace evergreenslam::lifelong
