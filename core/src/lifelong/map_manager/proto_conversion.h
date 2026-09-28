/**
 * @file proto_conversion.h
 * @author hang chen (chen@hang.plus)
 * @brief proto <-> core structs. Every protobuf include in the project stops at this header.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_MAP_MANAGER_PROTO_CONVERSION_H_
#define EVERGREENSLAM_LIFELONG_MAP_MANAGER_PROTO_CONVERSION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "common/time.h"
#include "lifelong/anchors/anchor_store.h"
#include "lifelong/map_manager/proto/anchors.pb.h"
#include "lifelong/map_manager/proto/map.pb.h"
#include "lifelong/pose_graph_data.h"
#include "lifelong/sessions/session_data.h"
#include "mapping/grid_mapping/grid_map.h"
#include "mapping/submap.h"
#include "sensor/point_cloud.h"

namespace evergreenslam::lifelong {

proto::Pose2d ToProto(const Eigen::Affine2d& pose);
Eigen::Affine2d FromProto(const proto::Pose2d& proto);

proto::Matrix3d ToProto(const Eigen::Matrix3d& matrix);
Eigen::Matrix3d FromProto(const proto::Matrix3d& proto);

proto::PointCloud ToProto(const sensor::PointCloud& point_cloud);
sensor::PointCloud FromProto(const proto::PointCloud& proto);

proto::Grid ToProto(const mapping::GridMapu8& grid);
mapping::GridMapu8 FromProto(const proto::Grid& proto);

proto::SubmapId ToProto(const SubmapId& id);
SubmapId FromProto(const proto::SubmapId& proto);
proto::NodeId ToProto(const NodeId& id);
NodeId FromProto(const proto::NodeId& proto);
proto::VariableId ToProto(const VariableId& id);
VariableId FromProto(const proto::VariableId& proto);

proto::Constraint ToProto(const Constraint& constraint);
Constraint FromProto(const proto::Constraint& proto);

proto::Node ToProto(const Node& node, const std::vector<SubmapId>& containing_submap_ids);
Node FromProto(const proto::Node& proto);
std::vector<SubmapId> ContainingSubmapIdsFromProto(const proto::Node& proto);

proto::Submap ToProto(const SubmapRecord& record);
SubmapRecord FromProto(const proto::Submap& proto);

proto::Session ToProto(const SessionData& session);
SessionData FromProto(const proto::Session& proto);

proto::Anchor ToProto(const Anchor& anchor);
Anchor FromProto(const proto::Anchor& proto);

common::Time TimeFromProto(int64_t nanos);
int64_t TimeToProto(common::Time time);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_MAP_MANAGER_PROTO_CONVERSION_H_
