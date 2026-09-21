/**
 * @file pose_graph_view.cc
 * @author hang chen (chen@hang.plus)
 * @brief The lifelong backend's state as one JSON document for the browser.
 * @version 0.1
 * @date 2026-08-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "pose_graph_view.h"

#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>

#include "lifelong/pose_graph.h"
#include "utils/transform/transform.h"

namespace evergreenslam::webui {
namespace {

using evergreenslam::utils::transform::GetYaw;
using evergreenslam::utils::transform::NormalizeAngle;

void AppendPose(std::ostringstream& out, const Eigen::Affine2d& pose) {
  out << '[' << pose.translation().x() << ',' << pose.translation().y() << ',' << GetYaw(pose)
      << ']';
}

void AppendTime(std::ostringstream& out, common::Time time) {
  out << std::setprecision(16) << common::ToUnixSeconds(time) << std::setprecision(9);
}

void AppendString(std::ostringstream& out, const std::string& text) {
  out << '"';
  for (const char character : text) {
    if (character == '"' || character == '\\') {
      out << '\\';
    }
    out << character;
  }
  out << '"';
}

// Edge type codes shared with the page: 0 odometry, 1 loop closure, 2 prior, 3 summary.
int TypeCode(const lifelong::Constraint& constraint) {
  if (constraint.recovered) {
    return 3;
  }
  switch (constraint.type) {
    case lifelong::Constraint::Type::INTRA_SUBMAP:
      return 0;
    case lifelong::Constraint::Type::INTER_SUBMAP:
      return 1;
    case lifelong::Constraint::Type::PRIOR:
      return 2;
  }
  return 0;
}

int SessionOf(const lifelong::VariableId& id) {
  return id.kind == lifelong::VariableId::Kind::NODE ? id.node_id().session_id
                                                     : id.submap_id().session_id;
}

std::optional<Eigen::Affine2d> PoseOf(const lifelong::PoseGraphData& data,
                                      const lifelong::VariableId& id) {
  if (!data.HasVariable(id)) {
    return std::nullopt;
  }
  return id.kind == lifelong::VariableId::Kind::NODE ? data.node(id.node_id()).global_pose
                                                     : data.submap(id.submap_id()).global_pose;
}

void AppendResidual(std::ostringstream& out, const lifelong::Constraint& constraint,
                    const Eigen::Affine2d& from_pose,
                    const std::optional<Eigen::Affine2d>& to_pose) {
  const Eigen::Affine2d predicted = to_pose.has_value()
                                        ? Eigen::Affine2d(from_pose * constraint.relative_pose)
                                        : constraint.relative_pose;
  const Eigen::Affine2d target = to_pose.value_or(from_pose);
  const Eigen::Affine2d error = Eigen::Affine2d(predicted.inverse() * target);
  out << error.translation().norm() << ',' << std::abs(NormalizeAngle(GetYaw(error)));
}

void AppendSessions(std::ostringstream& out, const lifelong::PoseGraph& graph) {
  const lifelong::PoseGraphData& data = graph.graph();
  const std::optional<lifelong::SessionId> fed = graph.session_manager().fed_session();
  out << "\"sessions\":[";
  bool first = true;
  for (const auto& [id, session] : data.sessions()) {
    if (!first) {
      out << ',';
    }
    first = false;
    out << "{\"id\":" << id.session_index << ",\"frozen\":" << (session.frozen() ? "true" : "false")
        << ",\"fed\":" << (fed.has_value() && *fed == id ? "true" : "false")
        << ",\"nodes\":" << session.node_ids.size() << ",\"submaps\":" << session.submap_ids.size()
        << ",\"start\":";
    AppendTime(out, session.start_time);
    out << ",\"duration\":" << common::ToSeconds(session.last_node_time - session.start_time)
        << '}';
  }
  out << ']';
}

void AppendNodes(std::ostringstream& out, const lifelong::PoseGraphData& data) {
  out << "\"nodes\":{\"session\":[";
  bool first = true;
  for (const auto& [id, node] : data.nodes()) {
    out << (first ? "" : ",") << id.session_id;
    first = false;
  }
  out << "],\"pose\":[";
  first = true;
  for (const auto& [id, node] : data.nodes()) {
    out << (first ? "" : ",") << node.global_pose.translation().x() << ','
        << node.global_pose.translation().y() << ',' << GetYaw(node.global_pose);
    first = false;
  }
  out << "]}";
}

void AppendSubmaps(std::ostringstream& out, const lifelong::PoseGraphData& data) {
  out << "\"submaps\":[";
  bool first = true;
  for (const auto& [id, record] : data.submaps()) {
    if (!first) {
      out << ',';
    }
    first = false;
    out << "{\"session\":" << id.session_id << ",\"index\":" << id.submap_index
        << ",\"scans\":" << record.submap->num_scans()
        << ",\"finished\":" << (record.submap->finished() ? "true" : "false") << ",\"pose\":";
    AppendPose(out, record.global_pose);
    const Eigen::Affine2d& world = record.global_pose;
    out << ",\"to_world\":";
    AppendPose(out, world);
    // Only a finished submap has an immutable grid; reading an unfinished one races the frontend.
    if (record.submap->finished()) {
      const mapping::GridMapu8& snapshot = record.submap->Snapshot();
      if (snapshot.width() > 0 && snapshot.height() > 0) {
        const double x0 = snapshot.origin_x();
        const double y0 = snapshot.origin_y();
        const double x1 = x0 + snapshot.width() * snapshot.resolution();
        const double y1 = y0 + snapshot.height() * snapshot.resolution();
        out << ",\"box\":[";
        bool first_corner = true;
        for (const Eigen::Vector2d& corner : {Eigen::Vector2d(x0, y0), Eigen::Vector2d(x1, y0),
                                              Eigen::Vector2d(x1, y1), Eigen::Vector2d(x0, y1)}) {
          const Eigen::Vector2d point = world * corner;
          out << (first_corner ? "" : ",") << point.x() << ',' << point.y();
          first_corner = false;
        }
        out << ']';
      }
    }
    out << '}';
  }
  out << ']';
}

void AppendConstraints(std::ostringstream& out, const lifelong::PoseGraphData& data) {
  std::ostringstream types;
  std::ostringstream ends;
  std::ostringstream residuals;
  std::ostringstream sessions;
  bool first = true;
  for (const lifelong::Constraint& constraint : data.constraints()) {
    const std::optional<Eigen::Affine2d> from_pose = PoseOf(data, constraint.from);
    if (!from_pose.has_value()) {
      continue;
    }
    std::optional<Eigen::Affine2d> to_pose;
    if (constraint.to.has_value()) {
      to_pose = PoseOf(data, *constraint.to);
      if (!to_pose.has_value()) {
        continue;
      }
    }
    const Eigen::Vector2d target = to_pose.has_value()
                                       ? Eigen::Vector2d(to_pose->translation())
                                       : Eigen::Vector2d(constraint.relative_pose.translation());
    types << (first ? "" : ",") << TypeCode(constraint);
    ends << (first ? "" : ",") << from_pose->translation().x() << ','
         << from_pose->translation().y() << ',' << target.x() << ',' << target.y();
    sessions << (first ? "" : ",") << SessionOf(constraint.from) << ','
             << (constraint.to.has_value() ? SessionOf(*constraint.to)
                                           : SessionOf(constraint.from));
    residuals << (first ? "" : ",");
    AppendResidual(residuals, constraint, *from_pose, to_pose);
    first = false;
  }
  out << "\"constraints\":{\"type\":[" << types.str() << "],\"ends\":[" << ends.str()
      << "],\"session\":[" << sessions.str() << "],\"residual\":[" << residuals.str() << "]}";
}

void AppendFreeze(std::ostringstream& out, const lifelong::SessionManager& manager) {
  const lifelong::FreezeVerdict& verdict = manager.last_verdict();
  out << "\"freeze\":{\"eligible\":" << (verdict.eligible ? "true" : "false") << ",\"rejection\":";
  AppendString(out, lifelong::ToString(verdict.rejection));
  out << ",\"submaps\":[";
  bool first = true;
  for (const lifelong::SubmapUncertainty& uncertainty : verdict.submaps) {
    out << (first ? "" : ",") << "{\"session\":" << uncertainty.id.session_id
        << ",\"index\":" << uncertainty.id.submap_index
        << ",\"unfinished\":" << (uncertainty.unfinished ? "true" : "false")
        << ",\"available\":" << (uncertainty.covariance_available ? "true" : "false")
        << ",\"passed\":" << (uncertainty.passed ? "true" : "false") << ",\"stddev\":["
        << uncertainty.stddev.x() << ',' << uncertainty.stddev.y() << ',' << uncertainty.stddev.z()
        << "]}";
    first = false;
  }
  out << "]}";
}

void AppendStats(std::ostringstream& out, const lifelong::PoseGraph& graph) {
  const lifelong::PoseGraphData& data = graph.graph();
  int num_frozen = 0;
  for (const auto& [id, session] : data.sessions()) {
    if (session.frozen()) {
      ++num_frozen;
    }
  }
  out << "\"stats\":{\"nodes\":" << data.nodes().size() << ",\"submaps\":" << data.submaps().size()
      << ",\"constraints\":" << data.constraints().size()
      << ",\"sessions\":" << data.sessions().size() << ",\"frozen_sessions\":" << num_frozen
      << ",\"matches_attempted\":" << graph.constraint_builder().num_matches_attempted()
      << ",\"matches_accepted\":" << graph.constraint_builder().num_constraints_added()
      << ",\"sessions_frozen\":" << graph.session_manager().num_sessions_frozen()
      << ",\"submaps_trimmed\":" << graph.trimmer().num_submaps_trimmed()
      << ",\"priors_added\":" << graph.trimmer().num_priors_added()
      << ",\"variables\":" << graph.optimization().num_variables()
      << ",\"residual_blocks\":" << graph.optimization().num_residual_blocks();
  if (graph.map_manager() != nullptr) {
    out << ",\"checkpoints\":" << graph.map_manager()->num_checkpoints_written()
        << ",\"frozen_files\":" << graph.map_manager()->num_frozen_files_written()
        << ",\"map_dir\":";
    AppendString(out, graph.map_manager()->directory());
  }
  out << '}';
}

}  // namespace

std::string SerializePoseGraph(const lifelong::PoseGraph& graph) {
  std::ostringstream out;
  out.precision(9);
  out << '{';
  AppendSessions(out, graph);
  out << ',';
  AppendNodes(out, graph.graph());
  out << ',';
  AppendSubmaps(out, graph.graph());
  out << ',';
  AppendConstraints(out, graph.graph());
  out << ',';
  AppendFreeze(out, graph.session_manager());
  out << ',';
  AppendStats(out, graph);
  // The DebugSink publishes in the frontend's local frame; everything above is global.
  const std::optional<Eigen::Affine2d> to_global = graph.ActiveSessionToGlobal();
  out << ",\"to_global\":";
  if (to_global.has_value()) {
    AppendPose(out, *to_global);
  } else {
    out << "null";
  }
  out << '}';
  return out.str();
}

}  // namespace evergreenslam::webui
