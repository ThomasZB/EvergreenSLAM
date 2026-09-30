/**
 * @file zone_store.cc
 * @author hang chen (chen@hang.plus)
 * @brief Keep-out zones in memory/: zone.yaml polygons in the frame of their nearest place.
 * @version 0.2
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/memory/zone_store.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <utility>

#include "common/file.h"
#include "service/graph/graph_reads.h"
#include "service/memory/fs_sandbox.h"
#include "service/memory/place_store.h"

namespace evergreenslam::agent {
namespace {

namespace fs = std::filesystem;

constexpr char kPlaceFileName[] = "place.yaml";

struct ParsedZone {
  std::string kind;
  std::optional<std::string> frame;
  Polygon2d polygon;
};

std::optional<ParsedZone> ParseZoneYaml(const std::string& text) {
  try {
    const YAML::Node node = YAML::Load(text);
    if (!node.IsMap() || !node["kind"] || !node["kind"].IsScalar() || !node["polygon"] ||
        !node["polygon"].IsSequence()) {
      return std::nullopt;
    }
    ParsedZone zone;
    zone.kind = node["kind"].as<std::string>();
    if (node["frame"] && node["frame"].IsScalar()) {
      zone.frame = node["frame"].as<std::string>();
    }
    for (const YAML::Node& vertex : node["polygon"]) {
      if (!vertex.IsSequence() || vertex.size() != 2) {
        return std::nullopt;
      }
      const Eigen::Vector2d p(vertex[0].as<double>(), vertex[1].as<double>());
      if (!p.allFinite()) {
        return std::nullopt;
      }
      zone.polygon.push_back(p);
    }
    if (zone.polygon.size() < 3) {
      return std::nullopt;
    }
    return zone;
  } catch (const YAML::Exception&) {
    return std::nullopt;
  }
}

bool IsRegularFile(const fs::path& path) {
  std::error_code error;
  return !fs::is_symlink(path, error) && fs::is_regular_file(path, error);
}

bool TooLarge(const Polygon2d& polygon) {
  Eigen::Vector2d lo = polygon.front();
  Eigen::Vector2d hi = polygon.front();
  for (const Eigen::Vector2d& p : polygon) {
    lo = lo.cwiseMin(p);
    hi = hi.cwiseMax(p);
  }
  return ((hi - lo).array() > ZoneStore::kMaxExtentM).any();
}

// Memory.nearest_place in egs, from the parent up to places/; a place.yaml beside zone.yaml is
// holds_place. False when a place.yaml exists but cannot be read (the scan is then incomplete).
bool FindFrame(const fs::path& memory, ZoneFile& zone) {
  if (fs::exists(memory / zone.path / kPlaceFileName)) {
    zone.problem = "holds_place";
    return true;
  }
  std::string directory = zone.path;
  while (directory.find('/') != std::string::npos) {
    directory = directory.substr(0, directory.rfind('/'));
    const fs::path file = memory / directory / kPlaceFileName;
    if (!IsRegularFile(file)) {
      continue;
    }
    zone.frame_path = directory;
    const std::optional<std::string> text = common::ReadFile(file.string());
    if (!text.has_value()) {
      return false;
    }
    zone.anchor = PlaceStore::ParsePlaceYaml(*text);
    if (!zone.anchor.has_value()) {
      zone.problem = "unreadable_place";
    }
    return true;
  }
  zone.problem = "no_place";
  return true;
}

}  // namespace

ZoneStore::ZoneStore(std::string memory_dir) : memory_dir_(std::move(memory_dir)) {}

ZoneScan ZoneStore::Scan() const {
  ZoneScan scan;
  std::vector<ZoneFile>& zones = scan.zones;
  const fs::path memory(memory_dir_);
  const fs::path places = memory / "places";
  std::error_code error;
  if (!fs::exists(places, error) && !error) {
    return scan;
  }
  fs::recursive_directory_iterator it(places, error);
  const fs::recursive_directory_iterator end;
  for (; !error && it != end; it.increment(error)) {
    const fs::directory_entry& entry = *it;
    std::error_code status_error;
    if (entry.is_symlink(status_error)) {
      continue;
    }
    if (entry.is_directory(status_error)) {
      if (FsSandbox::IsReservedName(entry.path().filename().string())) {
        it.disable_recursion_pending();
      }
      continue;
    }
    if (entry.path().filename() != kFileName || !entry.is_regular_file(status_error)) {
      continue;
    }
    ZoneFile zone;
    zone.path = entry.path().parent_path().lexically_relative(memory).generic_string();
    const std::optional<std::string> text = common::ReadFile(entry.path().string());
    if (!text.has_value()) {
      scan.error = "cannot read " + entry.path().string();
      break;
    }
    const std::optional<ParsedZone> parsed = ParseZoneYaml(*text);
    if (!parsed.has_value()) {
      LOG(WARNING) << "unparsable " << entry.path();
      zone.problem = "bad_zone_file";
    } else {
      zone.kind = parsed->kind;
      zone.frame = parsed->frame;
      zone.polygon = parsed->polygon;
      if (TooLarge(zone.polygon)) {
        zone.problem = "too_large";
      } else if (!FindFrame(memory, zone)) {
        scan.error = "cannot read the place.yaml above " + entry.path().string();
        break;
      }
    }
    zones.push_back(std::move(zone));
  }
  if (error && !scan.error.has_value()) {
    scan.error = "cannot walk " + places.string() + ": " + error.message();
  }
  if (scan.error.has_value()) {
    LOG_EVERY_N(WARNING, 100) << "zone scan incomplete: " << *scan.error;
  }
  std::sort(zones.begin(), zones.end(),
            [](const ZoneFile& a, const ZoneFile& b) { return a.path < b.path; });
  return scan;
}

std::vector<ResolvedZone> ResolveZonesOnTask(const lifelong::PoseGraph& pose_graph,
                                             const std::vector<ZoneFile>& zones) {
  std::map<lifelong::AnchorId, std::optional<lifelong::ResolvedAnchor>> anchors;
  std::vector<ResolvedZone> resolved;
  resolved.reserve(zones.size());
  for (const ZoneFile& zone : zones) {
    ResolvedZone out;
    out.path = zone.path;
    out.kind = zone.kind;
    out.frame = zone.frame;
    out.frame_path = zone.frame_path;
    out.anchor = zone.anchor;
    out.polygon = zone.polygon;
    out.reason = zone.problem;
    if (zone.problem.has_value() || !zone.anchor.has_value()) {
      resolved.push_back(std::move(out));
      continue;
    }
    auto it = anchors.find(*zone.anchor);
    if (it == anchors.end()) {
      it =
          anchors
              .emplace(*zone.anchor, pose_graph.anchors().Resolve(pose_graph.graph(), *zone.anchor))
              .first;
    }
    const std::optional<lifelong::ResolvedAnchor>& anchor = it->second;
    if (!anchor.has_value()) {
      out.state = "orphan";
      out.reason = "unknown_anchor";
    } else {
      out.state = AgentState(*anchor);
      if (!anchor->global_pose.has_value()) {
        out.reason = "orphan";
      } else if (zone.kind != ZoneStore::kKeepout) {
        out.reason = "unknown_kind";
      } else {
        Polygon2d xy;
        xy.reserve(zone.polygon.size());
        for (const Eigen::Vector2d& vertex : zone.polygon) {
          xy.push_back(*anchor->global_pose * vertex);
        }
        out.polygon_xy = std::move(xy);
        if (zone.frame.has_value() && zone.frame != zone.frame_path) {
          out.reason = "frame_mismatch";
        }
      }
    }
    resolved.push_back(std::move(out));
  }
  return resolved;
}

std::optional<std::vector<ResolvedZone>> ResolveZones(const lifelong::PoseGraph& pose_graph,
                                                      const ZoneStore& store) {
  const ZoneScan scan = store.Scan();
  if (scan.error.has_value()) {
    return std::nullopt;
  }
  return ResolveZonesOnTask(pose_graph, scan.zones);
}

int CountActiveKeepouts(const std::vector<ResolvedZone>& zones) {
  return static_cast<int>(std::count_if(
      zones.begin(), zones.end(), [](const ResolvedZone& zone) { return zone.IsActiveKeepout(); }));
}

}  // namespace evergreenslam::agent
