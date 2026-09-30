/**
 * @file zone_store.h
 * @author hang chen (chen@hang.plus)
 * @brief Keep-out zones in memory/: zone.yaml polygons in the frame of their nearest place.
 * @version 0.2
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_ZONE_STORE_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_ZONE_STORE_H_

#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "lifelong/anchors/anchor_store.h"
#include "lifelong/pose_graph.h"

namespace evergreenslam::agent {

using Polygon2d = std::vector<Eigen::Vector2d>;

// One memory/places/**/zone.yaml as read from disk, paired with its nearest place.yaml.
struct ZoneFile {
  // Node path of the directory holding zone.yaml ("places/kitchen/slope").
  std::string path;
  std::string kind;
  // `frame:` as written; the walk-up decides, this only detects a moved directory.
  std::optional<std::string> frame;
  // Metres, in the anchor frame of `frame_path`.
  Polygon2d polygon;
  // Nearest directory at or above `path` holding a place.yaml.
  std::optional<std::string> frame_path;
  std::optional<lifelong::AnchorId> anchor;
  // bad_zone_file, too_large, holds_place, no_place or unreadable_place; empty when resolvable.
  std::optional<std::string> problem;
};

struct ResolvedZone {
  std::string path;
  std::string kind;
  std::optional<std::string> frame;
  std::optional<std::string> frame_path;
  std::optional<lifelong::AnchorId> anchor;
  // pending, frozen, rebound, orphan; empty without an anchor.
  std::optional<std::string> state;
  Polygon2d polygon;
  // Map frame, from the anchor's global pose in the same solve; empty when unresolvable.
  std::optional<Polygon2d> polygon_xy;
  // Why polygon_xy is empty, or frame_mismatch beside a resolved polygon_xy.
  std::optional<std::string> reason;

  bool IsActiveKeepout() const { return kind == "keepout" && polygon_xy.has_value(); }
};

struct ZoneScan {
  std::vector<ZoneFile> zones;
  // Set when part of memory/places could not be read (EACCES, or a directory moved mid-walk):
  // `zones` is then partial, and a mask built from it would drop zones that still exist.
  std::optional<std::string> error;
};

class ZoneStore {
 public:
  explicit ZoneStore(std::string memory_dir);

  // memory/places/**/zone.yaml, skipping skills/ and symlinks, sorted by path. File reads only.
  ZoneScan Scan() const;

  const std::string& memory_dir() const { return memory_dir_; }

  static constexpr char kFileName[] = "zone.yaml";
  static constexpr char kKeepout[] = "keepout";
  // Wider or taller than this in the anchor frame → too_large: a typo, not a zone.
  static constexpr double kMaxExtentM = 100.0;

 private:
  std::string memory_dir_;
};

// Backend task only: pairs every scanned zone with its anchor's current global pose.
std::vector<ResolvedZone> ResolveZonesOnTask(const lifelong::PoseGraph& pose_graph,
                                             const std::vector<ZoneFile>& zones);
// Backend task only; the host's entry point (API.md "Core entry points"). Scans, then resolves;
// nullopt when the scan was incomplete.
std::optional<std::vector<ResolvedZone>> ResolveZones(const lifelong::PoseGraph& pose_graph,
                                                      const ZoneStore& store);

// Anywhere: the counts a legend or a log line prints.
int CountActiveKeepouts(const std::vector<ResolvedZone>& zones);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_ZONE_STORE_H_
