/**
 * @file place_store.h
 * @author hang chen (chen@hang.plus)
 * @brief The memory/ tree as the process sees it: README install, place.yaml bindings, index.tsv.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_MEMORY_PLACE_STORE_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_MEMORY_PLACE_STORE_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <string>
#include <vector>

#include "lifelong/anchors/anchor_store.h"

namespace evergreenslam::agent {

struct PlaceFile {
  // Node path of the directory holding place.yaml, relative to memory/ ("places/dock").
  std::string path;
  lifelong::AnchorId anchor = 0;
};

struct PlaceScan {
  // Sorted by path; includes the duplicates.
  std::vector<PlaceFile> places;
  // Every path whose anchor another place.yaml also names (a `cp -r`), sorted.
  std::vector<std::string> duplicate_paths;
  // place.yaml files found, parsable or not.
  int num_place_files = 0;

  // `places` without the duplicates: what `here` considers.
  std::vector<PlaceFile> Unique() const;
};

// A place with its anchor resolved in one task, as snapshot files and index.tsv list it.
struct PlaceRow {
  std::string path;
  lifelong::AnchorId anchor = 0;
  // Agent-facing state: pending, frozen, rebound, orphan; "unknown" when the store lacks the id.
  std::string state;
  std::optional<std::string> orphan_reason;
  std::optional<Eigen::Affine2d> pose;
};

class PlaceStore {
 public:
  explicit PlaceStore(const std::string& map_dir);

  // Creates memory/places/ and rewrites memory/README.md.
  bool Install() const;

  // memory/places/**/place.yaml, skipping skills/ and symlinks.
  PlaceScan Scan() const;
  // The anchor id `<node_path>/place.yaml` names; empty when absent or unparsable.
  std::optional<lifelong::AnchorId> ReadBinding(const std::string& node_path) const;
  // mkdir -p, then place.yaml = "anchor: <id>" atomically.
  bool WriteBinding(const std::string& node_path, lifelong::AnchorId id) const;
  bool WriteIndex(const std::vector<PlaceRow>& rows, int num_solves) const;

  // Throws RequestError: node paths start with places/, then slug directories, never skills.
  static void CheckNodePath(const std::string& node_path);
  static std::optional<lifelong::AnchorId> ParsePlaceYaml(const std::string& text);

  const std::string& map_dir() const { return map_dir_; }
  const std::string& memory_dir() const { return memory_dir_; }

 private:
  std::string map_dir_;
  std::string memory_dir_;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_MEMORY_PLACE_STORE_H_
