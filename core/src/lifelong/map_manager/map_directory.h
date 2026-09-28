/**
 * @file map_directory.h
 * @author hang chen (chen@hang.plus)
 * @brief The map directory's file set: generation-named files published by a manifest renamed
 * into place last, abandoned whole on any failure, and garbage collected at load.
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_DIRECTORY_H_
#define EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_DIRECTORY_H_

#include <cstdint>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace evergreenslam::lifelong {

// Knows file names and bytes, not what they hold: which files a commit rewrites and what the
// manifest says is the caller's.
class MapDirectory {
 public:
  struct Commit {
    int64_t generation = 0;
    std::vector<std::string> written;
    // Removed once the commit is published, never before.
    std::vector<std::string> superseded;
  };

  explicit MapDirectory(std::string directory);

  Commit Begin() const;
  // Writes `<stem>.g<generation>.pb` and returns its name; on failure nothing of it is left.
  std::optional<std::string> Write(Commit& commit, const std::string& stem,
                                   const std::string& contents);
  // The manifest goes last; on failure every file the commit wrote is removed and the previous
  // generation stays the published one.
  bool Publish(const Commit& commit, const std::string& manifest);
  void Abandon(const Commit& commit) const;

  std::optional<std::string> ReadManifest() const;
  std::optional<std::string> Read(const std::string& file_name) const;
  // Session or anchor files at the top level: with no manifest, a lost one rather than a new map.
  bool HasCommitFiles() const;
  // Only `session_*.pb`, `anchors*.pb` and `*.tmp` at the top level are candidates.
  void RemoveUnreferenced(const std::set<std::string>& referenced) const;
  std::string PathOf(const std::string& file_name) const;

  void set_generation(int64_t generation) { generation_ = generation; }
  void set_before_manifest_write_hook(std::function<void()> hook) {
    before_manifest_write_hook_ = std::move(hook);
  }

  const std::string& directory() const { return directory_; }
  int64_t generation() const { return generation_; }

 private:
  void Remove(const std::vector<std::string>& file_names) const;

  std::string directory_;
  int64_t generation_ = 0;
  std::function<void()> before_manifest_write_hook_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_DIRECTORY_H_
