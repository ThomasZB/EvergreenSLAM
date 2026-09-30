/**
 * @file fs_sandbox.h
 * @author hang chen (chen@hang.plus)
 * @brief Path rules for everything the service touches under memory/: no escape, no symlinks.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_MEMORY_FS_SANDBOX_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_MEMORY_FS_SANDBOX_H_

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace evergreenslam::agent {

struct SandboxPath {
  // Normalized, relative to memory/; empty for memory/ itself.
  std::string relative;
  std::vector<std::string> components;
  std::filesystem::path absolute;
  // How many leading components exist on disk.
  size_t existing_components = 0;

  bool is_root() const { return components.empty(); }
  bool exists() const { return existing_components == components.size(); }
};

class FsSandbox {
 public:
  // `memory_dir` must exist: its realpath is the root every path is checked against.
  explicit FsSandbox(const std::string& memory_dir);

  // Throws RequestError: bad_param (empty), path_escape (absolute, NUL, "..", outside the root
  // after realpath), symlink (any existing component). "." is memory/ itself.
  SandboxPath Resolve(const std::string& relative) const;

  // Throws RequestError not_slug for a component at or after `first` that is not a slug.
  static void CheckDirectoryNames(const SandboxPath& path, size_t first);
  static bool IsSlug(std::string_view name);
  // `skills` and `attachments`: scans skip them, so a node there would silently vanish.
  static bool IsReservedName(std::string_view name);
  // Any place.yaml, and memory/index.tsv and memory/README.md.
  static bool IsProcessOwned(const SandboxPath& path);

  const std::filesystem::path& root() const { return root_; }

 private:
  std::filesystem::path root_;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_MEMORY_FS_SANDBOX_H_
