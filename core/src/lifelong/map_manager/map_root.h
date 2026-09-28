/**
 * @file map_root.h
 * @author hang chen (chen@hang.plus)
 * @brief The map root's layout: named map directories `<root>/<name>/` and the `current` file.
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_ROOT_H_
#define EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_ROOT_H_

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace evergreenslam::lifelong {

// Layout only: what a map directory holds is MapManager's.
class MapRoot {
 public:
  static constexpr const char* kDefaultName = "default";

  explicit MapRoot(std::string root);

  // [a-z0-9][a-z0-9_-]*, the same rule the agent layer uses for directory names.
  static bool IsValidName(std::string_view name);

  std::string Resolve(const std::string& name) const;
  bool Exists(const std::string& name) const;
  // Sorted; a missing root lists nothing.
  std::vector<std::string> List() const;
  std::optional<std::string> ReadCurrent() const;
  bool WriteCurrent(const std::string& name) const;
  // `requested` if given, else the `current` file, else kDefaultName.
  std::string ChooseAtBoot(const std::string& requested) const;

  const std::string& root() const { return root_; }

 private:
  std::string CurrentPath() const;

  std::string root_;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_MAP_MANAGER_MAP_ROOT_H_
