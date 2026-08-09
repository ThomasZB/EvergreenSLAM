/**
 * @file yaml_utils.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/config/yaml_utils.h"

#include <algorithm>

namespace evergreenslam::utils::config {
namespace {

// Recursive rather than a loop over a reassigned Node: yaml-cpp's
// Node::operator= assigns into the node the variable already refers to instead
// of rebinding it, so `node = node[key]` does not walk down a level.
YAML::Node ResolvePath(const YAML::Node& root, const std::string& path) {
  if (path.empty()) {
    return root;
  }
  const size_t slash = path.find('/');
  if (slash == std::string::npos) {
    return Child(root, path);
  }
  const YAML::Node child = Child(root, path.substr(0, slash));
  if (!child) {
    return YAML::Node(YAML::NodeType::Undefined);
  }
  return ResolvePath(child, path.substr(slash + 1));
}

}  // namespace

std::vector<std::string> FindUnknownKeys(const YAML::Node& root, const Schema& schema) {
  std::vector<std::string> unknown;
  for (const auto& [path, known_keys] : schema) {
    const YAML::Node node = ResolvePath(root, path);
    if (!node || !node.IsMap()) {
      continue;
    }
    for (const auto& entry : node) {
      const std::string key = entry.first.as<std::string>();
      if (std::find(known_keys.begin(), known_keys.end(), key) == known_keys.end()) {
        unknown.push_back(path.empty() ? key : path + "/" + key);
      }
    }
  }
  return unknown;
}

}  // namespace evergreenslam::utils::config
