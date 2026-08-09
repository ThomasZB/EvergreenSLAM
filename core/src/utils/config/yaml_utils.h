/**
 * @file yaml_utils.h
 * @author hang chen (chen@hang.plus)
 * @brief Shared helpers for the LoadXxxOption() functions.
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_CONFIG_YAML_UTILS_H_
#define EVERGREENSLAM_UTILS_CONFIG_YAML_UTILS_H_

#include <yaml-cpp/yaml.h>

#include <string>
#include <utility>
#include <vector>

namespace evergreenslam::utils::config {

// A missing key keeps the default, so a config only has to carry what it
// overrides and an option can be added without breaking existing files.
template <typename T>
T LoadOption(const YAML::Node& node, const std::string& key, const T& default_value) {
  if (node && node[key]) {
    return node[key].as<T>();
  }
  return default_value;
}

inline YAML::Node Child(const YAML::Node& node, const std::string& key) {
  if (node && node[key]) {
    return node[key];
  }
  return YAML::Node(YAML::NodeType::Undefined);
}

// Section path relative to the root ("" for the root itself) and the keys that
// section understands, sub-section names included.
using Schema = std::vector<std::pair<std::string, std::vector<std::string>>>;

// Every key present in `root` that no option reads, reported as "section/key".
// Needed because LoadOption() falls back to the default on a missing key, so a
// misspelled option is otherwise accepted in silence and simply does nothing.
std::vector<std::string> FindUnknownKeys(const YAML::Node& root, const Schema& schema);

}  // namespace evergreenslam::utils::config

#endif  // EVERGREENSLAM_UTILS_CONFIG_YAML_UTILS_H_
