/**
 * @file place_store.cc
 * @author hang chen (chen@hang.plus)
 * @brief The memory/ tree as the process sees it: README install, place.yaml bindings, index.tsv.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/memory/place_store.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <map>

#include "common/file.h"
#include "service/http/request_error.h"
#include "service/memory/fs_sandbox.h"
#include "service/memory/memory_readme.h"
#include "utils/transform/transform.h"

namespace evergreenslam::agent {
namespace {

constexpr char kPlaceFileName[] = "place.yaml";

}  // namespace

std::vector<PlaceFile> PlaceScan::Unique() const {
  std::vector<PlaceFile> unique;
  for (const PlaceFile& place : places) {
    if (!std::binary_search(duplicate_paths.begin(), duplicate_paths.end(), place.path)) {
      unique.push_back(place);
    }
  }
  return unique;
}

PlaceStore::PlaceStore(const std::string& map_dir)
    : map_dir_(map_dir), memory_dir_((std::filesystem::path(map_dir) / "memory").string()) {}

bool PlaceStore::Install() const {
  std::error_code error;
  std::filesystem::create_directories(std::filesystem::path(memory_dir_) / "places", error);
  if (error) {
    LOG(ERROR) << "cannot create " << memory_dir_ << "/places: " << error.message();
    return false;
  }
  return common::WriteFileAtomically((std::filesystem::path(memory_dir_) / "README.md").string(),
                                     MemoryReadme());
}

PlaceScan PlaceStore::Scan() const {
  PlaceScan scan;
  const std::filesystem::path memory(memory_dir_);
  std::error_code error;
  std::filesystem::recursive_directory_iterator it(
      memory / "places", std::filesystem::directory_options::skip_permission_denied, error);
  const std::filesystem::recursive_directory_iterator end;
  for (; !error && it != end; it.increment(error)) {
    const std::filesystem::directory_entry& entry = *it;
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
    if (entry.path().filename() != kPlaceFileName || !entry.is_regular_file(status_error)) {
      continue;
    }
    ++scan.num_place_files;
    const std::optional<std::string> text = common::ReadFile(entry.path().string());
    const std::optional<lifelong::AnchorId> anchor =
        text.has_value() ? ParsePlaceYaml(*text) : std::nullopt;
    if (!anchor.has_value()) {
      LOG(WARNING) << "ignoring unparsable " << entry.path();
      continue;
    }
    scan.places.push_back(
        {entry.path().parent_path().lexically_relative(memory).generic_string(), *anchor});
  }
  std::sort(scan.places.begin(), scan.places.end(),
            [](const PlaceFile& a, const PlaceFile& b) { return a.path < b.path; });

  std::map<lifelong::AnchorId, int> uses;
  for (const PlaceFile& place : scan.places) {
    ++uses[place.anchor];
  }
  for (const PlaceFile& place : scan.places) {
    if (uses[place.anchor] > 1) {
      scan.duplicate_paths.push_back(place.path);
    }
  }
  return scan;
}

std::optional<lifelong::AnchorId> PlaceStore::ReadBinding(const std::string& node_path) const {
  const std::filesystem::path file =
      std::filesystem::path(memory_dir_) / node_path / kPlaceFileName;
  std::error_code error;
  if (std::filesystem::is_symlink(file, error)) {
    return std::nullopt;
  }
  const std::optional<std::string> text = common::ReadFile(file.string());
  return text.has_value() ? ParsePlaceYaml(*text) : std::nullopt;
}

bool PlaceStore::WriteBinding(const std::string& node_path, lifelong::AnchorId id) const {
  const std::filesystem::path directory = std::filesystem::path(memory_dir_) / node_path;
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    LOG(ERROR) << "cannot create " << directory << ": " << error.message();
    return false;
  }
  return common::WriteFileAtomically((directory / kPlaceFileName).string(),
                                     "anchor: " + std::to_string(id) + "\n");
}

bool PlaceStore::WriteIndex(const std::vector<PlaceRow>& rows, int num_solves) const {
  std::string text = "#path\tanchor\tstate\tx\ty\ttheta\tnum_solves\n";
  for (const PlaceRow& row : rows) {
    text += row.path + "\t" + std::to_string(row.anchor) + "\t" + row.state + "\t";
    if (row.pose.has_value()) {
      char pose[96];
      std::snprintf(pose, sizeof(pose), "%.3f\t%.3f\t%.4f", row.pose->translation().x(),
                    row.pose->translation().y(), utils::transform::GetYaw(*row.pose));
      text += pose;
    } else {
      text += "-\t-\t-";
    }
    text += "\t" + std::to_string(num_solves) + "\n";
  }
  return common::WriteFileAtomically((std::filesystem::path(memory_dir_) / "index.tsv").string(),
                                     text);
}

void PlaceStore::CheckNodePath(const std::string& node_path) {
  const std::string prefix = "places/";
  if (node_path.compare(0, prefix.size(), prefix) != 0 || node_path.size() == prefix.size()) {
    throw RequestError("bad_param", "node paths start with places/: " + node_path);
  }
  size_t begin = 0;
  while (begin < node_path.size()) {
    size_t end = node_path.find('/', begin);
    if (end == std::string::npos) {
      end = node_path.size();
    }
    const std::string component = node_path.substr(begin, end - begin);
    begin = end + 1;
    if (component.empty() || component == ".") {
      throw RequestError("bad_param", "empty path component: " + node_path);
    }
    if (component == "..") {
      throw RequestError("path_escape", "'..' in path: " + node_path);
    }
    if (FsSandbox::IsReservedName(component)) {
      throw RequestError("reserved_name", component + " is never a place: " + node_path);
    }
    if (!FsSandbox::IsSlug(component)) {
      throw RequestError("not_slug", "directory names match [a-z0-9][a-z0-9_-]*: " + node_path);
    }
  }
}

std::optional<lifelong::AnchorId> PlaceStore::ParsePlaceYaml(const std::string& text) {
  try {
    const YAML::Node node = YAML::Load(text);
    if (!node.IsMap() || !node["anchor"]) {
      return std::nullopt;
    }
    const lifelong::AnchorId id = node["anchor"].as<lifelong::AnchorId>();
    return id == 0 ? std::nullopt : std::optional<lifelong::AnchorId>(id);
  } catch (const YAML::Exception&) {
    return std::nullopt;
  }
}

}  // namespace evergreenslam::agent
