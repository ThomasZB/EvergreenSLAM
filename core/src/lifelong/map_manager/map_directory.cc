/**
 * @file map_directory.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/map_manager/map_directory.h"

#include <fcntl.h>
#include <glog/logging.h>
#include <unistd.h>

#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "common/file.h"

namespace evergreenslam::lifelong {
namespace {

constexpr char kManifestFileName[] = "manifest.pb";
constexpr char kTempSuffix[] = ".tmp";

bool StartsWith(const std::string& text, const std::string& prefix) {
  return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

bool EndsWith(const std::string& text, const std::string& suffix) {
  return text.size() >= suffix.size() &&
         text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool IsSessionOrAnchorFile(const std::string& name) {
  return (StartsWith(name, "session_") || StartsWith(name, "anchors")) && EndsWith(name, ".pb");
}

// Makes the renames in the directory durable, not just the bytes they point at.
void SyncDirectory(const std::string& directory) {
  const int fd = ::open(directory.c_str(), O_RDONLY);
  if (fd >= 0) {
    ::fsync(fd);
    ::close(fd);
  }
}

std::vector<std::string> TopLevelFiles(const std::string& directory) {
  std::vector<std::string> names;
  std::error_code error;
  for (const auto& item : std::filesystem::directory_iterator(directory, error)) {
    std::error_code type_error;
    if (item.is_regular_file(type_error)) {
      names.push_back(item.path().filename().string());
    }
  }
  LOG_IF(ERROR, error) << "cannot list " << directory << ": " << error.message();
  return names;
}

}  // namespace

MapDirectory::MapDirectory(std::string directory) : directory_(std::move(directory)) {
  std::error_code error;
  std::filesystem::create_directories(directory_, error);
  LOG_IF(ERROR, error) << "cannot create map directory " << directory_ << ": " << error.message();
}

MapDirectory::Commit MapDirectory::Begin() const {
  Commit commit;
  commit.generation = generation_ + 1;
  return commit;
}

std::optional<std::string> MapDirectory::Write(Commit& commit, const std::string& stem,
                                               const std::string& contents) {
  const std::string file_name = stem + ".g" + std::to_string(commit.generation) + ".pb";
  if (!common::WriteFileAtomically(PathOf(file_name), contents)) {
    LOG(ERROR) << file_name << " not written, commit " << commit.generation << " abandoned";
    return std::nullopt;
  }
  commit.written.push_back(file_name);
  return file_name;
}

bool MapDirectory::Publish(const Commit& commit, const std::string& manifest) {
  // The new names must be durable before a manifest that points at them.
  SyncDirectory(directory_);
  if (before_manifest_write_hook_) {
    before_manifest_write_hook_();
  }
  if (!common::WriteFileAtomically(PathOf(kManifestFileName), manifest)) {
    LOG(ERROR) << "manifest not written, commit " << commit.generation << " abandoned";
    Abandon(commit);
    return false;
  }
  SyncDirectory(directory_);
  generation_ = commit.generation;
  Remove(commit.superseded);
  return true;
}

void MapDirectory::Abandon(const Commit& commit) const { Remove(commit.written); }

std::optional<std::string> MapDirectory::ReadManifest() const { return Read(kManifestFileName); }

std::optional<std::string> MapDirectory::Read(const std::string& file_name) const {
  return common::ReadFile(PathOf(file_name));
}

bool MapDirectory::HasCommitFiles() const {
  for (const std::string& name : TopLevelFiles(directory_)) {
    if (IsSessionOrAnchorFile(name)) {
      return true;
    }
  }
  return false;
}

void MapDirectory::RemoveUnreferenced(const std::set<std::string>& referenced) const {
  for (const std::string& name : TopLevelFiles(directory_)) {
    if ((IsSessionOrAnchorFile(name) || EndsWith(name, kTempSuffix)) &&
        referenced.count(name) == 0) {
      LOG(INFO) << "removing " << name << ", not referenced by the manifest";
      Remove({name});
    }
  }
}

std::string MapDirectory::PathOf(const std::string& file_name) const {
  return (std::filesystem::path(directory_) / file_name).string();
}

void MapDirectory::Remove(const std::vector<std::string>& file_names) const {
  for (const std::string& file_name : file_names) {
    std::error_code error;
    std::filesystem::remove(PathOf(file_name), error);
    LOG_IF(WARNING, error) << "cannot remove " << file_name << ": " << error.message();
  }
}

}  // namespace evergreenslam::lifelong
