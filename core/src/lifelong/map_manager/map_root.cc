/**
 * @file map_root.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/map_manager/map_root.h"

#include <glog/logging.h>

#include <algorithm>
#include <filesystem>
#include <system_error>
#include <utility>

#include "common/file.h"

namespace evergreenslam::lifelong {
namespace {

constexpr char kCurrentFileName[] = "current";

bool IsNameStart(char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'); }

std::string Trim(const std::string& text) {
  const auto is_space = [](char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; };
  size_t begin = 0;
  size_t end = text.size();
  while (begin < end && is_space(text[begin])) {
    ++begin;
  }
  while (end > begin && is_space(text[end - 1])) {
    --end;
  }
  return text.substr(begin, end - begin);
}

}  // namespace

MapRoot::MapRoot(std::string root) : root_(std::move(root)) {}

bool MapRoot::IsValidName(std::string_view name) {
  if (name.empty() || !IsNameStart(name.front())) {
    return false;
  }
  return std::all_of(name.begin(), name.end(),
                     [](char c) { return IsNameStart(c) || c == '_' || c == '-'; });
}

std::string MapRoot::Resolve(const std::string& name) const {
  return (std::filesystem::path(root_) / name).string();
}

bool MapRoot::Exists(const std::string& name) const {
  std::error_code error;
  return std::filesystem::is_directory(Resolve(name), error);
}

std::vector<std::string> MapRoot::List() const {
  std::vector<std::string> names;
  std::error_code error;
  std::filesystem::directory_iterator it(root_, error);
  if (error) {
    return names;
  }
  for (const std::filesystem::directory_entry& entry : it) {
    const std::string name = entry.path().filename().string();
    if (IsValidName(name) && entry.is_directory(error)) {
      names.push_back(name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::optional<std::string> MapRoot::ReadCurrent() const {
  const std::optional<std::string> contents = common::ReadFile(CurrentPath());
  if (!contents.has_value()) {
    return std::nullopt;
  }
  std::string name = Trim(*contents);
  if (!IsValidName(name)) {
    return std::nullopt;
  }
  return name;
}

bool MapRoot::WriteCurrent(const std::string& name) const {
  std::error_code error;
  std::filesystem::create_directories(root_, error);
  if (error) {
    LOG(ERROR) << "cannot create map root " << root_ << ": " << error.message();
    return false;
  }
  return common::WriteFileAtomically(CurrentPath(), name + "\n");
}

std::string MapRoot::ChooseAtBoot(const std::string& requested) const {
  if (!requested.empty()) {
    return requested;
  }
  return ReadCurrent().value_or(kDefaultName);
}

std::string MapRoot::CurrentPath() const {
  return (std::filesystem::path(root_) / kCurrentFileName).string();
}

}  // namespace evergreenslam::lifelong
