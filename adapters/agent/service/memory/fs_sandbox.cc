/**
 * @file fs_sandbox.cc
 * @author hang chen (chen@hang.plus)
 * @brief Path rules for everything the service touches under memory/: no escape, no symlinks.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/memory/fs_sandbox.h"

#include <glog/logging.h>
#include <sys/stat.h>

#include <algorithm>
#include <cctype>
#include <cerrno>

#include "service/http/request_error.h"

namespace evergreenslam::agent {
namespace {

bool IsInside(const std::filesystem::path& root, const std::filesystem::path& path) {
  const std::string root_text = root.string();
  const std::string text = path.string();
  return text == root_text ||
         (text.size() > root_text.size() && text.compare(0, root_text.size(), root_text) == 0 &&
          text[root_text.size()] == '/');
}

}  // namespace

FsSandbox::FsSandbox(const std::string& memory_dir) {
  std::error_code error;
  root_ = std::filesystem::canonical(memory_dir, error);
  CHECK(!error) << "memory directory " << memory_dir << ": " << error.message();
}

SandboxPath FsSandbox::Resolve(const std::string& relative) const {
  if (relative.empty()) {
    throw RequestError("bad_param", "empty path");
  }
  if (relative.find('\0') != std::string::npos) {
    throw RequestError("path_escape", "NUL in path");
  }
  if (relative.front() == '/') {
    throw RequestError("path_escape", "absolute path: " + relative);
  }
  SandboxPath path;
  size_t begin = 0;
  while (begin <= relative.size()) {
    size_t end = relative.find('/', begin);
    if (end == std::string::npos) {
      end = relative.size();
    }
    const std::string component = relative.substr(begin, end - begin);
    begin = end + 1;
    if (component.empty() || component == ".") {
      continue;
    }
    if (component == "..") {
      throw RequestError("path_escape", "'..' in path: " + relative);
    }
    path.components.push_back(component);
  }

  path.absolute = root_;
  bool missing = false;
  for (const std::string& component : path.components) {
    path.absolute /= component;
    if (missing) {
      continue;
    }
    struct stat status;
    if (::lstat(path.absolute.c_str(), &status) != 0) {
      missing = true;
      continue;
    }
    if (S_ISLNK(status.st_mode)) {
      throw RequestError("symlink", "symbolic link in path: " + relative);
    }
    ++path.existing_components;
  }
  for (const std::string& component : path.components) {
    path.relative += (path.relative.empty() ? "" : "/") + component;
  }

  std::filesystem::path deepest = root_;
  for (size_t i = 0; i < path.existing_components; ++i) {
    deepest /= path.components[i];
  }
  std::error_code error;
  const std::filesystem::path real = std::filesystem::canonical(deepest, error);
  if (error || !IsInside(root_, real)) {
    throw RequestError("path_escape", "outside memory/: " + relative);
  }
  return path;
}

void FsSandbox::CheckDirectoryNames(const SandboxPath& path, size_t first) {
  for (size_t i = first; i < path.components.size(); ++i) {
    if (!IsSlug(path.components[i])) {
      throw RequestError("not_slug",
                         "directory names match [a-z0-9][a-z0-9_-]*: " + path.components[i]);
    }
  }
}

bool FsSandbox::IsReservedName(std::string_view name) {
  return name == "skills" || name == "attachments";
}

bool FsSandbox::IsSlug(std::string_view name) {
  if (name.empty()) {
    return false;
  }
  for (size_t i = 0; i < name.size(); ++i) {
    const char c = name[i];
    const bool alnum = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
    if (!alnum && (i == 0 || (c != '_' && c != '-'))) {
      return false;
    }
  }
  return true;
}

// Case-folded: on a case-insensitive filesystem "Place.yaml" is place.yaml.
bool FsSandbox::IsProcessOwned(const SandboxPath& path) {
  if (path.components.empty()) {
    return false;
  }
  const auto lower = [](std::string name) {
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return name;
  };
  if (lower(path.components.back()) == "place.yaml") {
    return true;
  }
  return path.components.size() == 1 &&
         (lower(path.components[0]) == "index.tsv" || lower(path.components[0]) == "readme.md");
}

}  // namespace evergreenslam::agent
