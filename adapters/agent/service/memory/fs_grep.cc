/**
 * @file fs_grep.cc
 * @author hang chen (chen@hang.plus)
 * @brief GET /fs/grep: fixed-string, case-insensitive line search over the agent's text files.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/memory/fs_grep.h"

#include <fnmatch.h>
#include <sys/stat.h>

#include <algorithm>
#include <filesystem>
#include <optional>

#include "common/file.h"

namespace evergreenslam::agent {
namespace {

namespace fs = std::filesystem;

const std::vector<std::string> kDefaultGlobs = {"node.yaml", "notes.md", "SKILL.md", "*.md",
                                                "*.yaml"};

char AsciiLower(char c) { return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c; }

std::string AsciiLower(std::string_view text) {
  std::string out(text);
  std::transform(out.begin(), out.end(), out.begin(), [](char c) { return AsciiLower(c); });
  return out;
}

class Grep {
 public:
  Grep(std::string_view pattern, const std::vector<std::string>& globs)
      : needle_(AsciiLower(pattern)), globs_(globs) {}

  bool Walk(const fs::path& directory, const std::string& relative) {
    std::vector<std::string> names;
    std::error_code error;
    for (fs::directory_iterator it(directory, error), end; !error && it != end;
         it.increment(error)) {
      names.push_back(it->path().filename().string());
    }
    std::sort(names.begin(), names.end());
    for (const std::string& name : names) {
      const fs::path child = directory / name;
      const std::string child_relative = relative.empty() ? name : relative + "/" + name;
      struct stat status;
      if (::lstat(child.c_str(), &status) != 0 || S_ISLNK(status.st_mode)) {
        continue;
      }
      if (S_ISDIR(status.st_mode)) {
        if (name != "attachments" && !Walk(child, child_relative)) {
          return false;
        }
      } else if (S_ISREG(status.st_mode) && Included(name) &&
                 !File(child, child_relative, status)) {
        return false;
      }
    }
    return true;
  }

  bool File(const fs::path& file, const std::string& relative, const struct stat& status) {
    if (file.filename() == "observations.jsonl" ||
        static_cast<uintmax_t>(status.st_size) > kMaxGrepFileBytes) {
      return true;
    }
    const std::optional<std::string> contents = common::ReadFile(file.string());
    if (!contents.has_value()) {
      return true;
    }
    int line_number = 0;
    size_t begin = 0;
    while (begin < contents->size()) {
      size_t end = contents->find('\n', begin);
      if (end == std::string::npos) {
        end = contents->size();
      }
      ++line_number;
      std::string_view line(contents->data() + begin, end - begin);
      begin = end + 1;
      if (!line.empty() && line.back() == '\r') {
        line.remove_suffix(1);
      }
      const auto found = std::search(line.begin(), line.end(), needle_.begin(), needle_.end(),
                                     [](char a, char b) { return AsciiLower(a) == b; });
      if (found == line.end()) {
        continue;
      }
      if (result_.matches.size() == kMaxGrepMatches) {
        result_.truncated = true;
        return false;
      }
      result_.matches.push_back({relative, line_number, std::string(line)});
    }
    return true;
  }

  GrepResult Take() { return std::move(result_); }

 private:
  bool Included(const std::string& name) const {
    return std::any_of(globs_.begin(), globs_.end(), [&name](const std::string& glob) {
      return ::fnmatch(glob.c_str(), name.c_str(), 0) == 0;
    });
  }

  const std::string needle_;
  const std::vector<std::string>& globs_;
  GrepResult result_;
};

}  // namespace

std::vector<std::string> ParseGrepGlobs(std::string_view include) {
  std::vector<std::string> globs;
  size_t begin = 0;
  while (begin <= include.size()) {
    size_t end = include.find(',', begin);
    if (end == std::string_view::npos) {
      end = include.size();
    }
    std::string_view item = include.substr(begin, end - begin);
    begin = end + 1;
    while (!item.empty() && item.front() == ' ') {
      item.remove_prefix(1);
    }
    while (!item.empty() && item.back() == ' ') {
      item.remove_suffix(1);
    }
    if (!item.empty()) {
      globs.emplace_back(item);
    }
  }
  return globs.empty() ? kDefaultGlobs : globs;
}

GrepResult GrepTree(const SandboxPath& start, std::string_view pattern,
                    const std::vector<std::string>& globs) {
  Grep grep(pattern, globs);
  struct stat status;
  if (::lstat(start.absolute.c_str(), &status) == 0) {
    if (S_ISDIR(status.st_mode)) {
      grep.Walk(start.absolute, start.relative);
    } else if (S_ISREG(status.st_mode)) {
      grep.File(start.absolute, start.relative, status);
    }
  }
  return grep.Take();
}

}  // namespace evergreenslam::agent
