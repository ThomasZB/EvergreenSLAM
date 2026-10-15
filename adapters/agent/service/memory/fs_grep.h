/**
 * @file fs_grep.h
 * @author hang chen (chen@hang.plus)
 * @brief GET /fs/grep: fixed-string, case-insensitive line search over the agent's text files.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_MEMORY_FS_GREP_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_MEMORY_FS_GREP_H_

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "service/memory/fs_sandbox.h"

namespace evergreenslam::agent {

constexpr size_t kMaxGrepMatches = 200;
constexpr uintmax_t kMaxGrepFileBytes = 1 << 20;

struct GrepMatch {
  std::string path;
  int line = 0;
  std::string text;
};

struct GrepResult {
  std::vector<GrepMatch> matches;
  bool truncated = false;
};

std::vector<std::string> ParseGrepGlobs(std::string_view include);

GrepResult GrepTree(const SandboxPath& start, std::string_view pattern,
                    const std::vector<std::string>& globs);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_MEMORY_FS_GREP_H_
