/**
 * @file file.h
 * @author hang chen (chen@hang.plus)
 * @brief Whole-file reads and crash-safe atomic writes.
 * @version 0.1
 * @date 2026-08-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_COMMON_FILE_H_
#define EVERGREENSLAM_COMMON_FILE_H_

#include <optional>
#include <string>

namespace evergreenslam::common {

bool WriteFileAtomically(const std::string& path, const std::string& contents);

std::optional<std::string> ReadFile(const std::string& path);

}  // namespace evergreenslam::common

#endif  // EVERGREENSLAM_COMMON_FILE_H_
