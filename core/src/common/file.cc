/**
 * @file file.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "common/file.h"

#include <fcntl.h>
#include <glog/logging.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

namespace evergreenslam::common {
namespace {

constexpr char kTempSuffix[] = ".tmp";

}  // namespace

bool WriteFileAtomically(const std::string& path, const std::string& contents) {
  const std::string temp_path = path + kTempSuffix;
  const auto fail = [&temp_path](const std::string& what) {
    LOG(ERROR) << what;
    std::error_code error;
    std::filesystem::remove(temp_path, error);
    return false;
  };
  {
    std::ofstream stream(temp_path, std::ios::binary | std::ios::trunc);
    if (!stream) {
      LOG(ERROR) << "cannot open " << temp_path << " for writing";
      return false;
    }
    stream.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    stream.close();
    if (stream.fail()) {
      return fail("short write to " + temp_path);
    }
  }
  // The rename is only atomic with respect to a crash if the bytes are on the device first.
  const int fd = ::open(temp_path.c_str(), O_RDONLY);
  if (fd < 0) {
    return fail("cannot reopen " + temp_path + ": " + std::strerror(errno));
  }
  const int synced = ::fsync(fd);
  const int sync_errno = errno;
  ::close(fd);
  if (synced != 0) {
    return fail("cannot fsync " + temp_path + ": " + std::strerror(sync_errno));
  }
  std::error_code error;
  std::filesystem::rename(temp_path, path, error);
  if (error) {
    return fail("cannot rename " + temp_path + " to " + path + ": " + error.message());
  }
  return true;
}

std::optional<std::string> ReadFile(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    return std::nullopt;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

}  // namespace evergreenslam::common
