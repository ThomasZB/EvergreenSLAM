/**
 * @file fs_endpoints.cc
 * @author hang chen (chen@hang.plus)
 * @brief The /fs/ endpoints: memory/ over HTTP for agents without the directory; no index, no
 * search.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "common/file.h"
#include "service/http/endpoints.h"
#include "service/http/http_reply.h"
#include "service/memory/fs_sandbox.h"

namespace evergreenslam::agent {
namespace {

namespace fs = std::filesystem;

constexpr uintmax_t kMaxCatBytes = 1 << 20;
constexpr int kMaxTreeDepth = 6;
constexpr int kDefaultTreeDepth = 3;

struct Entry {
  std::string name;
  bool directory = false;
  uintmax_t size = 0;
};

// lstat, so a symlink is listed as what it is and never followed.
std::vector<Entry> ListDirectory(const fs::path& directory) {
  std::vector<Entry> entries;
  std::error_code error;
  for (fs::directory_iterator it(directory, error), end; !error && it != end; it.increment(error)) {
    struct stat status;
    if (::lstat(it->path().c_str(), &status) != 0) {
      continue;
    }
    entries.push_back({it->path().filename().string(), S_ISDIR(status.st_mode),
                       S_ISREG(status.st_mode) ? static_cast<uintmax_t>(status.st_size) : 0});
  }
  std::sort(entries.begin(), entries.end(),
            [](const Entry& a, const Entry& b) { return a.name < b.name; });
  return entries;
}

bool IsDirectory(const fs::path& path) {
  struct stat status;
  return ::lstat(path.c_str(), &status) == 0 && S_ISDIR(status.st_mode);
}

// A place.yaml or node.yaml at `path` or anywhere below it; symlinks are neither followed nor
// counted, as PlaceStore::Scan skips them.
bool HoldsNodeFiles(const fs::path& path) {
  const auto is_node_file = [](const fs::path& file) {
    return file.filename() == "place.yaml" || file.filename() == "node.yaml";
  };
  if (!IsDirectory(path)) {
    return is_node_file(path);
  }
  std::error_code error;
  for (fs::recursive_directory_iterator
           it(path, fs::directory_options::skip_permission_denied, error),
       end;
       !error && it != end; it.increment(error)) {
    std::error_code status_error;
    if (!it->is_symlink(status_error) && it->is_regular_file(status_error) &&
        is_node_file(it->path())) {
      return true;
    }
  }
  return false;
}

void SendRefusal(httplib::Response& response, const std::string& reason,
                 const std::string& detail) {
  JsonWriter writer = BeginReceipt(reason);
  writer.Field("detail", detail).EndObject();
  SendJson(response, writer);
}

void SendOk(httplib::Response& response) {
  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.EndObject();
  SendJson(response, writer);
}

void RejectOwned(const SandboxPath& path) {
  if (FsSandbox::IsProcessOwned(path)) {
    throw RequestError("owned_by_process", path.relative + " is written by the SLAM process");
  }
}

void AppendTree(const fs::path& directory, const std::string& prefix, int depth, int max_depth,
                std::string& out, int& num_directories, int& num_files) {
  const std::vector<Entry> entries = ListDirectory(directory);
  for (size_t i = 0; i < entries.size(); ++i) {
    const bool last = i + 1 == entries.size();
    out += prefix + (last ? "└── " : "├── ") + entries[i].name + "\n";
    if (entries[i].directory) {
      ++num_directories;
      const std::string child_prefix = prefix + (last ? "    " : "│   ");
      if (depth < max_depth) {
        AppendTree(directory / entries[i].name, child_prefix, depth + 1, max_depth, out,
                   num_directories, num_files);
      } else if (!ListDirectory(directory / entries[i].name).empty()) {
        out += child_prefix + "└── …\n";
      }
    } else {
      ++num_files;
    }
  }
}

void HandleList(const ServiceContext& context, const httplib::Request& request,
                httplib::Response& response) {
  const SandboxPath path = context.sandbox.Resolve(RequiredParam(request, "path"));
  if (!IsDirectory(path.absolute)) {
    SendRefusal(response, "fs_error", "not a directory: " + path.relative);
    return;
  }
  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Key("entries").BeginArray();
  for (const Entry& entry : ListDirectory(path.absolute)) {
    writer.BeginObject()
        .Field("name", entry.name)
        .Field("type", entry.directory ? "dir" : "file")
        .Field("size", static_cast<int64_t>(entry.size))
        .EndObject();
  }
  writer.EndArray().EndObject();
  SendJson(response, writer);
}

void HandleTree(const ServiceContext& context, const httplib::Request& request,
                httplib::Response& response) {
  const SandboxPath path = context.sandbox.Resolve(RequiredParam(request, "path"));
  const std::optional<std::string> depth_param = OptionalParam(request, "depth");
  const int64_t depth =
      depth_param.has_value() ? ParseInt("depth", *depth_param) : kDefaultTreeDepth;
  if (depth < 1 || depth > kMaxTreeDepth) {
    throw RequestError("bad_param", "depth is 1 to 6");
  }
  if (!IsDirectory(path.absolute)) {
    SendRefusal(response, "fs_error", "not a directory: " + path.relative);
    return;
  }
  std::string out = (path.is_root() ? "." : path.relative) + "\n";
  int num_directories = 0;
  int num_files = 0;
  AppendTree(path.absolute, "", 1, static_cast<int>(depth), out, num_directories, num_files);
  out += "\n" + std::to_string(num_directories) + " directories, " + std::to_string(num_files) +
         " files\n";
  response.set_content(out, "text/plain; charset=utf-8");
}

void HandleCat(const ServiceContext& context, const httplib::Request& request,
               httplib::Response& response) {
  const SandboxPath path = context.sandbox.Resolve(RequiredParam(request, "path"));
  if (!path.exists()) {
    SendError(response, 404, "not_found", "no such file: " + path.relative);
    return;
  }
  struct stat status;
  if (::lstat(path.absolute.c_str(), &status) != 0 || !S_ISREG(status.st_mode)) {
    SendRefusal(response, "fs_error", "not a file: " + path.relative);
    return;
  }
  if (static_cast<uintmax_t>(status.st_size) > kMaxCatBytes) {
    throw RequestError("too_large", path.relative + " is over 1 MiB");
  }
  const std::optional<std::string> contents = common::ReadFile(path.absolute.string());
  if (!contents.has_value()) {
    SendRefusal(response, "fs_error", "cannot read " + path.relative);
    return;
  }
  response.set_content(*contents, "application/octet-stream");
}

void HandleWrite(const ServiceContext& context, const httplib::Request& request,
                 httplib::Response& response) {
  const SandboxPath path = context.sandbox.Resolve(RequiredParam(request, "path"));
  if (path.is_root()) {
    throw RequestError("path_escape", "cannot write memory/ itself");
  }
  RejectOwned(path);
  if (!IsDirectory(path.absolute.parent_path())) {
    SendRefusal(response, "fs_error", "parent directory missing: " + path.relative);
    return;
  }
  if (IsDirectory(path.absolute) ||
      !common::WriteFileAtomically(path.absolute.string(), request.body)) {
    SendRefusal(response, "fs_error", "cannot write " + path.relative);
    return;
  }
  SendOk(response);
}

void HandleAppend(const ServiceContext& context, const httplib::Request& request,
                  httplib::Response& response) {
  const SandboxPath path = context.sandbox.Resolve(RequiredParam(request, "path"));
  if (path.is_root()) {
    throw RequestError("path_escape", "cannot append to memory/ itself");
  }
  RejectOwned(path);
  const int fd = ::open(path.absolute.c_str(), O_WRONLY | O_APPEND | O_CREAT | O_NOFOLLOW, 0644);
  if (fd < 0) {
    SendRefusal(response, "fs_error", path.relative + ": " + std::strerror(errno));
    return;
  }
  const ssize_t written = ::write(fd, request.body.data(), request.body.size());
  ::close(fd);
  if (written != static_cast<ssize_t>(request.body.size())) {
    SendRefusal(response, "fs_error", "short append to " + path.relative);
    return;
  }
  SendOk(response);
}

void HandleMkdir(const ServiceContext& context, const httplib::Request& request,
                 httplib::Response& response) {
  const SandboxPath path = context.sandbox.Resolve(RequiredParam(request, "path"));
  // Every component, not just the missing ones: a case-insensitive filesystem finds "Kitchen"
  // as the existing "kitchen".
  FsSandbox::CheckDirectoryNames(path, 0);
  std::error_code error;
  fs::create_directories(path.absolute, error);
  if (error || !IsDirectory(path.absolute)) {
    SendRefusal(response, "fs_error", path.relative + ": " + error.message());
    return;
  }
  SendOk(response);
}

void HandleMove(const ServiceContext& context, const httplib::Request& request,
                httplib::Response& response) {
  const SandboxPath from = context.sandbox.Resolve(RequiredParam(request, "from"));
  const SandboxPath to = context.sandbox.Resolve(RequiredParam(request, "to"));
  if (from.is_root() || to.is_root()) {
    throw RequestError("path_escape", "cannot move memory/ itself");
  }
  if (!from.exists()) {
    SendRefusal(response, "fs_error", "no such file or directory: " + from.relative);
    return;
  }
  const bool directory = IsDirectory(from.absolute);
  // A directory holding place.yaml moves with it: that is how a place is renamed.
  if (!directory) {
    RejectOwned(from);
  }
  RejectOwned(to);
  if (directory) {
    FsSandbox::CheckDirectoryNames(to, to.components.size() - 1);
  }
  // Scans skip skills/ and attachments/ subtrees: a place or thing moved there would vanish.
  if (std::any_of(to.components.begin(), to.components.end(),
                  [](const std::string& c) { return FsSandbox::IsReservedName(c); }) &&
      HoldsNodeFiles(from.absolute)) {
    throw RequestError("reserved_name",
                       "skills and attachments never hold place.yaml or "
                       "node.yaml: " +
                           from.relative + " -> " + to.relative);
  }
  if (to.exists()) {
    SendRefusal(response, "exists", to.relative + " exists");
    return;
  }
  if (::rename(from.absolute.c_str(), to.absolute.c_str()) != 0) {
    SendRefusal(response, "fs_error",
                from.relative + " -> " + to.relative + ": " + std::strerror(errno));
    return;
  }
  SendOk(response);
}

void HandleRemove(const ServiceContext& context, const httplib::Request& request,
                  httplib::Response& response) {
  const SandboxPath path = context.sandbox.Resolve(RequiredParam(request, "path"));
  const bool recursive = BoolParam(request, "recursive", false);
  if (path.is_root()) {
    throw RequestError("path_escape", "cannot remove memory/ itself");
  }
  if (!path.exists()) {
    SendRefusal(response, "fs_error", "no such file or directory: " + path.relative);
    return;
  }
  std::error_code error;
  if (IsDirectory(path.absolute)) {
    if (!ListDirectory(path.absolute).empty() && !recursive) {
      SendRefusal(response, "not_empty", path.relative + " is not empty; pass recursive=1");
      return;
    }
    fs::remove_all(path.absolute, error);
  } else {
    RejectOwned(path);
    fs::remove(path.absolute, error);
  }
  if (error) {
    SendRefusal(response, "fs_error", path.relative + ": " + error.message());
    return;
  }
  SendOk(response);
}

}  // namespace

void RegisterFsEndpoints(httplib::Server& server, ServiceContext& context) {
  const auto route = [&context](auto handler) {
    return Guarded(
        [&context, handler](const httplib::Request& request, httplib::Response& response) {
          handler(context, request, response);
        });
  };
  server.Get("/fs/ls", route(HandleList));
  server.Get("/fs/tree", route(HandleTree));
  server.Get("/fs/cat", route(HandleCat));
  server.Post("/fs/write", route(HandleWrite));
  server.Post("/fs/append", route(HandleAppend));
  server.Post("/fs/mkdir", route(HandleMkdir));
  server.Post("/fs/mv", route(HandleMove));
  server.Post("/fs/rm", route(HandleRemove));
}

}  // namespace evergreenslam::agent
