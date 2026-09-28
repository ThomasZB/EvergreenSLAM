/**
 * @file maps_endpoints.cc
 * @author hang chen (chen@hang.plus)
 * @brief /maps, /maps/new, /maps/open: the map root's named maps and the host's switch request.
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <filesystem>
#include <optional>
#include <string>
#include <system_error>

#include "lifelong/map_manager/map_root.h"
#include "service/endpoints.h"
#include "service/http_reply.h"

namespace evergreenslam::agent {
namespace {

using lifelong::MapRoot;

void SendRefusal(httplib::Response& response, const std::string& reason) {
  JsonWriter writer = BeginReceipt(reason);
  writer.EndObject();
  SendJson(response, writer);
}

void SendSwitch(httplib::Response& response, const std::string& name, bool pending) {
  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Field("map", name).Field("pending", pending).EndObject();
  SendJson(response, writer);
}

std::string RequiredName(const httplib::Request& request) {
  const std::string name = RequiredParam(request, "name");
  if (!MapRoot::IsValidName(name)) {
    throw RequestError("not_slug", "map names match [a-z0-9][a-z0-9_-]*: " + name);
  }
  return name;
}

bool CanSwitch(const ServiceContext& context) {
  return !context.map_root.empty() && context.hooks.switch_map != nullptr;
}

void HandleList(const ServiceContext& context, httplib::Response& response) {
  if (context.map_root.empty()) {
    SendRefusal(response, "not_supported");
    return;
  }
  std::error_code error;
  const std::string root = std::filesystem::absolute(context.map_root, error).string();
  JsonWriter writer = BeginReceipt(std::nullopt);
  writer.Field("map_root", root).Field("current", context.map_name).Key("maps").BeginArray();
  for (const std::string& name : MapRoot(context.map_root).List()) {
    writer.BeginObject().Field("name", name).Field("current", name == context.map_name).EndObject();
  }
  writer.EndArray().EndObject();
  SendJson(response, writer);
}

void HandleNew(ServiceContext& context, const httplib::Request& request,
               httplib::Response& response) {
  const std::string name = RequiredName(request);
  if (!CanSwitch(context)) {
    SendRefusal(response, "not_supported");
    return;
  }
  // Answering against the map being left would be wrong, and a second request would replace
  // the first.
  if (context.switch_pending.load()) {
    SendRefusal(response, "switching");
    return;
  }
  const MapRoot root(context.map_root);
  if (root.Exists(name)) {
    SendRefusal(response, "exists");
    return;
  }
  // Created now, so a repeated `new` answers `exists` before the switch lands.
  std::error_code error;
  std::filesystem::create_directories(root.Resolve(name), error);
  if (error) {
    SendRefusal(response, "fs_error");
    return;
  }
  if (!context.hooks.switch_map(MapSwitchRequest{name, true})) {
    SendRefusal(response, "not_supported");
    return;
  }
  context.switch_pending = true;
  SendSwitch(response, name, true);
}

void HandleOpen(ServiceContext& context, const httplib::Request& request,
                httplib::Response& response) {
  const std::string name = RequiredName(request);
  if (!CanSwitch(context)) {
    SendRefusal(response, "not_supported");
    return;
  }
  // Answering against the map being left would be wrong, and a second request would replace
  // the first.
  if (context.switch_pending.load()) {
    SendRefusal(response, "switching");
    return;
  }
  if (!MapRoot(context.map_root).Exists(name)) {
    SendRefusal(response, "unknown_map");
    return;
  }
  if (name == context.map_name) {
    SendSwitch(response, name, false);
    return;
  }
  if (!context.hooks.switch_map(MapSwitchRequest{name, false})) {
    SendRefusal(response, "not_supported");
    return;
  }
  context.switch_pending = true;
  SendSwitch(response, name, true);
}

}  // namespace

void RegisterMapsEndpoints(httplib::Server& server, ServiceContext& context) {
  server.Get("/maps", Guarded([&context](const httplib::Request&, httplib::Response& response) {
               HandleList(context, response);
             }));
  server.Post("/maps/new",
              Guarded([&context](const httplib::Request& request, httplib::Response& response) {
                HandleNew(context, request, response);
              }));
  server.Post("/maps/open",
              Guarded([&context](const httplib::Request& request, httplib::Response& response) {
                HandleOpen(context, request, response);
              }));
}

}  // namespace evergreenslam::agent
