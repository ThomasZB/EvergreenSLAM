/**
 * @file agent_service.cc
 * @author hang chen (chen@hang.plus)
 * @brief The agent layer's HTTP server: PoseGraph operations and the memory/ tree for `egs`.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/agent_service.h"

#include <glog/logging.h>

#include <cstdio>
#include <filesystem>
#include <mutex>
#include <random>
#include <thread>
#include <utility>

#include "common/file.h"
#include "service/http/endpoints.h"
#include "service/http/http_reply.h"
#include "service/http/httplib_include.h"
#include "service/http/routes.h"
#include "service/http/serial_executor.h"
#include "service/http/service_context.h"

namespace evergreenslam::agent {
namespace {

constexpr size_t kMaxRequestBytes = 8 << 20;
constexpr char kApiVersion[] = "0.3";

std::string NewInstanceId() {
  std::random_device device;
  std::string id;
  for (int i = 0; i < 4; ++i) {
    char word[9];
    std::snprintf(word, sizeof(word), "%08x", static_cast<unsigned>(device()));
    id += word;
  }
  return id;
}

constexpr int kPoolThreads = 8;
// At most 1 + 4 pool threads ever sit in serial handlers, so /pose always finds a free one.
constexpr int kMaxSerialWaiting = 4;

}  // namespace

struct AgentService::Impl {
  Impl(lifelong::PoseGraph& pose_graph, AgentServiceHooks hooks, PlaceStore places,
       const AgentServiceOption& option)
      : context(pose_graph, std::move(hooks), std::move(places), option.map_root, option.map_name) {
  }

  ServiceContext context;
  const std::string instance_id = NewInstanceId();
  SerialExecutor serial{kMaxSerialWaiting};
  httplib::Server server;
  std::thread listener;
  int port = -1;
  std::once_flag stop_once;
};

namespace {

PlaceStore InstalledPlaceStore(const lifelong::PoseGraph& pose_graph) {
  CHECK(pose_graph.map_manager() != nullptr)
      << "the agent service needs a map directory: construct it only with a map_manager()";
  PlaceStore places(pose_graph.map_manager()->directory());
  if (!places.Install()) {
    LOG(ERROR) << "could not install " << places.memory_dir() << "/README.md";
  }
  return places;
}

void WriteInstanceFile(const ServiceContext& context, const std::string& instance_id) {
  const std::string path =
      (std::filesystem::path(context.places.map_dir()) / "agent_instance").string();
  if (!common::WriteFileAtomically(path, instance_id + "\n")) {
    LOG(ERROR) << "could not write " << path << ": egs on this machine falls back to /fs";
  }
}

void RegisterRootEndpoint(Routes& server, const ServiceContext& context,
                          const std::string& instance_id) {
  server.ConcurrentGet(
      "/root",
      Guarded([&context, &instance_id](const httplib::Request&, httplib::Response& response) {
        std::error_code error;
        const std::string map_dir =
            std::filesystem::absolute(context.places.map_dir(), error).string();
        JsonWriter writer = BeginReceipt(std::nullopt);
        writer.Field("api_version", kApiVersion)
            .Field("instance_id", instance_id)
            .Field("map_dir", map_dir)
            .Field("memory_dir", context.sandbox.root().string());
        if (context.map_root.empty()) {
          writer.NullField("map_root").NullField("map");
        } else {
          writer.Field("map_root", std::filesystem::absolute(context.map_root, error).string())
              .Field("map", context.map_name);
        }
        writer.EndObject();
        SendJson(response, writer);
      }));
}

}  // namespace

AgentService::AgentService(lifelong::PoseGraph& pose_graph, AgentServiceHooks hooks,
                           const AgentServiceOption& option)
    : impl_(std::make_unique<Impl>(pose_graph, std::move(hooks), InstalledPlaceStore(pose_graph),
                                   option)) {
  const std::string& bind = option.bind;
  const int port = option.port;
  httplib::Server& server = impl_->server;
  // Service-side state stays single-threaded on the serial executor; one request per connection,
  // so an idle client never pins a pool thread.
  server.new_task_queue = [] { return new httplib::ThreadPool(kPoolThreads); };
  server.set_keep_alive_max_count(1);
  server.set_payload_max_length(kMaxRequestBytes);
  server.set_error_handler([](const httplib::Request& request, httplib::Response& response) {
    if (response.body.empty()) {
      SendError(response, response.status, response.status == 404 ? "not_found" : "bad_request",
                request.method + " " + request.path);
    }
  });

  WriteInstanceFile(impl_->context, impl_->instance_id);
  Routes routes(server, impl_->serial);
  RegisterRootEndpoint(routes, impl_->context, impl_->instance_id);
  RegisterPoseEndpoints(routes, impl_->context);
  RegisterPlaceEndpoints(routes, impl_->context);
  RegisterSessionEndpoints(routes, impl_->context);
  RegisterEventsEndpoints(routes, impl_->context, impl_->instance_id);
  RegisterViewEndpoints(routes, impl_->context);
  RegisterFsEndpoints(routes, impl_->context);
  RegisterMapsEndpoints(routes, impl_->context);
  RegisterZonesEndpoints(routes, impl_->context);

  impl_->port =
      port == 0 ? server.bind_to_any_port(bind) : (server.bind_to_port(bind, port) ? port : -1);
  if (impl_->port < 0) {
    LOG(ERROR) << "agent service could not bind " << bind << ":" << port;
    return;
  }
  impl_->listener = std::thread([this] { impl_->server.listen_after_bind(); });
  server.wait_until_ready();
  LOG(INFO) << "agent service on http://" << bind << ":" << impl_->port << ", memory at "
            << impl_->context.places.memory_dir();
}

AgentService::~AgentService() { Stop(); }

void AgentService::OnBackendStarted() { impl_->context.backend_started = true; }

void AgentService::Stop() {
  std::call_once(impl_->stop_once, [this] {
    impl_->context.stopping = true;
    impl_->server.stop();
    if (impl_->listener.joinable()) {
      impl_->listener.join();
    }
  });
}

int AgentService::port() const { return impl_->port; }

}  // namespace evergreenslam::agent
