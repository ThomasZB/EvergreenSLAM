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

#include <filesystem>
#include <mutex>
#include <thread>
#include <utility>

#include "service/endpoints.h"
#include "service/http_reply.h"
#include "service/httplib_include.h"
#include "service/service_context.h"

namespace evergreenslam::agent {
namespace {

constexpr size_t kMaxRequestBytes = 8 << 20;

}  // namespace

struct AgentService::Impl {
  Impl(lifelong::PoseGraph& pose_graph, AgentServiceHooks hooks, PlaceStore places,
       const AgentServiceOption& option)
      : context(pose_graph, std::move(hooks), std::move(places), option.map_root, option.map_name) {
  }

  ServiceContext context;
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

void RegisterRootEndpoint(httplib::Server& server, const ServiceContext& context) {
  server.Get(
      "/root", Guarded([&context](const httplib::Request&, httplib::Response& response) {
        std::error_code error;
        const std::string map_dir =
            std::filesystem::absolute(context.places.map_dir(), error).string();
        JsonWriter writer = BeginReceipt(std::nullopt);
        writer.Field("map_dir", map_dir).Field("memory_dir", context.sandbox.root().string());
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
  // One worker: the service-side state is single-threaded, and one connection must not pin it.
  server.new_task_queue = [] { return new httplib::ThreadPool(1); };
  server.set_keep_alive_max_count(1);
  server.set_payload_max_length(kMaxRequestBytes);
  server.set_error_handler([](const httplib::Request& request, httplib::Response& response) {
    if (response.body.empty()) {
      SendError(response, response.status, response.status == 404 ? "not_found" : "bad_request",
                request.method + " " + request.path);
    }
  });

  RegisterRootEndpoint(server, impl_->context);
  RegisterPlaceEndpoints(server, impl_->context);
  RegisterSessionEndpoints(server, impl_->context);
  RegisterMapEndpoints(server, impl_->context);
  RegisterFsEndpoints(server, impl_->context);
  RegisterMapsEndpoints(server, impl_->context);

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
