/**
 * @file fake_host_main.cc
 * @author hang chen (chen@hang.plus)
 * @brief A stand-in host for the egs integration test: simulated laps, then the real service.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <glog/logging.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "lifelong/anchors/anchor_scenario.h"
#include "service/agent_service.h"
#include "service_test_client.h"

namespace evergreenslam::agent {
namespace {

using lifelong::PoseGraph;
using lifelong::testing::DriftedOdometry;
using lifelong::testing::Frontend;
using lifelong::testing::kNodesPerLap;
using lifelong::testing::LoopGroundTruth;
using lifelong::testing::LoopRoom;
using lifelong::testing::SimulateScan;
using lifelong::testing::TestTime;

// No real frontend runs here; a healthy-looking constant for egs to print.
constexpr double kFakeMatchScore = 0.8;

struct Flags {
  int port = 0;
  std::string map_dir;
  int laps = 2;
};

Flags ParseFlags(int argc, char** argv) {
  Flags flags;
  for (int i = 1; i < argc; ++i) {
    const std::string name = argv[i];
    if (i + 1 >= argc) {
      LOG(FATAL) << "usage: fake_host [--port N] [--map_dir DIR] [--laps N]";
    }
    const std::string value = argv[++i];
    if (name == "--port") {
      flags.port = std::atoi(value.c_str());
    } else if (name == "--map_dir") {
      flags.map_dir = value;
    } else if (name == "--laps") {
      flags.laps = std::atoi(value.c_str());
    } else {
      LOG(FATAL) << "unknown flag " << name;
    }
  }
  if (flags.map_dir.empty()) {
    flags.map_dir = lifelong::testing::MakeTempDir("evergreenslam_fake_host");
  }
  return flags;
}

int Run(const Flags& flags) {
  PoseGraph backend(lifelong::testing::NoFreeze(lifelong::testing::AnchorTestOption()),
                    flags.map_dir);
  testing::SimulatedHost host;
  // A real map root's layout without a switch hook: /maps lists, /maps/new is not_supported.
  const std::filesystem::path map_dir = std::filesystem::absolute(flags.map_dir);
  AgentServiceOption option;
  option.port = flags.port;
  option.map_root = map_dir.parent_path().string();
  option.map_name = map_dir.filename().string();
  AgentService service(backend, host.Hooks(), option);
  if (service.port() <= 0) {
    return EXIT_FAILURE;
  }
  backend.Start(TestTime(0));
  service.OnBackendStarted();

  const int num_steps = flags.laps * kNodesPerLap;
  const std::vector<Eigen::Affine2d> odometry = DriftedOdometry(num_steps);
  Frontend frontend(backend, odometry);
  for (int step = 0; step < num_steps; ++step) {
    frontend.Feed(step);
    backend.WaitUntilQuiescent();
    host.Update(TestTime(step), odometry[step], SimulateScan(LoopRoom(), LoopGroundTruth(step)),
                kFakeMatchScore);
  }

  std::cout << "READY " << service.port() << std::endl;
  std::string line;
  while (std::getline(std::cin, line)) {
  }
  service.Stop();
  backend.Finish();
  return EXIT_SUCCESS;
}

}  // namespace
}  // namespace evergreenslam::agent

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  return evergreenslam::agent::Run(evergreenslam::agent::ParseFlags(argc, argv));
}
