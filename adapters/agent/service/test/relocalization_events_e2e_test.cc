/**
 * @file relocalization_events_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Relocalization request ids, their outcomes in /here and /status, and /events paging, over
 * real HTTP in front of a lost session on a frozen simulated-laser base.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "lifelong/anchors/anchor_scenario.h"
#include "service/agent_service.h"
#include "service_test_client.h"
#include "utils/transform/transform.h"

namespace evergreenslam::agent {
namespace {

using lifelong::PoseGraph;
using lifelong::testing::Frontend;
using lifelong::testing::kNodesPerLap;
using lifelong::testing::LoopGroundTruth;
using lifelong::testing::LoopRoom;
using lifelong::testing::SimulateScan;
using lifelong::testing::TestTime;
using testing::JsonNumber;
using testing::JsonRaw;
using testing::Reply;
using testing::SimulatedHost;
using testing::TestClient;

// The rebooted frontend's local frame sits far from the global one: booted at identity it is lost.
const Eigen::Affine2d& LocalToTruth() {
  static const Eigen::Affine2d offset = utils::transform::FromXYTheta(47.65, -33.8, 1.6);
  return offset;
}

std::string PoseParam(double value) { return std::to_string(value); }

httplib::Params PoseParams(const Eigen::Affine2d& pose) {
  return {{"x", PoseParam(pose.translation().x())},
          {"y", PoseParam(pose.translation().y())},
          {"theta", PoseParam(utils::transform::GetYaw(pose))}};
}

size_t Count(const std::string& body, const std::string& needle) {
  size_t count = 0;
  for (size_t at = body.find(needle); at != std::string::npos; at = body.find(needle, at + 1)) {
    ++count;
  }
  return count;
}

class RelocalizationEventsE2eTest : public ::testing::Test {
 protected:
  void SetUp() override {
    map_dir_ = lifelong::testing::MakeTempDir("evergreenslam_agent_relocalization");
    const lifelong::PoseGraphOption option = lifelong::testing::AnchorTestOption();
    const int total_steps = 2 * kNodesPerLap;
    {
      PoseGraph backend(option, map_dir_);
      backend.Start(TestTime(0));
      Frontend frontend(backend, lifelong::testing::GroundTruthOdometry(total_steps));
      lifelong::testing::FreezeABase(backend, frontend, total_steps);
      backend.Finish();
    }
    for (int step = 0; step < total_steps; ++step) {
      local_.push_back(Eigen::Affine2d(LocalToTruth().inverse() * LoopGroundTruth(step)));
    }
    backend_ = std::make_unique<PoseGraph>(lifelong::testing::NoFreeze(option), map_dir_);
    service_ = std::make_unique<AgentService>(*backend_, host_.Hooks(), AgentServiceOption{});
    backend_->Start(TestTime(total_steps), Eigen::Affine2d::Identity());
    service_->OnBackendStarted();
    frontend_ = std::make_unique<Frontend>(*backend_, local_);
    client_ = std::make_unique<TestClient>(service_->port());
  }

  void TearDown() override {
    client_.reset();
    service_->Stop();
    backend_->Finish();
  }

  void FeedThrough(int last_step) {
    for (; step_ <= last_step; ++step_) {
      frontend_->Feed(step_);
      backend_->WaitUntilQuiescent();
      host_.Update(TestTime(step_), local_[step_],
                   SimulateScan(LoopRoom(), LoopGroundTruth(step_)));
    }
  }

  std::string Relocalization(const std::string& id) {
    backend_->WaitUntilQuiescent();
    const Reply here = client_->Get("/here?relocalization=" + id);
    EXPECT_EQ(JsonRaw(here.body, "ok"), "true") << here.body;
    return here.body;
  }

  std::string map_dir_;
  std::vector<Eigen::Affine2d> local_;
  SimulatedHost host_;
  std::unique_ptr<PoseGraph> backend_;
  std::unique_ptr<AgentService> service_;
  std::unique_ptr<Frontend> frontend_;
  std::unique_ptr<TestClient> client_;
  int step_ = 0;
};

TEST_F(RelocalizationEventsE2eTest, RequestIdsReportTheirOutcomeInHereAndStatus) {
  EXPECT_EQ(JsonRaw(client_->Get("/here").body, "relocalization"), "null");
  EXPECT_EQ(JsonRaw(client_->Get("/status").body, "relocalization"), "null");

  // Before the first keyframe the request waits for it.
  const Reply init = client_->Post("/init-pose", PoseParams(LoopGroundTruth(0)));
  ASSERT_EQ(JsonRaw(init.body, "ok"), "true") << init.body;
  EXPECT_EQ(JsonRaw(init.body, "request_id"), "1");
  std::string body = Relocalization("1");
  EXPECT_EQ(JsonRaw(body, "relocalization.id"), "1");
  EXPECT_EQ(JsonRaw(body, "relocalization.kind"), "pose");
  EXPECT_EQ(JsonRaw(body, "relocalization.anchor"), "null");
  EXPECT_EQ(JsonRaw(body, "relocalization.state"), "pending");
  EXPECT_EQ(JsonRaw(body, "relocalization.requested_ns"), "null");
  EXPECT_EQ(JsonRaw(body, "relocalization.correction_m"), "null");

  FeedThrough(14);
  body = Relocalization("1");
  EXPECT_EQ(JsonRaw(body, "relocalization.state"), "matched") << body;
  EXPECT_GT(JsonNumber(body, "relocalization.candidates"), 0);
  EXPECT_GT(JsonNumber(body, "relocalization.constraints"), 0);
  EXPECT_GT(JsonNumber(body, "relocalization.correction_m"), 20.0);
  EXPECT_EQ(JsonRaw(body, "relocalization.requested_ns"),
            std::to_string(common::ToUnixNanos(TestTime(0))));
  EXPECT_EQ(JsonRaw(body, "aligned_to_base"), "true");
  EXPECT_NEAR(JsonNumber(body, "robot.x"), LoopGroundTruth(14).translation().x(), 0.15);

  // Outside every finished submap: settled at once, with nothing to match against.
  const Reply far =
      client_->Post("/init-pose", PoseParams(utils::transform::FromXYTheta(400, -250, 0)));
  EXPECT_EQ(JsonRaw(far.body, "request_id"), "2");
  body = Relocalization("2");
  EXPECT_EQ(JsonRaw(body, "relocalization.state"), "no_match");
  EXPECT_EQ(JsonRaw(body, "relocalization.candidates"), "0");
  EXPECT_NE(JsonRaw(body, "relocalization.resolved_ns"), "null");
  EXPECT_EQ(JsonRaw(Relocalization("1"), "relocalization.state"), "matched");

  const Reply saved = client_->Post("/place/save", {{"path", "places/dock"}});
  const std::string anchor = JsonRaw(saved.body, "anchor");
  const Reply by_place = client_->Post("/init-pose", {{"anchor", anchor}});
  EXPECT_EQ(JsonRaw(by_place.body, "request_id"), "3");
  body = Relocalization("3");
  EXPECT_EQ(JsonRaw(body, "relocalization.kind"), "place");
  EXPECT_EQ(JsonRaw(body, "relocalization.anchor"), anchor);
  EXPECT_NE(JsonRaw(body, "relocalization.state"), "pending");

  const Reply global = client_->Post("/relocalize");
  EXPECT_EQ(JsonRaw(global.body, "request_id"), "4");
  body = Relocalization("4");
  EXPECT_EQ(JsonRaw(body, "relocalization.kind"), "global");
  EXPECT_NE(JsonRaw(body, "relocalization.state"), "pending");

  const Reply status = client_->Get("/status");
  EXPECT_EQ(JsonRaw(status.body, "relocalization.id"), "4") << status.body;
  EXPECT_EQ(JsonRaw(client_->Get("/here").body, "relocalization.id"), "4");
  EXPECT_EQ(JsonRaw(Relocalization("99"), "relocalization"), "null");
  EXPECT_EQ(client_->Get("/here?relocalization=x").status, 400);
  EXPECT_EQ(JsonRaw(client_->Post("/init-pose", {{"anchor", "999"}}).body, "request_id"), "null");
}

TEST_F(RelocalizationEventsE2eTest, EventsPageBySequenceAndReportLoss) {
  EXPECT_EQ(client_->Get("/events?since=-1").status, 400);
  Reply events = client_->Get("/events");
  ASSERT_EQ(JsonRaw(events.body, "ok"), "true") << events.body;
  EXPECT_EQ(JsonRaw(events.body, "lost"), "false");
  const std::string boot_next = JsonRaw(events.body, "next_seq");

  FeedThrough(14);
  client_->Post("/init-pose", PoseParams(LoopGroundTruth(14)));
  backend_->WaitUntilQuiescent();
  events = client_->Get("/events?since=" + boot_next);
  EXPECT_EQ(JsonRaw(events.body, "lost"), "false");
  EXPECT_NE(events.body.find("\"kind\":\"pose_correction\""), std::string::npos) << events.body;
  EXPECT_NE(events.body.find("\"kind\":\"relocalization\",\"request_id\":1,\"state\":\"matched\""),
            std::string::npos)
      << events.body;
  const std::string next = JsonRaw(events.body, "next_seq");
  EXPECT_EQ(JsonRaw(client_->Get("/events?since=" + next).body, "events"), "[");
  EXPECT_EQ(
      JsonRaw(client_->Get("/events?since=" + std::to_string(std::stoi(next) + 50)).body, "lost"),
      "true");

  // Each far request settles at once and logs one event; enough of them overrun the ring.
  for (int i = 0; i < 260; ++i) {
    client_->Post("/init-pose", PoseParams(utils::transform::FromXYTheta(400, -250, 0)));
  }
  backend_->WaitUntilQuiescent();
  events = client_->Get("/events?since=" + next);
  EXPECT_EQ(JsonRaw(events.body, "lost"), "true");
  EXPECT_GT(JsonNumber(events.body, "first_seq"), std::stod(next));
  EXPECT_EQ(Count(events.body, "\"seq\":"), 256u);
}

}  // namespace
}  // namespace evergreenslam::agent
