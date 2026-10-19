/**
 * @file kidnap_report_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief A kidnap the backend found on its own, as /here, /status and /events report it, over real
 * HTTP in front of a session booted onto the wrong spot of a frozen simulated-laser base.
 * @version 0.1
 * @date 2026-10-19
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

const Eigen::Affine2d& LocalToTruth() {
  static const Eigen::Affine2d offset = utils::transform::FromXYTheta(47.65, -33.8, 1.6);
  return offset;
}

// Inside the room, 3 m and 0.6 rad off: the base can disagree with it (core kidnap_e2e_test).
Eigen::Affine2d WrongSeed() {
  const Eigen::Affine2d center = utils::transform::FromXYTheta(6.5, 5.0, 0.0);
  return Eigen::Affine2d(utils::transform::FromXYTheta(1.5, -0.5, 0.0) * center *
                         utils::transform::FromXYTheta(0.0, 0.0, 0.6) * center.inverse() *
                         LocalToTruth());
}

lifelong::PoseGraphOption KidnapOption() {
  lifelong::PoseGraphOption option = lifelong::testing::AnchorTestOption();
  option.kidnap_detector.max_base_score = 0.5;
  option.kidnap_detector.min_base_coverage = 0.4;
  return option;
}

class KidnapReportE2eTest : public ::testing::Test {
 protected:
  void SetUp() override {
    map_dir_ = lifelong::testing::MakeTempDir("evergreenslam_agent_kidnap");
    const lifelong::PoseGraphOption option = KidnapOption();
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
    backend_->Start(TestTime(total_steps), WrongSeed());
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

  std::string map_dir_;
  std::vector<Eigen::Affine2d> local_;
  SimulatedHost host_;
  std::unique_ptr<PoseGraph> backend_;
  std::unique_ptr<AgentService> service_;
  std::unique_ptr<Frontend> frontend_;
  std::unique_ptr<TestClient> client_;
  int step_ = 0;
};

TEST_F(KidnapReportE2eTest, HereStatusAndEventsReportTheKidnapAndItsSearch) {
  EXPECT_EQ(JsonRaw(client_->Get("/here").body, "localization.kidnap"), "null");

  FeedThrough(40);
  const Reply here = client_->Get("/here");
  ASSERT_EQ(JsonRaw(here.body, "ok"), "true") << here.body;
  ASSERT_NE(JsonRaw(here.body, "localization.kidnap"), "null") << here.body;
  const std::string request_id = JsonRaw(here.body, "localization.kidnap.request_id");
  EXPECT_EQ(request_id, "1");
  EXPECT_EQ(JsonNumber(here.body, "localization.kidnap.keyframes"),
            KidnapOption().kidnap_detector.min_suspect_keyframes);
  EXPECT_EQ(JsonRaw(here.body, "localization.kidnap.lowest_match_score"), "null");
  EXPECT_LT(JsonNumber(here.body, "localization.kidnap.base_score"), 0.5);
  EXPECT_GE(JsonNumber(here.body, "localization.kidnap.base_coverage"), 0.4);
  EXPECT_GE(JsonNumber(here.body, "localization.kidnap.since_base_match_m"), 3.0);
  EXPECT_EQ(JsonRaw(here.body, "localization.kidnap.continuity_broken"), "false")
      << "the search snapped the session onto the base and closures confirmed it";
  EXPECT_EQ(JsonRaw(here.body, "relocalization.id"), request_id);
  EXPECT_EQ(JsonRaw(here.body, "relocalization.kind"), "kidnap");
  EXPECT_EQ(JsonRaw(here.body, "relocalization.state"), "matched");
  EXPECT_GT(JsonNumber(here.body, "relocalization.correction_m"), 1.0);

  const Reply status = client_->Get("/status");
  EXPECT_EQ(JsonRaw(status.body, "localization.kidnap.request_id"), request_id);
  EXPECT_EQ(JsonRaw(status.body, "relocalization.kind"), "kidnap");

  const Reply events = client_->Get("/events");
  ASSERT_EQ(JsonRaw(events.body, "ok"), "true") << events.body;
  const size_t kidnap = events.body.find("\"kind\":\"kidnap_suspected\"");
  ASSERT_NE(kidnap, std::string::npos) << events.body;
  const size_t outcome = events.body.find("\"kind\":\"relocalization\"", kidnap);
  EXPECT_NE(outcome, std::string::npos) << "the search's outcome follows the suspicion";
  EXPECT_NE(events.body.find("\"request_id\":1,\"session\":", kidnap), std::string::npos)
      << events.body;
}

}  // namespace
}  // namespace evergreenslam::agent
