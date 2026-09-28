/**
 * @file session_manager.h
 * @author hang chen (chen@hang.plus)
 * @brief Who is the fed session, when does any unfrozen session freeze, and what happens then.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_SESSIONS_SESSION_MANAGER_H_
#define EVERGREENSLAM_LIFELONG_SESSIONS_SESSION_MANAGER_H_

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <vector>

#include "common/time.h"
#include "lifelong/backend_handles.h"
#include "lifelong/pose_graph_data.h"
#include "lifelong/sessions/freeze_judge.h"
#include "lifelong/sessions/frozen_coverage.h"

namespace evergreenslam::lifelong {

class SessionObserver {
 public:
  virtual ~SessionObserver() = default;

  virtual void OnSessionFrozen(const PoseGraphData& graph, SessionId id) = 0;

  virtual void OnSessionStarted(const PoseGraphData& graph, SessionId id) {}
};

struct SessionManagerOption {
  bool auto_freeze = true;
  // Simulation values, pending calibration on real bags.
  double min_new_area = 200.0;
  double min_new_area_fraction = 0.1;
  double growth_tolerance = 20.0;
  double track_growth_tolerance = 10.0;
  int submaps_without_growth = 7;
  double coverage_resolution = 0.15;
  double track_resolution = 0.4;
  double frozen_covered_fraction = 0.9;
  FreezeJudgeOption freeze_judge;

  friend std::ostream& operator<<(std::ostream& os, const SessionManagerOption& option) {
    os << "SessionManagerOption:" << std::endl;
    os << "  auto_freeze: " << option.auto_freeze << std::endl;
    os << "  min_new_area: " << option.min_new_area << std::endl;
    os << "  min_new_area_fraction: " << option.min_new_area_fraction << std::endl;
    os << "  growth_tolerance: " << option.growth_tolerance << std::endl;
    os << "  track_growth_tolerance: " << option.track_growth_tolerance << std::endl;
    os << "  submaps_without_growth: " << option.submaps_without_growth << std::endl;
    os << "  coverage_resolution: " << option.coverage_resolution << std::endl;
    os << "  track_resolution: " << option.track_resolution << std::endl;
    os << "  frozen_covered_fraction: " << option.frozen_covered_fraction << std::endl;
    os << option.freeze_judge;
    return os;
  }
};

SessionManagerOption LoadSessionManagerOption(const YAML::Node& node);

// Stepped only at OnOptimizedOnTask; the event hooks just set `dirty`.
class SessionManager {
 public:
  enum class Phase { BOOTSTRAP, LOCALIZING, EXPANDING, NO_GROWTH, JUDGING };

  struct ExpansionState {
    Phase phase = Phase::BOOTSTRAP;
    double novel_area = 0.0;
    double novel_track = 0.0;
    // After the previous accounted submap (EXPANDING).
    double last_area = 0.0;
    double last_track = 0.0;
    // When NO_GROWTH was entered.
    double baseline_area = 0.0;
    double baseline_track = 0.0;
    // Finished submaps accounted since NO_GROWTH was entered.
    int submaps_without_growth = 0;
    int accounted_submap_index = -1;
    bool dirty = false;
    bool anchored = false;
  };

  explicit SessionManager(SessionHandle& handle,
                          const SessionManagerOption& option = SessionManagerOption());

  // Once per boot: rotation opens every later fed session.
  SessionId Start(common::Time time,
                  const Eigen::Affine2d& local_to_global = Eigen::Affine2d::Identity());

  void AddObserver(std::shared_ptr<SessionObserver> observer);

  void OnSubmapFinishedOnTask(const SubmapId& id);
  void OnConstraintAddedOnTask(const Constraint& constraint);
  void RebuildAfterLoadOnTask();
  // The one entry: after the cadence solve and the regular trim, so the account and the verdict
  // read poses that closures have already been solved into.
  void OnOptimizedOnTask();

  // The freeze sequence without a verdict; queued, like everything that touches the graph.
  void FreezeFedSession(std::optional<SessionId> expected = std::nullopt);

  // After a fed freeze this is already the replacement session.
  std::optional<SessionId> fed_session() const { return fed_session_; }
  const FreezeVerdict& last_verdict() const { return last_verdict_; }
  // Atomic so a threaded caller may poll it; everything else here is backend-task state.
  int num_sessions_frozen() const { return num_sessions_frozen_.load(std::memory_order_acquire); }
  int num_judgements() const { return num_judgements_; }
  const FreezeJudge& freeze_judge() const { return judge_; }
  const FrozenCoverage& coverage() const { return coverage_; }
  const std::map<SessionId, ExpansionState>& expansion() const { return expansion_; }
  // Backend task only. A freeze sequence is queued and assumes its session outlives it.
  bool freezing() const { return freezing_; }

 private:
  int Refresh(SessionId id, ExpansionState& state);
  bool Judge(SessionId id, const ExpansionState& state);
  void FreezeAndMaybeRotate(SessionId id);
  void TrimCoveredSubmapsOnTask(SessionId id);
  void OnFrozenOnTask(SessionId id);

  SessionHandle& handle_;
  SessionManagerOption option_;
  FreezeJudge judge_;
  CovarianceEvaluator covariance_evaluator_;
  FrozenCoverage coverage_;
  std::vector<std::shared_ptr<SessionObserver>> observers_;

  std::optional<SessionId> fed_session_;
  std::map<SessionId, ExpansionState> expansion_;
  FreezeVerdict last_verdict_;
  int num_judgements_ = 0;
  std::atomic<int> num_sessions_frozen_{0};
  // The freeze sequence solves, and a solve would otherwise judge the half-frozen session.
  bool freezing_ = false;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_SESSIONS_SESSION_MANAGER_H_
