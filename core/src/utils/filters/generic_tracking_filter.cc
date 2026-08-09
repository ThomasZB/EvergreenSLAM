/**
 * @file generic_tracking_filter.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/filters/generic_tracking_filter.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

#include "glog/logging.h"
#include "utils/transform/transform.h"

namespace evergreenslam::utils::filters {

GenericTrackingFilter::GenericTrackingFilter(const GenericTrackingFilterOption& option)
    : GenericTrackingFilter(option.trans_alpha, option.rot_alpha, option.trans_sigma,
                            option.rot_sigma) {}

GenericTrackingFilter::GenericTrackingFilter(double trans_alpha, double rot_alpha,
                                             double trans_sigma, double rot_sigma) {
  CHECK_GT(trans_alpha, 0.0);
  CHECK_GT(rot_alpha, 0.0);
  trans_alpha_ = trans_alpha;
  rot_alpha_ = rot_alpha;
  trans_sigma_ = trans_sigma;
  rot_sigma_ = rot_sigma;
  H_ = Eigen::Matrix<double, 3, 9>::Zero();
  H_.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity();
  Reset();
}

void GenericTrackingFilter::Reset() {
  current_state_.time = common::Time::min();
  current_state_.state.setZero();
  current_state_.covariance = Eigen::Matrix<double, 9, 9>::Identity() * 1e6;
  state_history_.clear();
}

void GenericTrackingFilter::ResetZero() {
  current_state_.time = common::Time::min();
  current_state_.state.setZero();
  current_state_.covariance = Eigen::Matrix<double, 9, 9>::Identity() * 1e6;
  current_state_.covariance.block<3, 3>(0, 0) = Eigen::Matrix<double, 3, 3>::Identity() * 1e-6;
  state_history_.clear();
}

GenericTrackingFilter::State GenericTrackingFilter::PredictTime(common::Time time) const {
  if (state_history_.empty()) {
    return current_state_;
  }
  State base = NearestState(time);
  double dt = common::ToSeconds(time - base.time);
  if (dt <= 0) {
    return base;
  }
  State state;
  state.time = time;
  state.state = PredictState(base.state, dt);
  auto F = BuildF(base.state, dt);
  auto Q = BuildQ(dt);
  state.covariance = F * base.covariance * F.transpose() + Q;
  return state;
}

void GenericTrackingFilter::Update(const Measurement& m, bool from_history) {
  if (!Predict(m.time)) {
    LOG(WARNING) << "measurement older than the whole filter history, ignored";
    return;
  }

  Eigen::Matrix<double, 3, 1> z = transform::ToVector3(m.pose);
  Eigen::Matrix<double, 3, 1> y = z - H_ * current_state_.state;
  y(2) = utils::transform::NormalizeAngle(y(2));
  Eigen::Matrix<double, 3, 3> S = H_ * current_state_.covariance * H_.transpose() + m.covariance;
  Eigen::Matrix<double, 9, 3> K = current_state_.covariance * H_.transpose() * S.inverse();
  current_state_.state = current_state_.state + K * y;
  Eigen::Matrix<double, 9, 9> cov =
      (Eigen::Matrix<double, 9, 9>::Identity() - K * H_) * current_state_.covariance;
  current_state_.covariance = (cov + cov.transpose()) / 2.0;
  current_state_.time = m.time;
  current_state_.measurement = m;

  if (from_history) {
    return;
  }
  UpdateHistoryAndRePredict();
}

bool GenericTrackingFilter::Predict(common::Time time) {
  if (state_history_.empty()) {
    return true;
  }
  State state = PredictTime(time);
  // Older than the whole history, so there is nothing to replay from; reject without touching
  // current_state_.
  if (state.time > time) {
    return false;
  }
  current_state_ = state;
  return true;
}

namespace {

std::tuple<double, double> BuildAB(double dt, double omega) {
  if (std::abs(omega) < 1e-6) {
    return {dt, 0.5 * omega * dt * dt};
  } else {
    double a = std::sin(omega * dt) / omega;
    double b = (1 - std::cos(omega * dt)) / omega;
    return {a, b};
  }
}

Eigen::Matrix<double, 2, 2> BuildSingerVA(double dt, double alpha, double sigma) {
  double eat = std::exp(-alpha * dt);
  double e2at = std::exp(-2 * alpha * dt);
  double at = alpha * dt;
  double s2 = sigma * sigma;
  double a2 = alpha * alpha;

  Eigen::Matrix<double, 2, 2> Q;
  Q(0, 0) = s2 * (2 * at - 3 + 4 * eat - e2at) / a2;
  Q(0, 1) = s2 * (1 - 2 * eat + e2at) / alpha;
  Q(1, 0) = Q(0, 1);
  Q(1, 1) = s2 * (1 - e2at);
  return Q;
}

}  // namespace

Eigen::Matrix<double, 9, 1> GenericTrackingFilter::PredictState(
    const Eigen::Matrix<double, 9, 1>& state, double dt) const {
  Eigen::Matrix<double, 9, 1> new_state;
  double r = state(2);
  double vx = state(3);
  double vy = state(4);
  double w = state(5);
  double ax = state(6);
  double ay = state(7);
  double aw = state(8);
  double eart = std::exp(-rot_alpha_ * dt);
  double eatt = std::exp(-trans_alpha_ * dt);

  auto [a, b] = BuildAB(dt, w);
  const Eigen::Vector2d xy =
      state.segment<2>(0) +
      Eigen::Rotation2Dd(r) * Eigen::Vector2d(a * vx - b * vy, b * vx + a * vy);

  new_state.segment<2>(0) = xy;
  new_state(2) = r + w * dt + aw * (rot_alpha_ * dt - 1 + eart) / (rot_alpha_ * rot_alpha_);
  new_state(3) = vx + ax * (1 - eatt) / trans_alpha_;
  new_state(4) = vy + ay * (1 - eatt) / trans_alpha_;
  new_state(5) = w + aw * (1 - eart) / rot_alpha_;
  new_state(6) = ax * eatt;
  new_state(7) = ay * eatt;
  new_state(8) = aw * eart;

  return new_state;
}

GenericTrackingFilter::State GenericTrackingFilter::NearestState(common::Time time) const {
  auto it = std::lower_bound(state_history_.begin(), state_history_.end(), time,
                             [](const State& s, const common::Time& t) { return s.time < t; });
  if (it != state_history_.end() && it->time == time) {
    return *it;
  }
  if (it == state_history_.begin()) {
    return state_history_.front();
  }
  return *(it - 1);
}

Eigen::Matrix<double, 9, 9> GenericTrackingFilter::BuildQ(double dt) const {
  Eigen::Matrix<double, 9, 9> Q = Eigen::Matrix<double, 9, 9>::Zero();
  auto Qv = BuildSingerVA(dt, trans_alpha_, trans_sigma_);
  Q(3, 3) = Qv(0, 0);
  Q(3, 6) = Qv(0, 1);
  Q(6, 3) = Qv(1, 0);
  Q(6, 6) = Qv(1, 1);
  Q(4, 4) = Qv(0, 0);
  Q(4, 7) = Qv(0, 1);
  Q(7, 4) = Qv(1, 0);
  Q(7, 7) = Qv(1, 1);
  auto Qw = BuildSingerVA(dt, rot_alpha_, rot_sigma_);
  Q(5, 5) = Qw(0, 0);
  Q(5, 8) = Qw(0, 1);
  Q(8, 5) = Qw(1, 0);
  Q(8, 8) = Qw(1, 1);
  // TODO(hang): q3x
  return Q;
}

Eigen::Matrix<double, 9, 9> GenericTrackingFilter::BuildF(const Eigen::Matrix<double, 9, 1>& state,
                                                          double dt) const {
  Eigen::Matrix<double, 9, 9> F;
  for (int i = 0; i < 9; ++i) {
    Eigen::Matrix<double, 9, 1> sp = state;
    Eigen::Matrix<double, 9, 1> sm = state;
    double e = 1e-5;
    if (i == 2) {
      e = 1e-6;
    }
    sp(i) += e;
    sm(i) -= e;
    Eigen::Matrix<double, 9, 1> fp = PredictState(sp, dt);
    Eigen::Matrix<double, 9, 1> fm = PredictState(sm, dt);
    Eigen::Matrix<double, 9, 1> diff = fp - fm;
    diff(2) = utils::transform::NormalizeAngle(diff(2));
    F.col(i) = diff / (2 * e);
  }
  return F;
}

void GenericTrackingFilter::UpdateHistoryAndRePredict() {
  common::Time time = current_state_.time;

  std::vector<State> replay_q;
  while (!state_history_.empty() && state_history_.back().time > time) {
    replay_q.push_back(state_history_.back());
    state_history_.pop_back();
  }
  std::reverse(replay_q.begin(), replay_q.end());

  if (!state_history_.empty() && state_history_.back().time == time) {
    state_history_.back() = current_state_;
  } else {
    state_history_.push_back(current_state_);
  }

  for (const auto& s : replay_q) {
    Update(s.measurement, true);
    state_history_.push_back(current_state_);
  }

  while (state_history_.size() > max_history_size_) {
    state_history_.pop_front();
  }
}

}  // namespace evergreenslam::utils::filters