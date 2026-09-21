/**
 * @file backend_handles.h
 * @author hang chen (chen@hang.plus)
 * @brief Narrow backend interfaces the submodules see instead of the whole facade.
 * @version 0.1
 * @date 2026-08-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_BACKEND_HANDLES_H_
#define EVERGREENSLAM_LIFELONG_BACKEND_HANDLES_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "common/time.h"
#include "lifelong/pose_graph_data.h"
#include "lifelong/task_queue.h"

namespace evergreenslam::lifelong {

class Optimization;

class TrimmingHandle {
 public:
  virtual ~TrimmingHandle() = default;

  virtual const PoseGraphData& graph() const = 0;
  virtual Optimization& optimization() = 0;
  virtual TrimReport ApplyTrim(const TrimRequest& request) = 0;
  virtual void Enqueue(TaskQueue::Task task) = 0;
};

class ConstraintHandle {
 public:
  virtual ~ConstraintHandle() = default;

  virtual const PoseGraphData& graph() const = 0;
  virtual bool AddConstraintIfEndpointsLive(const Constraint& constraint) = 0;
  virtual void Enqueue(TaskQueue::Task task) = 0;
};

class SessionHandle {
 public:
  virtual ~SessionHandle() = default;

  virtual const PoseGraphData& graph() const = 0;
  virtual Optimization& optimization() = 0;
  virtual SessionId StartNewSession(common::Time time, const Eigen::Affine2d& local_to_global) = 0;
  virtual void FreezeSession(SessionId id) = 0;
  virtual SessionId RotateAndFreezeFedSessionOnTask(SessionId id) = 0;
  virtual void TrimSubmapsOnTask(const std::vector<SubmapId>& ids) = 0;
  virtual void Enqueue(TaskQueue::Task task) = 0;
};

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_BACKEND_HANDLES_H_
