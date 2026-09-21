/**
 * @file chow_liu_tree.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_trimmer/chow_liu_tree.h"

#include <glog/logging.h>

#include <algorithm>
#include <limits>

namespace evergreenslam::lifelong {

std::vector<TreeEdge> ComputeMaximumSpanningTree(const Eigen::MatrixXd& mutual_information) {
  const int n = static_cast<int>(mutual_information.rows());
  CHECK_EQ(mutual_information.cols(), n) << "mutual information matrix must be square";
  if (n <= 1) {
    return {};
  }

  // Replacing only on a strictly better weight makes ties deterministic.
  std::vector<bool> in_tree(n, false);
  std::vector<double> best_weight(n, -std::numeric_limits<double>::infinity());
  std::vector<int> best_parent(n, -1);
  in_tree[0] = true;
  for (int v = 1; v < n; ++v) {
    best_weight[v] = mutual_information(0, v);
    best_parent[v] = 0;
  }

  std::vector<TreeEdge> edges;
  edges.reserve(n - 1);
  for (int step = 1; step < n; ++step) {
    int next = -1;
    for (int v = 0; v < n; ++v) {
      if (in_tree[v]) {
        continue;
      }
      if (next == -1 || best_weight[v] > best_weight[next]) {
        next = v;
      }
    }
    CHECK_GE(next, 0);
    const int parent = best_parent[next];
    edges.push_back(TreeEdge{std::min(parent, next), std::max(parent, next)});
    in_tree[next] = true;
    for (int v = 0; v < n; ++v) {
      if (!in_tree[v] && mutual_information(next, v) > best_weight[v]) {
        best_weight[v] = mutual_information(next, v);
        best_parent[v] = next;
      }
    }
  }

  std::sort(edges.begin(), edges.end(), [](const TreeEdge& a, const TreeEdge& b) {
    return a.first != b.first ? a.first < b.first : a.second < b.second;
  });
  return edges;
}

}  // namespace evergreenslam::lifelong
