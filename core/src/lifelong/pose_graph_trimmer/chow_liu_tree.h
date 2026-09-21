/**
 * @file chow_liu_tree.h
 * @author hang chen (chen@hang.plus)
 * @brief Maximum spanning tree over a mutual information matrix, on compact indices.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_CHOW_LIU_TREE_H_
#define EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_CHOW_LIU_TREE_H_

#include <Eigen/Core>
#include <vector>

namespace evergreenslam::lifelong {

struct TreeEdge {
  int first = 0;
  int second = 0;

  bool operator==(const TreeEdge& other) const {
    return first == other.first && second == other.second;
  }
};

// Maximum spanning tree over mutual information. The matrix must be square and symmetric.
std::vector<TreeEdge> ComputeMaximumSpanningTree(const Eigen::MatrixXd& mutual_information);

}  // namespace evergreenslam::lifelong

#endif  // EVERGREENSLAM_LIFELONG_POSE_GRAPH_TRIMMER_CHOW_LIU_TREE_H_
