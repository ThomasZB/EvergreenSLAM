/**
 * @file chow_liu_tree_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Prim against brute-force enumeration of every spanning tree.
 * @version 0.1
 * @date 2026-08-09
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/pose_graph_trimmer/chow_liu_tree.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

namespace evergreenslam::lifelong {
namespace {

// mt19937 output is specified bit for bit by the standard, unlike the distributions, so the same
// weights appear on every platform.
double PortableUniform(std::mt19937& engine) {
  return (static_cast<double>(engine()) + 0.5) / 4294967296.0;
}

Eigen::MatrixXd RandomSymmetricWeights(int n, std::uint32_t seed) {
  std::mt19937 engine(seed);
  Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const double w = PortableUniform(engine);
      weights(i, j) = w;
      weights(j, i) = w;
    }
  }
  return weights;
}

struct BruteForceResult {
  double best_weight = 0.0;
  std::vector<TreeEdge> best_edges;
  int num_trees = 0;
};

class UnionFind {
 public:
  explicit UnionFind(int n) : parent_(n) {
    for (int i = 0; i < n; ++i) {
      parent_[i] = i;
    }
  }
  int Find(int x) {
    while (parent_[x] != x) {
      parent_[x] = parent_[parent_[x]];
      x = parent_[x];
    }
    return x;
  }
  bool Union(int a, int b) {
    a = Find(a);
    b = Find(b);
    if (a == b) {
      return false;
    }
    parent_[a] = b;
    return true;
  }

 private:
  std::vector<int> parent_;
};

BruteForceResult EnumerateSpanningTrees(const Eigen::MatrixXd& weights) {
  const int n = static_cast<int>(weights.rows());
  std::vector<TreeEdge> all_edges;
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      all_edges.push_back(TreeEdge{i, j});
    }
  }
  const int num_edges = static_cast<int>(all_edges.size());
  std::vector<bool> mask(num_edges, false);
  std::fill(mask.begin(), mask.begin() + (n - 1), true);

  BruteForceResult result;
  result.best_weight = -1.0;
  do {
    UnionFind components(n);
    double total = 0.0;
    bool acyclic = true;
    std::vector<TreeEdge> edges;
    for (int e = 0; e < num_edges && acyclic; ++e) {
      if (!mask[e]) {
        continue;
      }
      if (!components.Union(all_edges[e].first, all_edges[e].second)) {
        acyclic = false;
        break;
      }
      total += weights(all_edges[e].first, all_edges[e].second);
      edges.push_back(all_edges[e]);
    }
    if (!acyclic) {
      continue;
    }
    ++result.num_trees;
    if (total > result.best_weight) {
      result.best_weight = total;
      result.best_edges = edges;
    }
  } while (std::prev_permutation(mask.begin(), mask.end()));
  return result;
}

double TotalWeight(const std::vector<TreeEdge>& edges, const Eigen::MatrixXd& weights) {
  double total = 0.0;
  for (const TreeEdge& edge : edges) {
    total += weights(edge.first, edge.second);
  }
  return total;
}

std::vector<TreeEdge> Sorted(std::vector<TreeEdge> edges) {
  std::sort(edges.begin(), edges.end(), [](const TreeEdge& a, const TreeEdge& b) {
    return a.first != b.first ? a.first < b.first : a.second < b.second;
  });
  return edges;
}

TEST(ChowLiuTreeTest, EmptyAndSingletonHaveNoEdges) {
  EXPECT_TRUE(ComputeMaximumSpanningTree(Eigen::MatrixXd::Zero(0, 0)).empty());
  EXPECT_TRUE(ComputeMaximumSpanningTree(Eigen::MatrixXd::Zero(1, 1)).empty());
}

TEST(ChowLiuTreeTest, TwoVariablesYieldTheOnlyEdge) {
  Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(2, 2);
  weights(0, 1) = weights(1, 0) = 0.7;
  const std::vector<TreeEdge> edges = ComputeMaximumSpanningTree(weights);
  ASSERT_EQ(edges.size(), 1u);
  EXPECT_EQ(edges.front(), (TreeEdge{0, 1}));
}

TEST(ChowLiuTreeTest, MatchesBruteForceOnRandomWeights) {
  for (int n = 3; n <= 6; ++n) {
    for (std::uint32_t seed = 0; seed < 10; ++seed) {
      const Eigen::MatrixXd weights = RandomSymmetricWeights(n, seed * 97 + n);
      const BruteForceResult brute = EnumerateSpanningTrees(weights);
      const std::vector<TreeEdge> prim = ComputeMaximumSpanningTree(weights);

      ASSERT_EQ(static_cast<int>(prim.size()), n - 1) << "n " << n << " seed " << seed;
      // Continuous random weights: the maximum tree is unique, so the edge sets must agree.
      EXPECT_NEAR(TotalWeight(prim, weights), brute.best_weight, 1e-12)
          << "n " << n << " seed " << seed;
      EXPECT_EQ(Sorted(prim), Sorted(brute.best_edges)) << "n " << n << " seed " << seed;
    }
  }
}

TEST(ChowLiuTreeTest, PrefersTheStrongestEdgesOnAChainableCase) {
  Eigen::MatrixXd weights = Eigen::MatrixXd::Zero(3, 3);
  weights(0, 1) = weights(1, 0) = 2.0;
  weights(1, 2) = weights(2, 1) = 1.5;
  weights(0, 2) = weights(2, 0) = 0.1;
  const std::vector<TreeEdge> edges = ComputeMaximumSpanningTree(weights);
  ASSERT_EQ(edges.size(), 2u);
  EXPECT_EQ(edges[0], (TreeEdge{0, 1}));
  EXPECT_EQ(edges[1], (TreeEdge{1, 2}));
}

}  // namespace
}  // namespace evergreenslam::lifelong
