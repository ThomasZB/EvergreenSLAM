/**
 * @file probability_grid_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-07-26
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "mapping/grid_mapping/probability_grid.h"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "mapping/grid_mapping/grid_map.h"
#include "mapping/grid_mapping/probability_values.h"

namespace evergreenslam::mapping {
namespace {

constexpr float kResolution = 0.05f;

// Every test coordinate is a multiple of kResolution plus 0.02, so a point
// always lands 0.4 of a cell in whatever origin the grid picks. Without that
// the assertions would sit on a cell boundary and the expected cell would
// depend on float rounding rather than on the code under test.
const std::vector<Eigen::Vector2d>& WrittenPoints() {
  static const std::vector<Eigen::Vector2d> points = {
      Eigen::Vector2d(0.32f, 0.17f), Eigen::Vector2d(0.52f, 0.37f), Eigen::Vector2d(0.12f, 0.57f),
      Eigen::Vector2d(0.72f, 0.07f), Eigen::Vector2d(0.22f, 0.27f)};
  return points;
}

std::vector<uint8_t> HitTable() {
  return ComputeLookupTableToApplyOdds(Odds(kDefaultHitProbability));
}

std::vector<uint8_t> MissTable() {
  return ComputeLookupTableToApplyOdds(Odds(kDefaultMissProbability));
}

// Writes each point a different number of times with a different table, so
// that every written cell ends up with a distinct probability. A shifted
// lookup after a grow then reads a wrong value rather than a lucky one.
struct Written {
  ProbabilityGrid grid{kResolution};
  std::vector<float> probabilities;
};

Written MakeWrittenGrid() {
  Written written;
  written.grid.GrowToInclude(Eigen::Vector2d(0.f, 0.f), Eigen::Vector2d(0.8f, 0.6f));

  const std::vector<uint8_t> hit = HitTable();
  const std::vector<uint8_t> miss = MissTable();
  const std::vector<const std::vector<uint8_t>*> tables = {&hit, &miss, &hit, &miss, &hit};
  const std::vector<int> repeats = {1, 2, 3, 1, 2};

  for (int round = 0; round < 3; ++round) {
    for (size_t i = 0; i < WrittenPoints().size(); ++i) {
      if (repeats[i] <= round) {
        continue;
      }
      written.grid.ApplyLookupTable(written.grid.ToCell(WrittenPoints()[i]), *tables[i]);
    }
    written.grid.FinishUpdate();
  }

  for (const Eigen::Vector2d& point : WrittenPoints()) {
    written.probabilities.push_back(written.grid.GetProbability(written.grid.ToCell(point)));
  }
  return written;
}

void ExpectUnchanged(const ProbabilityGrid& grid, const std::vector<float>& expected,
                     const char* stage) {
  for (size_t i = 0; i < WrittenPoints().size(); ++i) {
    const Eigen::Array2i cell = grid.ToCell(WrittenPoints()[i]);
    ASSERT_TRUE(grid.IsInside(cell)) << stage << ", point " << i;
    EXPECT_TRUE(grid.IsKnown(cell)) << stage << ", point " << i;
    EXPECT_NEAR(grid.GetProbability(cell), expected[i], 1e-6) << stage << ", point " << i;

    // The four neighbours were never written, so a one cell shift shows up
    // here as a known cell instead of an unknown one.
    const Eigen::Array2i offsets[4] = {Eigen::Array2i(1, 0), Eigen::Array2i(-1, 0),
                                       Eigen::Array2i(0, 1), Eigen::Array2i(0, -1)};
    for (const Eigen::Array2i& offset : offsets) {
      const Eigen::Array2i neighbour = cell + offset;
      EXPECT_FALSE(grid.IsKnown(neighbour))
          << stage << ", point " << i << ", offset " << offset.x() << ", " << offset.y();
      EXPECT_NEAR(grid.GetProbability(neighbour), kUnknownProbability, 1e-6)
          << stage << ", point " << i;
    }
  }
}

TEST(ProbabilityGridTest, WrittenCellsHaveDistinctProbabilities) {
  const Written written = MakeWrittenGrid();
  for (size_t i = 0; i < written.probabilities.size(); ++i) {
    for (size_t j = i + 1; j < written.probabilities.size(); ++j) {
      EXPECT_GT(std::abs(written.probabilities[i] - written.probabilities[j]), 1e-3f)
          << "points " << i << " and " << j;
    }
  }
  // Hits must read above the 0.5 prior and misses below it, or every other
  // assertion in this file is measuring an inverted map.
  EXPECT_GT(written.probabilities[0], 0.5f);
  EXPECT_LT(written.probabilities[1], 0.5f);
}

TEST(ProbabilityGridTest, GrowToNegativeSideKeepsWorldPositions) {
  Written written = MakeWrittenGrid();
  ExpectUnchanged(written.grid, written.probabilities, "before grow");

  const float old_origin_x = written.grid.origin_x();
  const float old_origin_y = written.grid.origin_y();
  const int old_width = written.grid.width();
  const int old_height = written.grid.height();

  // Far enough out that kGrowPaddingCells cannot absorb it; otherwise the
  // grow is a no-op and this test would pass without testing anything.
  written.grid.GrowToInclude(Eigen::Vector2d(-20.f, -20.f), Eigen::Vector2d(-19.f, -19.f));

  ASSERT_LT(written.grid.origin_x(), old_origin_x);
  ASSERT_LT(written.grid.origin_y(), old_origin_y);
  ASSERT_GT(written.grid.width(), old_width);
  ASSERT_GT(written.grid.height(), old_height);

  ExpectUnchanged(written.grid, written.probabilities, "after negative grow");
}

TEST(ProbabilityGridTest, GrowToPositiveSideKeepsWorldPositions) {
  Written written = MakeWrittenGrid();
  const float old_origin_x = written.grid.origin_x();
  const float old_origin_y = written.grid.origin_y();
  const int old_width = written.grid.width();
  const int old_height = written.grid.height();

  written.grid.GrowToInclude(Eigen::Vector2d(20.f, 20.f), Eigen::Vector2d(21.f, 21.f));

  ASSERT_GT(written.grid.width(), old_width);
  ASSERT_GT(written.grid.height(), old_height);
  EXPECT_FLOAT_EQ(written.grid.origin_x(), old_origin_x);
  EXPECT_FLOAT_EQ(written.grid.origin_y(), old_origin_y);

  ExpectUnchanged(written.grid, written.probabilities, "after positive grow");
}

TEST(ProbabilityGridTest, RepeatedGrowsInBothDirectionsKeepWorldPositions) {
  Written written = MakeWrittenGrid();
  written.grid.GrowToInclude(Eigen::Vector2d(-30.f, 5.f), Eigen::Vector2d(-29.f, 6.f));
  ExpectUnchanged(written.grid, written.probabilities, "grow -x +y");
  written.grid.GrowToInclude(Eigen::Vector2d(15.f, -40.f), Eigen::Vector2d(16.f, -39.f));
  ExpectUnchanged(written.grid, written.probabilities, "grow +x -y");
  written.grid.GrowToInclude(Eigen::Vector2d(0.1f, 0.1f), Eigen::Vector2d(0.2f, 0.2f));
  ExpectUnchanged(written.grid, written.probabilities, "no-op grow");
}

TEST(ProbabilityGridTest, ApplyLookupTableIsIdempotentWithinOneUpdate) {
  ProbabilityGrid grid(kResolution);
  grid.GrowToInclude(Eigen::Vector2d(0.f, 0.f), Eigen::Vector2d(1.f, 1.f));
  const std::vector<uint8_t> hit = HitTable();
  const std::vector<uint8_t> miss = MissTable();

  const Eigen::Array2i cell = grid.ToCell(Eigen::Vector2d(0.52f, 0.42f));
  const Eigen::Array2i other = grid.ToCell(Eigen::Vector2d(0.72f, 0.62f));

  EXPECT_TRUE(grid.ApplyLookupTable(cell, hit));
  const float after_first = grid.GetProbability(cell);
  EXPECT_GT(after_first, 0.5f);

  EXPECT_FALSE(grid.ApplyLookupTable(cell, hit));
  EXPECT_NEAR(grid.GetProbability(cell), after_first, 1e-6);
  EXPECT_FALSE(grid.ApplyLookupTable(cell, miss));
  EXPECT_NEAR(grid.GetProbability(cell), after_first, 1e-6);

  EXPECT_TRUE(grid.ApplyLookupTable(other, miss));

  grid.FinishUpdate();
  EXPECT_LT(static_cast<int>(grid.GetValue(cell)), static_cast<int>(kUpdateMarker));
  EXPECT_LT(static_cast<int>(grid.GetValue(other)), static_cast<int>(kUpdateMarker));
  EXPECT_TRUE(grid.IsKnown(cell));
  EXPECT_NEAR(grid.GetProbability(cell), after_first, 1e-6);

  EXPECT_TRUE(grid.ApplyLookupTable(cell, hit));
  const float after_second = grid.GetProbability(cell);
  EXPECT_GT(after_second, after_first + 1e-3f);
  grid.FinishUpdate();
  EXPECT_LT(static_cast<int>(grid.GetValue(cell)), static_cast<int>(kUpdateMarker));
  EXPECT_NEAR(grid.GetProbability(cell), after_second, 1e-6);
}

TEST(ProbabilityGridTest, KnownAreaCoversExactlyTheCellsWritten) {
  ProbabilityGrid grid(kResolution);
  EXPECT_TRUE(grid.empty());
  EXPECT_TRUE(grid.known_area().isEmpty());

  grid.GrowToInclude(Eigen::Vector2d(0.f, 0.f), Eigen::Vector2d(1.f, 1.f));
  EXPECT_TRUE(grid.empty()) << "growing alone writes no cells";

  const std::vector<Eigen::Vector2d> points = {
      Eigen::Vector2d(0.22f, 0.17f), Eigen::Vector2d(0.72f, 0.62f), Eigen::Vector2d(0.42f, 0.37f)};
  const std::vector<uint8_t> hit = HitTable();
  int min_x = 0;
  int min_y = 0;
  int max_x = 0;
  int max_y = 0;
  for (size_t i = 0; i < points.size(); ++i) {
    const Eigen::Array2i cell = grid.ToCell(points[i]);
    ASSERT_TRUE(grid.ApplyLookupTable(cell, hit));
    if (i == 0) {
      min_x = max_x = cell.x();
      min_y = max_y = cell.y();
    } else {
      min_x = std::min(min_x, cell.x());
      min_y = std::min(min_y, cell.y());
      max_x = std::max(max_x, cell.x());
      max_y = std::max(max_y, cell.y());
    }
  }
  grid.FinishUpdate();

  EXPECT_FALSE(grid.empty());
  const Eigen::AlignedBox2i& area = grid.known_area();
  EXPECT_EQ(area.min().x(), min_x);
  EXPECT_EQ(area.min().y(), min_y);
  EXPECT_EQ(area.max().x(), max_x);
  EXPECT_EQ(area.max().y(), max_y);
}

void ExpectSnapshotMatches(const Written& written, const char* stage) {
  const GridMapu8 snapshot = written.grid.ToSnapshot();

  EXPECT_EQ(snapshot.unknown_value(), kUnknownValue) << stage;
  EXPECT_FLOAT_EQ(snapshot.resolution(), kResolution) << stage;
  ASSERT_GT(snapshot.width(), 0) << stage;
  ASSERT_GT(snapshot.height(), 0) << stage;

  for (size_t i = 0; i < WrittenPoints().size(); ++i) {
    const Eigen::Vector2d& point = WrittenPoints()[i];
    EXPECT_NEAR(ValueToProbability(snapshot.GetValueAtPoint(point)), written.probabilities[i], 1e-6)
        << stage << ", point " << i;

    // Pins the crop offset: one cell out and these read a written value.
    const Eigen::Array2i cell = written.grid.ToCell(point);
    const Eigen::Array2i offsets[4] = {Eigen::Array2i(1, 0), Eigen::Array2i(-1, 0),
                                       Eigen::Array2i(0, 1), Eigen::Array2i(0, -1)};
    for (const Eigen::Array2i& offset : offsets) {
      const Eigen::Vector2d neighbour = written.grid.ToCenter(cell + offset);
      EXPECT_NEAR(ValueToProbability(snapshot.GetValueAtPoint(neighbour)), kUnknownProbability,
                  1e-6)
          << stage << ", point " << i;
    }
  }

  const Eigen::Vector2d far_away(12.32f, -8.17f);
  EXPECT_NEAR(ValueToProbability(snapshot.GetValueAtPoint(far_away)), kUnknownProbability, 1e-6)
      << stage;
  EXPECT_NEAR(written.grid.GetProbability(written.grid.ToCell(far_away)), kUnknownProbability, 1e-6)
      << stage;
}

TEST(ProbabilityGridTest, SnapshotMatchesSourceAtWorldPositions) {
  const Written written = MakeWrittenGrid();
  ExpectSnapshotMatches(written, "no grow");
}

TEST(ProbabilityGridTest, SnapshotAfterAGrowStillMatchesSource) {
  // known_area() is in cell coordinates, so a grow that moves the origin has
  // to move it too or the crop lands on the wrong region.
  Written written = MakeWrittenGrid();
  written.grid.GrowToInclude(Eigen::Vector2d(-20.f, -20.f), Eigen::Vector2d(-19.f, -19.f));
  ExpectSnapshotMatches(written, "after negative grow");
  written.grid.GrowToInclude(Eigen::Vector2d(25.f, 25.f), Eigen::Vector2d(26.f, 26.f));
  ExpectSnapshotMatches(written, "after positive grow");
}

TEST(ProbabilityGridTest, EmptyGridGivesEmptySnapshot) {
  const ProbabilityGrid grid(kResolution);
  EXPECT_TRUE(grid.empty());
  const GridMapu8 snapshot = grid.ToSnapshot();
  EXPECT_EQ(snapshot.width(), 0);
  EXPECT_EQ(snapshot.height(), 0);
  EXPECT_TRUE(snapshot.data().empty());
  EXPECT_EQ(snapshot.unknown_value(), kUnknownValue);
  EXPECT_NEAR(ValueToProbability(snapshot.GetValueAtPoint(Eigen::Vector2d(0.f, 0.f))),
              kUnknownProbability, 1e-6);
}

TEST(ProbabilityGridTest, UnwrittenCellsInsideTheGridAreUnknown) {
  ProbabilityGrid grid(kResolution);
  grid.GrowToInclude(Eigen::Vector2d(0.f, 0.f), Eigen::Vector2d(1.f, 1.f));
  for (int y = 0; y < grid.height(); y += 3) {
    for (int x = 0; x < grid.width(); x += 3) {
      const Eigen::Array2i cell(x, y);
      ASSERT_FALSE(grid.IsKnown(cell)) << "cell " << x << ", " << y;
      EXPECT_EQ(static_cast<int>(grid.GetValue(cell)), static_cast<int>(kUnknownValue));
      EXPECT_NEAR(grid.GetProbability(cell), kUnknownProbability, 1e-6);
    }
  }
}

}  // namespace
}  // namespace evergreenslam::mapping
