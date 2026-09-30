/**
 * @file keepout_mask_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief The nav2 keep-out mask: its raster, when a tick publishes, and the live publisher.
 * @version 0.2
 * @date 2026-09-29
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "keepout_mask_publisher.h"

namespace evergreenslam::ros2 {
namespace {

int8_t CellAt(const nav_msgs::msg::OccupancyGrid& mask, double x, double y) {
  const int col =
      static_cast<int>(std::floor((x - mask.info.origin.position.x) / mask.info.resolution));
  const int row =
      static_cast<int>(std::floor((y - mask.info.origin.position.y) / mask.info.resolution));
  EXPECT_TRUE(col >= 0 && row >= 0 && col < static_cast<int>(mask.info.width) &&
              row < static_cast<int>(mask.info.height))
      << x << ", " << y;
  return mask.data[static_cast<size_t>(row) * mask.info.width + col];
}

TEST(KeepoutMaskTest, MarksCellsInsideEitherPolygonWithinTheirBoundsPlusMargin) {
  // A square and, 3 m to its right, a triangle whose bounding box is half empty.
  const std::vector<agent::Polygon2d> polygons = {
      {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}},
      {{3.0, -0.5}, {4.0, -0.5}, {3.0, 0.5}},
  };
  const nav_msgs::msg::OccupancyGrid mask = *BuildKeepoutMask(polygons, 0.05, 0.5, 0.8, 4e6);

  EXPECT_FLOAT_EQ(mask.info.resolution, 0.05f);
  EXPECT_DOUBLE_EQ(mask.info.origin.position.x, -0.5);
  EXPECT_DOUBLE_EQ(mask.info.origin.position.y, -1.0);
  EXPECT_EQ(mask.info.width, 100u);
  EXPECT_EQ(mask.info.height, 50u);
  ASSERT_EQ(mask.data.size(), 100u * 50u);

  EXPECT_EQ(CellAt(mask, 0.5, 0.5), 100);
  EXPECT_EQ(CellAt(mask, 0.03, 0.97), 100);
  EXPECT_EQ(CellAt(mask, 3.2, -0.3), 100);
  EXPECT_EQ(CellAt(mask, 2.0, 0.5), 0);
  EXPECT_EQ(CellAt(mask, 3.8, 0.3), 0);
  EXPECT_EQ(CellAt(mask, -0.3, 1.3), 0);
  EXPECT_EQ(CellAt(mask, 1.2, 0.5), 0);
  for (const int8_t value : mask.data) {
    ASSERT_TRUE(value == 0 || value == 100);
  }
}

TEST(KeepoutMaskTest, NoZonesClearWithOneFreeCell) {
  const nav_msgs::msg::OccupancyGrid mask = *BuildKeepoutMask({}, 0.05, 0.5, 0.8, 4e6);
  EXPECT_EQ(mask.info.width, 1u);
  EXPECT_EQ(mask.info.height, 1u);
  ASSERT_EQ(mask.data.size(), 1u);
  EXPECT_EQ(mask.data[0], 0);
}

TEST(KeepoutMaskTest, FarApartZonesCoarsenTheGridAndAbsurdOnesAreRefused) {
  const agent::Polygon2d near = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};
  const agent::Polygon2d far = {{150.0, 150.0}, {152.0, 150.0}, {152.0, 152.0}, {150.0, 152.0}};
  const std::optional<nav_msgs::msg::OccupancyGrid> mask =
      BuildKeepoutMask({near, far}, 0.05, 0.5, 0.8, 4e6);
  ASSERT_TRUE(mask.has_value());
  EXPECT_FLOAT_EQ(mask->info.resolution, 0.1f);
  EXPECT_LE(mask->data.size(), 4000000u);
  EXPECT_EQ(CellAt(*mask, 1.0, 1.0), 100);
  EXPECT_EQ(CellAt(*mask, 151.0, 151.0), 100);
  EXPECT_EQ(CellAt(*mask, 75.0, 75.0), 0);

  const agent::Polygon2d absurd = {{1e9, 0.0}, {1e9 + 2.0, 0.0}, {1e9, 2.0}};
  EXPECT_FALSE(BuildKeepoutMask({near, absurd}, 0.05, 0.5, 0.8, 4e6).has_value());
}

agent::ResolvedZone Zone(const std::string& kind, std::optional<agent::Polygon2d> xy) {
  agent::ResolvedZone zone;
  zone.kind = kind;
  zone.polygon_xy = std::move(xy);
  return zone;
}

TEST(KeepoutMaskTest, TickPublishesOnlyTrustedChangedMasks) {
  const agent::Polygon2d square = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
  const std::vector<agent::ResolvedZone> zones = {Zone("keepout", square), Zone("slowdown", square),
                                                  Zone("keepout", std::nullopt)};
  const agent::BaseAlignment aligned{true, true};
  const agent::BaseAlignment unaligned{true, false};
  const agent::BaseAlignment no_base{false, false};
  const nav_msgs::msg::OccupancyGrid clear = ClearingKeepoutMask();

  EXPECT_FALSE(NextKeepoutMask(unaligned, zones, clear).has_value());
  EXPECT_FALSE(NextKeepoutMask(aligned, std::nullopt, clear).has_value());

  const std::optional<nav_msgs::msg::OccupancyGrid> first = NextKeepoutMask(no_base, zones, clear);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->info.width, 40u);
  EXPECT_EQ(CellAt(*first, 0.5, 0.5), 100);
  EXPECT_FALSE(NextKeepoutMask(aligned, zones, *first).has_value());

  // The last zone gone: one clearing grid, then nothing more.
  const std::optional<nav_msgs::msg::OccupancyGrid> cleared =
      NextKeepoutMask(aligned, std::vector<agent::ResolvedZone>{}, *first);
  ASSERT_TRUE(cleared.has_value());
  EXPECT_EQ(cleared->data, std::vector<int8_t>{0});
  EXPECT_FALSE(NextKeepoutMask(aligned, std::vector<agent::ResolvedZone>{}, clear).has_value());
}

class RclcppTest : public ::testing::Test {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

// The live wiring: a clearing grid on construction reaches a late transient_local subscriber, and
// the publisher drains queued ticks when it goes.
TEST_F(RclcppTest, PublisherClearsOnConstructionAndDrainsOnTeardown) {
  const std::filesystem::path memory =
      std::filesystem::temp_directory_path() /
      ("evergreenslam_keepout_mask_" + std::to_string(::getpid())) / "memory";
  std::filesystem::create_directories(memory / "places/nowhere");
  std::ofstream(memory / "places/nowhere/zone.yaml")
      << "kind: keepout\npolygon: [[0, 0], [1, 0], [1, 1]]\n";

  auto node = std::make_shared<rclcpp::Node>("keepout_mask_test");
  lifelong::PoseGraph pose_graph;
  const agent::ZoneStore zones(memory.string());
  auto publisher =
      std::make_unique<KeepoutMaskPublisher>(*node, pose_graph, zones, "map", "keepout_mask");

  std::vector<nav_msgs::msg::OccupancyGrid> received;
  auto subscription = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "keepout_mask", rclcpp::QoS(1).transient_local().reliable(),
      [&received](const nav_msgs::msg::OccupancyGrid& mask) { received.push_back(mask); });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (received.empty() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].header.frame_id, "map");
  EXPECT_EQ(received[0].data, std::vector<int8_t>{0});

  for (int i = 0; i < 5; ++i) {
    publisher->Publish();
  }
  publisher.reset();
  rclcpp::spin_some(node);
  // The zone has no place: nothing new resolves, and an unchanged mask is not republished.
  EXPECT_EQ(received.size(), 1u);
  std::filesystem::remove_all(memory.parent_path());
}

}  // namespace
}  // namespace evergreenslam::ros2
