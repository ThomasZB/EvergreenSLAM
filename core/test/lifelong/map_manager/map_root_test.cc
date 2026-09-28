/**
 * @file map_root_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief Name rule, boot choice precedence, listing and the `current` file.
 * @version 0.1
 * @date 2026-09-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "lifelong/map_manager/map_root.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

namespace evergreenslam::lifelong {
namespace {

std::string MakeTempDir(const std::string& name) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      ("evergreenslam_root_" + name + "_" + std::to_string(::getpid()));
  std::filesystem::remove_all(path);
  std::filesystem::create_directories(path);
  return path.string();
}

void WriteText(const std::string& path, const std::string& text) {
  std::ofstream stream(path, std::ios::trunc);
  stream << text;
}

TEST(MapRootTest, ValidatesNames) {
  for (const char* name : {"default", "a", "0", "lab_2", "b-1", "9x"}) {
    EXPECT_TRUE(MapRoot::IsValidName(name)) << name;
  }
  for (const char* name : {"", "_a", "-a", "A", "a/b", "..", ".", "a b", "caf\xc3\xa9", "a.b"}) {
    EXPECT_FALSE(MapRoot::IsValidName(name)) << name;
  }
}

TEST(MapRootTest, ResolveJoinsWithoutTouchingDisk) {
  const MapRoot root("/nonexistent/maps");
  EXPECT_EQ(root.Resolve("lab"), "/nonexistent/maps/lab");
  EXPECT_FALSE(root.Exists("lab"));
  EXPECT_TRUE(root.List().empty());
  EXPECT_FALSE(root.ReadCurrent().has_value());
}

TEST(MapRootTest, ChooseAtBootPrefersParameterThenCurrentThenDefault) {
  const MapRoot root(MakeTempDir("choose"));
  EXPECT_EQ(root.ChooseAtBoot(""), MapRoot::kDefaultName);
  ASSERT_TRUE(root.WriteCurrent("lab"));
  EXPECT_EQ(root.ChooseAtBoot(""), "lab");
  EXPECT_EQ(root.ChooseAtBoot("office"), "office");
}

TEST(MapRootTest, ListKeepsValidDirectoriesOnly) {
  const std::string dir = MakeTempDir("list");
  const MapRoot root(dir);
  for (const char* name : {"zeta", "alpha", "Upper", "_hidden"}) {
    std::filesystem::create_directories(root.Resolve(name));
  }
  WriteText(root.Resolve("plain_file"), "x");
  ASSERT_TRUE(root.WriteCurrent("alpha"));
  EXPECT_EQ(root.List(), (std::vector<std::string>{"alpha", "zeta"}));
  EXPECT_TRUE(root.Exists("zeta"));
  EXPECT_FALSE(root.Exists("plain_file"));
}

TEST(MapRootTest, WriteCurrentCreatesTheRootAndRoundTrips) {
  const std::string dir = MakeTempDir("roundtrip") + "/nested/maps";
  const MapRoot root(dir);
  ASSERT_TRUE(root.WriteCurrent("b"));
  EXPECT_EQ(root.ReadCurrent(), std::optional<std::string>("b"));
  ASSERT_TRUE(root.WriteCurrent("default"));
  EXPECT_EQ(root.ReadCurrent(), std::optional<std::string>("default"));
}

TEST(MapRootTest, ReadCurrentTrimsAndRejectsGarbage) {
  const MapRoot root(MakeTempDir("garbage"));
  const std::string current = root.root() + "/current";
  WriteText(current, "  lab\n\n");
  EXPECT_EQ(root.ReadCurrent(), std::optional<std::string>("lab"));
  WriteText(current, "../etc\n");
  EXPECT_FALSE(root.ReadCurrent().has_value());
  WriteText(current, "");
  EXPECT_FALSE(root.ReadCurrent().has_value());
  EXPECT_EQ(root.ChooseAtBoot(""), MapRoot::kDefaultName);
}

}  // namespace
}  // namespace evergreenslam::lifelong
