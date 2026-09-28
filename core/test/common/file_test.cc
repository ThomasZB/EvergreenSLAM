/**
 * @file file_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "common/file.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace evergreenslam::common {
namespace {

class FileTest : public ::testing::Test {
 protected:
  void SetUp() override {
    directory_ = std::filesystem::temp_directory_path() /
                 ("evergreenslam_file_test_" +
                  std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                  ::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::create_directories(directory_);
  }

  void TearDown() override { std::filesystem::remove_all(directory_); }

  std::string PathOf(const std::string& name) const { return (directory_ / name).string(); }

  std::filesystem::path directory_;
};

TEST_F(FileTest, WriteAtomicallySucceedsAndLeavesNoTempFile) {
  const std::string path = PathOf("data.bin");
  EXPECT_TRUE(WriteFileAtomically(path, "payload"));
  EXPECT_TRUE(std::filesystem::exists(path));
  EXPECT_FALSE(std::filesystem::exists(path + ".tmp"));
}

TEST_F(FileTest, ReadMissingFileReturnsNullopt) {
  EXPECT_EQ(ReadFile(PathOf("does_not_exist.bin")), std::nullopt);
}

TEST_F(FileTest, WriteThenReadRoundTrips) {
  const std::string path = PathOf("data.bin");
  const std::string contents("binary\0with\nembedded\0nulls", 26);
  ASSERT_TRUE(WriteFileAtomically(path, contents));
  EXPECT_EQ(ReadFile(path), contents);
}

TEST_F(FileTest, WriteReplacesExistingContents) {
  const std::string path = PathOf("data.bin");
  ASSERT_TRUE(WriteFileAtomically(path, "old contents that are longer"));
  ASSERT_TRUE(WriteFileAtomically(path, "new"));
  EXPECT_EQ(ReadFile(path), "new");
}

TEST_F(FileTest, FailedRenameReportsFailureAndRemovesTheTempFile) {
  const std::string path = PathOf("blocked.bin");
  ASSERT_TRUE(std::filesystem::create_directory(path));
  EXPECT_FALSE(WriteFileAtomically(path, "payload"));
  EXPECT_FALSE(std::filesystem::exists(path + ".tmp"));
  EXPECT_TRUE(std::filesystem::is_directory(path));
}

TEST_F(FileTest, UnopenableTempFileIsAFailureAndLeftAlone) {
  const std::string path = PathOf("data.bin");
  ASSERT_TRUE(std::filesystem::create_directories(path + ".tmp/occupied"));
  EXPECT_FALSE(WriteFileAtomically(path, "payload"));
  EXPECT_TRUE(std::filesystem::is_directory(path + ".tmp/occupied"));
  EXPECT_FALSE(std::filesystem::exists(path));
}

}  // namespace
}  // namespace evergreenslam::common
