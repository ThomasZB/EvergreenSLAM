/**
 * @file remote_entry_e2e_test.cc
 * @author hang chen (chen@hang.plus)
 * @brief What an agent on another machine relies on, over real HTTP: the instance id, /fs/grep,
 * the /fs/cat limit and /snapshots/file.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <gtest/gtest.h>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "common/file.h"
#include "service/agent_service.h"
#include "service_test_client.h"

namespace evergreenslam::agent {
namespace {

namespace fs = std::filesystem;

using lifelong::PoseGraph;
using lifelong::testing::TestTime;
using testing::JsonRaw;
using testing::Reply;
using testing::SimulatedHost;
using testing::TestClient;

std::string Reason(const Reply& reply) { return JsonRaw(reply.body, "reason"); }

void Write(const fs::path& path, const std::string& contents) {
  fs::create_directories(path.parent_path());
  ASSERT_TRUE(common::WriteFileAtomically(path.string(), contents)) << path;
}

std::vector<std::string> GrepLines(const std::string& body) {
  std::vector<std::string> lines;
  size_t position = 0;
  while ((position = body.find("{\"path\":", position)) != std::string::npos) {
    const size_t end = testing::MatchingClose(body, position);
    const std::string match = body.substr(position, end + 1 - position);
    lines.push_back(JsonRaw(match, "path") + ":" + JsonRaw(match, "line") + ":" +
                    JsonRaw(match, "text"));
    position = end;
  }
  return lines;
}

struct Fixture {
  explicit Fixture(const std::string& name)
      : map_dir(lifelong::testing::MakeTempDir(name)),
        memory(fs::path(map_dir) / "memory"),
        backend(lifelong::testing::NoFreeze(lifelong::testing::AnchorTestOption()), map_dir) {}

  const std::string map_dir;
  const fs::path memory;
  PoseGraph backend;
  SimulatedHost host;
};

TEST(RemoteEntryE2eTest, EachServiceWritesAFreshInstanceIdThatRootReports) {
  Fixture fixture("evergreenslam_agent_instance");
  const fs::path instance_file = fs::path(fixture.map_dir) / "agent_instance";
  std::string first;
  {
    AgentService service(fixture.backend, fixture.host.Hooks(), AgentServiceOption{});
    TestClient client(service.port());
    const Reply root = client.Get("/root");
    ASSERT_EQ(root.status, 200);
    EXPECT_EQ(JsonRaw(root.body, "api_version"), "0.3");
    first = JsonRaw(root.body, "instance_id");
    EXPECT_EQ(first.size(), 32u);
    EXPECT_EQ(first.find_first_not_of("0123456789abcdef"), std::string::npos) << first;
    EXPECT_EQ(common::ReadFile(instance_file.string()).value_or(""), first + "\n");
    EXPECT_EQ(fs::path(JsonRaw(root.body, "memory_dir")).parent_path() / "agent_instance",
              fs::canonical(instance_file));
  }
  AgentService service(fixture.backend, fixture.host.Hooks(), AgentServiceOption{});
  const std::string second = JsonRaw(TestClient(service.port()).Get("/root").body, "instance_id");
  EXPECT_NE(second, first);
  EXPECT_EQ(common::ReadFile(instance_file.string()).value_or(""), second + "\n");
}

TEST(RemoteEntryE2eTest, GrepIsFixedStringCaseInsensitiveAndSkipsLogsAttachmentsLinksAndBigFiles) {
  Fixture fixture("evergreenslam_agent_grep");
  const fs::path& memory = fixture.memory;
  AgentService service(fixture.backend, fixture.host.Hooks(), AgentServiceOption{});
  TestClient client(service.port());

  Write(memory / "places/kitchen/node.yaml", "kind: room\naliases: [Galley]\n");
  Write(memory / "places/kitchen/notes.md", "the galley door sticks\r\nnothing\nGALLEY light\n");
  Write(memory / "places/kitchen/observations.jsonl", "{\"note\": \"galley\"}\n");
  Write(memory / "places/kitchen/attachments/1.md", "galley\n");
  Write(memory / "places/kitchen/data.txt", "galley\n");
  Write(memory / "places/kitchen/big.md", "galley\n" + std::string(1 << 20, 'x'));
  Write(memory / "places/office/SKILL.md", "axb\n");
  fs::create_symlink(memory / "places/kitchen/notes.md", memory / "places/office/link.md");

  const Reply all = client.Get("/fs/grep?pattern=galley");
  ASSERT_EQ(all.status, 200) << all.body;
  EXPECT_EQ(JsonRaw(all.body, "truncated"), "false");
  EXPECT_EQ(GrepLines(all.body),
            (std::vector<std::string>{"places/kitchen/node.yaml:2:aliases: [Galley]",
                                      "places/kitchen/notes.md:1:the galley door sticks",
                                      "places/kitchen/notes.md:3:GALLEY light"}));
  EXPECT_EQ(GrepLines(client.Get("/fs/grep?pattern=GALLEY&path=places/kitchen/node.yaml").body),
            (std::vector<std::string>{"places/kitchen/node.yaml:2:aliases: [Galley]"}));
  EXPECT_EQ(
      GrepLines(client.Get("/fs/grep?pattern=galley&path=places&include=*.txt,%20node.yaml").body),
      (std::vector<std::string>{"places/kitchen/data.txt:1:galley",
                                "places/kitchen/node.yaml:2:aliases: [Galley]"}));
  EXPECT_TRUE(GrepLines(client.Get("/fs/grep?pattern=a.b").body).empty());
  EXPECT_EQ(GrepLines(client.Get("/fs/grep?pattern=AXB&path=places/office").body),
            (std::vector<std::string>{"places/office/SKILL.md:1:axb"}));

  std::string many;
  for (int i = 0; i < 250; ++i) {
    many += "galley " + std::to_string(i) + "\n";
  }
  Write(memory / "places/hall/notes.md", many);
  const Reply truncated = client.Get("/fs/grep?pattern=galley&path=places/hall");
  EXPECT_EQ(JsonRaw(truncated.body, "truncated"), "true");
  EXPECT_EQ(GrepLines(truncated.body).size(), 200u);

  EXPECT_EQ(Reason(client.Get("/fs/grep?pattern=x&path=../")), "path_escape");
  EXPECT_EQ(Reason(client.Get("/fs/grep?pattern=x&path=/etc")), "path_escape");
  EXPECT_EQ(Reason(client.Get("/fs/grep?pattern=x&path=places/office/link.md")), "symlink");
  EXPECT_EQ(Reason(client.Get("/fs/grep?pattern=x&path=places/none")), "fs_error");
  EXPECT_EQ(client.Get("/fs/grep?path=places").status, 400);
  EXPECT_EQ(Reason(client.Get("/fs/grep?pattern=")), "bad_param");
}

TEST(RemoteEntryE2eTest, CatServesUpToEightMebibytes) {
  Fixture fixture("evergreenslam_agent_cat");
  AgentService service(fixture.backend, fixture.host.Hooks(), AgentServiceOption{});
  TestClient client(service.port());
  const std::string limit(8 << 20, 'a');
  Write(fixture.memory / "places/dock/attachments/1.png", limit);
  Write(fixture.memory / "places/dock/attachments/2.png", limit + "b");
  const Reply cat = client.Get("/fs/cat?path=places/dock/attachments/1.png");
  EXPECT_EQ(cat.status, 200);
  EXPECT_EQ(cat.body.size(), limit.size());
  const Reply over = client.Get("/fs/cat?path=places/dock/attachments/2.png");
  EXPECT_EQ(over.status, 400);
  EXPECT_EQ(Reason(over), "too_large");
}

TEST(RemoteEntryE2eTest, SnapshotFilesServeOnlyListedNamesInsideTheirSnapshot) {
  Fixture fixture("evergreenslam_agent_snapshot_file");
  AgentService service(fixture.backend, fixture.host.Hooks(), AgentServiceOption{});
  TestClient client(service.port());
  fixture.backend.Start(TestTime(0));
  service.OnBackendStarted();

  const Reply snapshot = client.Post("/snapshot");
  ASSERT_EQ(JsonRaw(snapshot.body, "ok"), "true") << snapshot.body;
  const std::string seq = JsonRaw(snapshot.body, "seq");
  const fs::path dir = JsonRaw(snapshot.body, "dir");
  for (const char* name : {"map.json", "trajectory.csv", "places.json", "summary.txt"}) {
    const Reply file = client.Get("/snapshots/file?seq=" + seq + "&name=" + name);
    ASSERT_EQ(file.status, 200) << name << ": " << file.body;
    EXPECT_EQ(file.body, common::ReadFile((dir / name).string()).value_or("-")) << name;
  }

  const fs::path map_dir = fixture.map_dir;
  Write(map_dir / "snapshots" / "secret.txt", "x");
  Write(dir / "extra.txt", "x");
  for (const std::string& query : std::vector<std::string>{
           "seq=" + seq + "&name=map.pgm", "seq=" + seq + "&name=extra.txt",
           "seq=" + seq + "&name=../secret.txt", "seq=" + seq + "&name=../../manifest.pb",
           "seq=" + seq + "&name=%2Fetc%2Fpasswd", std::string("seq=7&name=summary.txt")}) {
    const Reply refused = client.Get("/snapshots/file?" + query);
    EXPECT_EQ(refused.status, 404) << query;
    EXPECT_EQ(Reason(refused), "not_found") << query;
  }
  EXPECT_EQ(Reason(client.Get("/snapshots/file?seq=0&name=summary.txt")), "bad_param");
  EXPECT_EQ(Reason(client.Get("/snapshots/file?seq=x&name=summary.txt")), "bad_param");
  EXPECT_EQ(Reason(client.Get("/snapshots/file?seq=" + seq)), "bad_param");

  fs::rename(dir / "summary.txt", dir / "summary.real");
  fs::create_symlink(dir / "summary.real", dir / "summary.txt");
  EXPECT_EQ(client.Get("/snapshots/file?seq=" + seq + "&name=summary.txt").status, 404);
  fixture.backend.Finish();
}

}  // namespace
}  // namespace evergreenslam::agent
