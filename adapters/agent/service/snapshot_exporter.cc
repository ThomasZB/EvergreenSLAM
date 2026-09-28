/**
 * @file snapshot_exporter.cc
 * @author hang chen (chen@hang.plus)
 * @brief Writes one solve's map, trajectory and places as plain files under snapshots/NNNNNN/.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/snapshot_exporter.h"

#include <glog/logging.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <utility>

#include "service/gray_png.h"
#include "service/json_writer.h"
#include "utils/transform/transform.h"

namespace evergreenslam::agent {
namespace {

constexpr uint8_t kOccupiedPixel = 0;
constexpr uint8_t kFreePixel = 254;
constexpr uint8_t kUnknownPixel = 205;
constexpr int kMaxPngSide = 1024;

bool HasCells(const mapping::GridMapu8& grid) { return grid.width() > 0 && grid.height() > 0; }

// Top row first: row r holds cell y = height - 1 - r.
std::vector<uint8_t> TrinaryRaster(const mapping::GridMapu8& grid) {
  std::vector<uint8_t> raster(static_cast<size_t>(grid.width()) * grid.height());
  for (int y = 0; y < grid.height(); ++y) {
    const size_t row = static_cast<size_t>(grid.height() - 1 - y);
    for (int x = 0; x < grid.width(); ++x) {
      raster[row * grid.width() + x] = TrinaryValue(grid.GetValue(x, y));
    }
  }
  return raster;
}

// Integer factor so the long side fits; an occupied cell anywhere in a block wins, then free.
std::vector<uint8_t> Downsample(const std::vector<uint8_t>& raster, int width, int height,
                                int& out_width, int& out_height) {
  const int factor = std::max(1, (std::max(width, height) + kMaxPngSide - 1) / kMaxPngSide);
  out_width = (width + factor - 1) / factor;
  out_height = (height + factor - 1) / factor;
  std::vector<uint8_t> out(static_cast<size_t>(out_width) * out_height, kUnknownPixel);
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      const uint8_t value = raster[static_cast<size_t>(row) * width + col];
      uint8_t& pixel = out[static_cast<size_t>(row / factor) * out_width + col / factor];
      if (value == kOccupiedPixel || (value == kFreePixel && pixel == kUnknownPixel)) {
        pixel = value;
      }
    }
  }
  return out;
}

std::string Pgm(const std::vector<uint8_t>& raster, int width, int height) {
  std::string pgm = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
  pgm.append(raster.begin(), raster.end());
  return pgm;
}

template <typename... Args>
std::string Format(const char* format, Args... args) {
  char buffer[160];
  std::snprintf(buffer, sizeof(buffer), format, args...);
  return buffer;
}

// free_thresh 0.196 so 205 = (255 - 205) / 255 reads back as unknown, not free.
std::string MapYaml(const mapping::GridMapu8& grid) {
  std::ostringstream out;
  out.precision(12);
  out << "image: map.pgm\n"
      << "mode: trinary\n"
      << "resolution: " << grid.resolution() << "\n"
      << "origin: [" << grid.origin_x() << ", " << grid.origin_y() << ", 0.0]\n"
      << "negate: 0\n"
      << "occupied_thresh: 0.65\n"
      << "free_thresh: 0.196\n";
  return out.str();
}

std::optional<common::Time> NewestTime(const std::vector<SnapshotNode>& nodes) {
  std::optional<common::Time> newest;
  for (const SnapshotNode& node : nodes) {
    if (!newest.has_value() || node.time > *newest) {
      newest = node.time;
    }
  }
  return newest;
}

std::string MapJson(const SnapshotInput& input) {
  JsonWriter writer;
  const std::optional<common::Time> newest = NewestTime(input.nodes);
  writer.BeginObject()
      .Field("boot_count", input.boot_count)
      .Field("num_solves", input.num_solves)
      .Field("generated_at_ns", common::ToUnixNanos(input.generated_at))
      .Key("newest_node_ns");
  if (newest.has_value()) {
    writer.Int(common::ToUnixNanos(*newest));
  } else {
    writer.Null();
  }
  writer.Key("fed_session");
  if (input.fed_session.has_value()) {
    writer.Int(*input.fed_session);
  } else {
    writer.Null();
  }
  writer.Field("resolution", input.grid.resolution())
      .Key("origin")
      .BeginArray()
      .Double(input.grid.origin_x())
      .Double(input.grid.origin_y())
      .EndArray()
      .Field("width", input.grid.width())
      .Field("height", input.grid.height())
      .EndObject();
  return writer.str() + "\n";
}

std::string TrajectoryCsv(const std::vector<SnapshotNode>& nodes) {
  std::string csv = "stamp_ns,x,y,theta,session\n";
  for (const SnapshotNode& node : DecimateTrajectory(nodes, 1.0)) {
    csv += std::to_string(common::ToUnixNanos(node.time)) + "," +
           Format("%.4f,%.4f,%.5f", node.global_pose.translation().x(),
                  node.global_pose.translation().y(), utils::transform::GetYaw(node.global_pose)) +
           "," + std::to_string(node.session) + "\n";
  }
  return csv;
}

std::string PlacesJson(const std::vector<PlaceRow>& places) {
  JsonWriter writer;
  writer.BeginArray();
  for (const PlaceRow& place : places) {
    writer.BeginObject()
        .Field("path", place.path)
        .Field("anchor", place.anchor)
        .Field("state", place.state)
        .OptionalField("orphan_reason", place.orphan_reason);
    writer.Key("pose");
    if (place.pose.has_value()) {
      writer.BeginObject()
          .Field("x", place.pose->translation().x())
          .Field("y", place.pose->translation().y())
          .Field("theta", utils::transform::GetYaw(*place.pose))
          .EndObject();
    } else {
      writer.Null();
    }
    writer.EndObject();
  }
  writer.EndArray();
  return writer.str() + "\n";
}

std::string Summary(const SnapshotInput& input, int seq) {
  using common::operator<<;
  std::ostringstream out;
  const mapping::GridMapu8& grid = input.grid;
  out << "snapshot " << SequenceName(seq) << ", solve " << input.num_solves << ", boot "
      << input.boot_count << ", generated " << input.generated_at << "\n";
  if (HasCells(grid)) {
    int unknown = 0;
    for (const uint8_t cell : grid.data()) {
      unknown += mapping::IsKnownValue(cell) ? 0 : 1;
    }
    const double x1 = grid.origin_x() + grid.width() * grid.resolution();
    const double y1 = grid.origin_y() + grid.height() * grid.resolution();
    out << "map: " << grid.width() << " x " << grid.height() << " cells at " << grid.resolution()
        << " m\n"
        << Format("bounds: x [%.2f, %.2f] m, y [%.2f, %.2f] m\n", grid.origin_x(), x1,
                  grid.origin_y(), y1)
        << Format("unknown fraction: %.3f\n",
                  static_cast<double>(unknown) / static_cast<double>(grid.data().size()));
  } else {
    out << "map: empty (no finished submap)\n";
  }
  if (input.robot.has_value()) {
    out << Format("robot: x %.2f y %.2f theta %.3f\n", input.robot->translation().x(),
                  input.robot->translation().y(), utils::transform::GetYaw(*input.robot));
  } else {
    out << "robot: unknown\n";
  }
  out << "sessions:\n";
  for (const SnapshotSession& session : input.sessions) {
    out << "  " << session.id << " " << session.role << ", " << session.num_nodes << " nodes, "
        << session.num_submaps << " submaps\n";
  }
  out << "places (path anchor state x y theta dist_m):\n";
  for (const PlaceRow& place : input.places) {
    out << "  " << place.path << " " << place.anchor << " " << place.state;
    if (place.pose.has_value()) {
      out << Format(" %.2f %.2f %.3f", place.pose->translation().x(), place.pose->translation().y(),
                    utils::transform::GetYaw(*place.pose));
      if (input.robot.has_value()) {
        out << Format(" %.2f", (place.pose->translation() - input.robot->translation()).norm());
      } else {
        out << " -";
      }
    } else {
      out << " - - - -";
    }
    out << "\n";
  }
  return out.str();
}

bool WriteBytes(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  stream.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  if (!stream) {
    LOG(ERROR) << "cannot write " << path;
    return false;
  }
  return true;
}

}  // namespace

uint8_t TrinaryValue(uint8_t cell) {
  if (!mapping::IsKnownValue(cell)) {
    return kUnknownPixel;
  }
  return mapping::ValueToProbability(cell) > 0.5 ? kOccupiedPixel : kFreePixel;
}

std::vector<SnapshotNode> DecimateTrajectory(std::vector<SnapshotNode> nodes, double min_arc_m) {
  std::stable_sort(nodes.begin(), nodes.end(), [](const SnapshotNode& a, const SnapshotNode& b) {
    return a.session != b.session ? a.session < b.session : a.time < b.time;
  });
  std::vector<SnapshotNode> kept;
  double arc = 0.0;
  for (size_t i = 0; i < nodes.size(); ++i) {
    const bool first = i == 0 || nodes[i - 1].session != nodes[i].session;
    const bool last = i + 1 == nodes.size() || nodes[i + 1].session != nodes[i].session;
    if (!first) {
      arc += (nodes[i].global_pose.translation() - nodes[i - 1].global_pose.translation()).norm();
    }
    if (first || last || arc >= min_arc_m) {
      kept.push_back(nodes[i]);
      arc = 0.0;
    }
  }
  return kept;
}

int NextSequenceNumber(const std::string& dir) {
  int highest = 0;
  std::error_code error;
  for (std::filesystem::directory_iterator it(dir, error), end; !error && it != end;
       it.increment(error)) {
    const std::string name = it->path().filename().string();
    if (name.size() < 6 ||
        !std::all_of(name.begin(), name.begin() + 6, [](char c) { return std::isdigit(c) != 0; })) {
      continue;
    }
    highest = std::max(highest, std::stoi(name.substr(0, 6)));
  }
  return highest + 1;
}

std::string SequenceName(int seq) {
  char name[16];
  std::snprintf(name, sizeof(name), "%06d", seq);
  return name;
}

SnapshotExporter::SnapshotExporter(std::string snapshots_dir)
    : snapshots_dir_(std::move(snapshots_dir)), next_seq_(NextSequenceNumber(snapshots_dir_)) {}

std::optional<SnapshotResult> SnapshotExporter::Export(const SnapshotInput& input) {
  SnapshotResult result;
  result.seq = next_seq_++;
  const std::filesystem::path final_dir =
      std::filesystem::path(snapshots_dir_) / SequenceName(result.seq);
  const std::filesystem::path temp_dir = final_dir.string() + ".tmp";
  std::error_code error;
  std::filesystem::remove_all(temp_dir, error);
  std::filesystem::create_directories(temp_dir, error);
  if (error) {
    LOG(ERROR) << "cannot create " << temp_dir << ": " << error.message();
    return std::nullopt;
  }

  std::vector<std::pair<std::string, std::string>> files;
  if (HasCells(input.grid)) {
    const int width = input.grid.width();
    const int height = input.grid.height();
    const std::vector<uint8_t> raster = TrinaryRaster(input.grid);
    files.emplace_back("map.pgm", Pgm(raster, width, height));
    files.emplace_back("map.yaml", MapYaml(input.grid));
    int png_width = 0;
    int png_height = 0;
    const std::vector<uint8_t> small = Downsample(raster, width, height, png_width, png_height);
    const std::vector<uint8_t> png = EncodeGrayPng(small, png_width, png_height);
    if (png.empty()) {
      LOG(ERROR) << "PNG encoding failed";
      return std::nullopt;
    }
    files.emplace_back("map.png", std::string(png.begin(), png.end()));
  }
  files.emplace_back("map.json", MapJson(input));
  files.emplace_back("trajectory.csv", TrajectoryCsv(input.nodes));
  files.emplace_back("places.json", PlacesJson(input.places));
  files.emplace_back("summary.txt", Summary(input, result.seq));

  for (const auto& [name, bytes] : files) {
    if (!WriteBytes(temp_dir / name, bytes)) {
      return std::nullopt;
    }
    result.files.push_back(name);
  }
  std::filesystem::rename(temp_dir, final_dir, error);
  if (error) {
    LOG(ERROR) << "cannot rename " << temp_dir << " to " << final_dir << ": " << error.message();
    return std::nullopt;
  }
  result.dir = final_dir.string();
  return result;
}

}  // namespace evergreenslam::agent
