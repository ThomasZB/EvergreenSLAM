# evergreenslam_ros

ROS 2 adapter for EvergreenSLAM. `core/` stays free of ROS; everything ROS lives here.

Built and tested on **ROS 2 Jazzy / Ubuntu 24.04**. Jazzy rather than Humble because its
yaml-cpp 0.8 exports the namespaced `yaml-cpp::yaml-cpp` target that `core/CMakeLists.txt`
already links, so `core` needs no change; Humble's 0.7 exports only the bare target.

## Contents

| target | what it is |
| --- | --- |
| `bag_laser_odometry` | reads a rosbag2 directly and scores the result against a reference trajectory in the same bag. The primary verification path. |
| `laser_odometry_node` | live node: `/scan` in, `/odom` + tf + `OccupancyGrid` out |
| `laser_scan_converter` | `sensor_msgs/LaserScan` to `sensor::PointCloud` |

## Build

`docker-compose.yml` at the repo root wraps every command below — `docker compose run --rm
core-test` / `ros-build`, `BAG=... docker compose up bag`, `docker compose up live`; see the
header comment there for the full list and the tunable env vars. The raw commands:

```bash
docker build -t evergreenslam:ros2 adapters/ros2
docker run --rm -v "$PWD":/ws -v /tmp/out:/out evergreenslam:ros2 bash -lc \
  "cmake -S /ws/adapters/ros2 -B /out/ros_build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
   && cmake --build /out/ros_build -j8 && cd /out/ros_build && ctest --output-on-failure"
```

The same image also builds and tests `core` on its own, which is worth doing before any release:
Apple clang and gcc disagree about undefined behaviour, so a suite that is green on macOS can
still break at `-O2` on Linux.

```bash
docker run --rm -v "$PWD":/ws evergreenslam:ros2 bash -lc \
  "cmake -S /ws -B /tmp/bt -DEVERGREENSLAM_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release \
   && cmake --build /tmp/bt -j8 && cd /tmp/bt && ctest"
```

## Getting a bag

Public ROS 2 bags carrying 2D `LaserScan` are scarce; ROS 1 ones are plentiful. `rosbags`
converts them and needs no ROS installation:

```bash
pip install rosbags
curl -O http://download.ros.org/data/graph_slam/wg-cafe.bag
```

Bags from around 2010 need three fixes that plain `rosbags-convert` does not make: topic names
lack the leading `/` that ROS 2 requires, `frame_id`s keep a leading `/` that tf2 rejects, and
their message definitions reference `roslib/Header`, which aborts the type resolution.
`tools/convert_ros1_bag.py` makes all three:

```bash
python3 tools/convert_ros1_bag.py wg-cafe.bag bags/wg_cafe
```

Newer bags also convert with `rosbags-convert --src in.bag --dst out_dir --dst-version 8`.

## Replay a bag

```bash
bag_laser_odometry --bag <rosbag2 dir> --config configs/evergreenslam.yaml \
  --scan_topic /base_scan --base_frame base_footprint --reference_frame map
```

The reference trajectory comes from either `--reference_frame` (walks the recorded tf tree) or
`--odom_topic` (a `nav_msgs/Odometry` topic). Without a tf tree, pass the sensor mounting as
`--base_from_laser x,y,theta`. Writes `<out_prefix>_trajectory.csv` and `<out_prefix>_map.pgm`.

Reported: absolute error against the reference (rms / worst / final), per-scan relative error,
and final drift as a percentage of the reference path.

Add `--webui_port 8642` to watch it in a browser; see `apps/webui/README.md`. That paces replay
to real time, so pass `--speed 4` (or `--speed 0` for unpaced) after it. The estimate is
unaffected either way — the sink is const throughout and the whole `wg-cafe` run comes out
identical with and without it.

### When the error looks bad, check the reference first

`--map_from_reference` draws the map at the reference poses instead of the estimated ones,
through the identical code path. A reference that is itself drifting smears its own map. This
decides which trajectory is wrong using only the scans, without assuming either one is right —
on `gmapping/hallway_slow`, whose `/odom` carries an uncompensated ~0.18 deg/s gyro bias, the
reference draws one corridor twice at ~40 degrees while the estimate draws it once.

## Live node

```bash
ros2 launch evergreenslam_ros laser_odometry.launch.py
ros2 bag play <rosbag2 dir>
```

The launch file reads the ROS-side parameters (topic, frames, webui port) from
`config/config.yaml`; edit that file rather than passing overrides. Running the executable
directly still works — the scan topic is a parameter, with a remap-friendly default:

```bash
ros2 run evergreenslam_ros laser_odometry_node --ros-args \
  -p config:=configs/evergreenslam.yaml -p base_frame:=base_footprint -r scan:=/base_scan
```

Subscribes with `SensorDataQoS()`. A RELIABLE subscriber receives nothing at all from a
BEST_EFFORT publisher and reports no error, and bag players and lidar drivers are usually
BEST_EFFORT.

Parameters: `config`, `scan_topic` (`scan`), `odom_frame` (`odom`), `base_frame` (`base_link`),
`publish_tf` (`true`), `map_publish_period` (`1.0` s), `webui_port` (`0` = off).

The published grid is the newest active submap, so it covers roughly the last
`num_scans_per_submap` scans and resets when a submap rotates. It is not a global map; that
arrives with the pose graph. `bag_laser_odometry` writes a whole-run map to `_map.pgm` instead.
