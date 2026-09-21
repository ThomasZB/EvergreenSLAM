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
| `pose_graph_publisher` | the lifelong graph as rviz topics: submap list and textures, trajectory and constraint markers, global map |

## Build

`adapters/ros2/` is a colcon workspace source: clone the repo under any `ws/src` and

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash          # .zsh on macOS
ros2 launch evergreenslam_ros laser_odometry.launch.py
```

builds `evergreenslam_msgs`, this package and the rviz plugin (`adapters/ros2/evergreenslam_rviz`).
`core/` and `apps/webui/` carry `COLCON_IGNORE`: they are not packages, this package pulls them
in with `add_subdirectory`.

Day to day, through pixi + RoboStack (macOS Apple Silicon or Linux; rviz2 and `ros2 bag play`
share the loopback with the node, so DDS discovery needs no network setup). The tasks wrap the
same colcon commands with `build-pixi/{colcon,install}` as the bases:

```bash
pixi run -e ros ros-test                                  # colcon build + test
SCAN_TOPIC=/base_scan BASE_FRAME=base_footprint pixi run -e ros bag bags/wg_cafe
pixi run -e ros live                                      # then: pixi run -e ros rviz
```

Or in Docker. `docker-compose.yml` at the repo root wraps every command below —
`docker compose run --rm core-test` / `ros-build`, `BAG=... docker compose up bag`,
`docker compose up live`; see the header comment there for the full list and the tunable env
vars. The raw commands:

```bash
docker build -t evergreenslam:ros2 adapters/ros2/evergreenslam_ros
docker run --rm -v "$PWD":/ws -v /tmp/out:/out evergreenslam:ros2 bash -lc \
  "colcon --log-base /out/log build --base-paths /ws/adapters/ros2 --build-base /out/colcon \
   --install-base /out/install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
   && colcon --log-base /out/log test --base-paths /ws/adapters/ros2 --build-base /out/colcon \
   --install-base /out/install && colcon test-result --test-result-base /out/colcon --verbose"
```

The same image also builds and tests `core` on its own, which is worth doing before any release:
Apple clang and gcc disagree about undefined behaviour, so a suite that is green on macOS can
still break at `-O2` on Linux.

```bash
docker run --rm -v "$PWD":/ws evergreenslam:ros2 bash -lc \
  "cmake -S /ws/core -B /tmp/bt -DEVERGREENSLAM_BUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release \
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

The classic 2D SLAM datasets (Intel Research Lab, ACES, Freiburg 079, MIT Killian Court) exist
only as CARMEN logs, from the Freiburg SLAM evaluation page
(`ais.informatik.uni-freiburg.de/slamevaluation/datasets.php`). `tools/convert_carmen_log.py`
turns a `.clf` into a rosbag2 with `/scan` and `/odom` (the odometry there is wheel odometry,
not ground truth — the `.relations` files next to the logs are the truth). CARMEN logs carry no
per-beam timestamps, so the scans convert with zero time offsets and undistortion degenerates
to a no-op; acceptable for these old 180-degree scanners. Scans come out in frame `laser`:
pass `--base_from_laser <robot_frontlaser_offset>,0,0` (the converter prints the offset).

```bash
python3 tools/convert_carmen_log.py intel.clf bags/intel
```

## Replay a bag

```bash
bag_laser_odometry --bag <rosbag2 dir> --config configs/evergreenslam.yaml \
  --scan_topic /base_scan --base_frame base_footprint --reference_frame map
```

The reference trajectory comes from either `--reference_frame` (walks the recorded tf tree) or
`--odom_topic` (a `nav_msgs/Odometry` topic). Without a tf tree, pass the sensor mounting as
`--base_from_laser x,y,theta`. Writes `<out_prefix>_trajectory.csv` and `<out_prefix>_map.pgm`.

The lifelong backend (pose graph, loop closure, freezing, trimming, persistence) runs by
default; `--no_backend` keeps the raw laser odometry as a baseline. With the backend the tool
also writes `<out_prefix>_nodes.csv` (optimized keyframe poses) and
`<out_prefix>_global_map.pgm` (all sessions' submaps composed at optimized poses), and
`--map_dir <dir>` loads a saved map at boot and checkpoints into it during the run — running
the same command twice continues the first run's map.

Reported: absolute error against the reference (rms / worst / final), per-scan relative error,
final drift as a percentage of the reference path, and — with the backend — loop closure,
freeze, trim and checkpoint counts plus the optimized absolute error rms next to the odometry
baseline.

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
`publish_tf` (`true`), `map_publish_period` (`1.0` s), `webui_port` (`0` = off), `lifelong`
(`true`), `map_dir` (`""` = no persistence), `map_frame` (`map`).

With the backend on (the default), `/map` carries the assembled global map in `map_frame` and
tf gains `map -> odom` with the optimizer's correction, so RViz composes the corrected robot
pose while `/odom` never jumps. With `lifelong:=false` the published grid falls back to the
newest active submap in the odom frame, covering roughly the last `num_scans_per_submap` scans.

### Pose graph topics

With the backend on, a timer of period `map_publish_period` also publishes the graph itself.
Every message is stamped with the newest keyframe's **sensor** time, not the node clock, so a
bag replayed on sim time stays inside RViz's tf window; nothing is published before the first
keyframe. Names are relative, so a node namespace or a remap applies.

| name | type | what it carries |
| --- | --- | --- |
| `submap_list` | `evergreenslam_msgs/SubmapList` | one entry per submap: ids, `submap_version`, global pose in `map_frame`, `is_finished`, `is_frozen` |
| `submap_query` | `evergreenslam_msgs/SubmapQuery` (service) | the occupancy texture of one submap |
| `trajectory_node_list` | `visualization_msgs/MarkerArray` | one `LINE_STRIP` per session through its optimized node poses; golden-angle hue per session, frozen sessions at half alpha |
| `constraint_list` | `visualization_msgs/MarkerArray` | `LINE_LIST`s grouped as intra (grey), inter (green), prior (amber), recovered (magenta) and inter residuals (red) |
| `map` | `nav_msgs/OccupancyGrid` | the assembled global map, republished only when it changes |

`submap_list`, both marker arrays and `map` are `QoS(1).transient_local()`, so RViz started
late still gets the current state. A plain `ros2 topic echo` needs
`--qos-durability transient_local --qos-reliability reliable` to see the latched message.

`submap_version` is the submap's scan count: it grows while the submap is open and stops once
the submap is finished, so a viewer re-queries a texture exactly when that number changes.
`SubmapQuery` answers from a cache the backend task refreshes each cycle — the service callback
never touches the graph. On failure (unknown, trimmed or still empty submap) `error_message` is
set and the texture comes back with `width = height = 0`.

A texture's `cells` are one byte per cell, `0..100` occupancy in percent and `255` unknown,
row-major with x fastest. Cell `(0, 0)` is the corner at `slice_pose`, which is the grid corner
expressed in the submap frame; a world point is `entry.pose * (slice_pose * (x * resolution,
y * resolution))`. No `local_pose` is involved anywhere.
