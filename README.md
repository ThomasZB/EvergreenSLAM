# EvergreenSLAM

Always-on 2D lidar SLAM with named places, as a spatial memory for LLM agents.

[![ci](https://github.com/ThomasZB/EvergreenSLAM/actions/workflows/ci.yml/badge.svg)](https://github.com/ThomasZB/EvergreenSLAM/actions/workflows/ci.yml)

English | [中文](docs/README.zh.md)

---

## Hello, world

```
$ egs here
current places/dock 2.4m
closest places/dock 2.4m frozen; places/kitchen 15.3m frozen
robot 0.48,1.02,-2.72  keyframe 0s  @solve 135
base 0.75 (cover 98%)  since base match 0.0m  match 0.70 (avg 0.70)
```

<p align="center">
  <img src="docs/images/demo.gif" width="960" alt="Replay of the revo_lds bag in the browser debug view: the map grows, sessions 0 and 1 freeze, session 2 starts">
  <br>
  <sub><code>pixi run -e ros demo</code>: Cartographer's public revo_lds bag (a Neato Revo LDS, 16 min of
  driving) replayed at 8x in the browser debug view. Sessions #0 and #1 freeze in turn and #2 starts
  mapping.</sub>
</p>

An agent asked to fetch a cola from the fridge understands the request but not the home: where
the fridge is, how to reach the kitchen, where the cola was last seen. Whatever model is used, it
needs a spatial memory: it must be able to remember a place from a command, a sentence or a photo,
send the robot back there later, and keep that memory across context compaction, map
re-optimization and process restarts. EvergreenSLAM is a 2D lidar SLAM designed around this
requirement.

- **One mode, no operator.** It runs as a background process with no separate mapping and
  localization modes: it always maps into the current *session* while localizing against sessions
  it has already *frozen*, and a session freezes itself once it has added enough new area and its
  poses are well constrained.
- **Bounded by area, not uptime.** Trimming marginalizes what it removes and sparsifies the result
  to a Chow-Liu tree, and frozen poses are held constant in the optimizer, so that memory and CPU
  are bounded by the mapped area rather than by uptime.
- **Places that keep resolving.** A place is an *anchor* bound to a submap of the pose graph rather
  than to coordinates, exposed to the agent as a directory tree plus a small CLI (`egs`, also
  served as MCP tools).
- **One cheap sensor.** A 2D lidar and a CPU; no IMU, wheel odometry, camera or GPU. It has been run
  on a 215 CNY lidar and an RK3588 board, and on the public 2D datasets.

The scan matching and the submaps follow Cartographer; the lifelong layer above them is the
project's own. How it relates to slam_toolbox is under Related work.

<p align="center">
  <img src="docs/images/car.jpg" width="640" alt="The test robot: a 2D lidar on an Orange Pi, mecanum wheels, two power banks">
  <br>
  <sub>The test platform: a 2D lidar (215 CNY, about 30 USD), an Orange Pi with an RK3588, mecanum wheels, two power banks.</sub>
</p>

## Motivation

**The agent does not know the home.** Language is no longer the bottleneck: an agent asked for a
cola from the fridge knows what is wanted. What it lacks is the layout. Its memory is text, and a
layout described in enough detail to act on takes a lot of context and survives only as long as
the context does, compaction included.

**Coordinates are fragile.** Writing places down as map coordinates does not hold for long. A
coordinate goes stale when the map is rebuilt or re-optimized, often without the agent noticing,
and means nothing in another map. Every agent on a conventional SLAM would have to keep its own
record of where things are and keep that record consistent with the map. What an agent can use is
a *name* that keeps resolving to the same physical spot, plus a place to record what it saw there.

**SLAM without an operator.** The usual workflow: tune the odometry, calibrate extrinsics and
synchronize timestamps, drive around to map, save, switch to localization mode, tune parameters
when the environment changes, and rebuild when it changes too much. Vendors usually package this
or train their customers. For a newcomer a mistake shows up as ghosting or drift, sometimes only
later, and a rebuild means entering every saved station again. Each step needs an operator, and
the agent should not have to be one.

**The intended use.** For example: walk the robot around the home once, saying "this is the
kitchen" in the kitchen and "this is the desk" at the desk, and do nothing else about mapping or
localization. The next day, "go and check the kitchen" works. The agent notes that the scissors
are in the left desk drawer and the apples on the dining table, and days later "where are the
scissors?" leads it to the right place. Furniture moves, the map is re-optimized, the robot
restarts, and the places still resolve. Fetching the cola then starts from knowing that the fridge
is in the kitchen and how to get there, before an arm takes over. The later steps need
manipulation and vision; the layer underneath, places that stay valid without anyone maintaining
the map, is what this project argues SLAM itself should provide.

**Lifelong SLAM, but where.** Underneath this is lifelong SLAM: always on, mapping while in use,
following changes, with resources that do not grow with uptime. The research literature on it is
long, yet to the author's knowledge few open-source 2D implementations can be used as they are;
slam_toolbox's lifelong node is described by its README as highly experimental and removes nodes
without marginalizing them (see Related work).

**Cost.** The memory should be cheap enough to leave running on a small robot all day. A 3D lidar
costs a few hundred dollars and up and needs a board that can keep up with it. Cameras are cheap,
but dense or learned visual SLAM usually wants more compute than a small board has, and cameras
degrade in darkness, on textureless walls and under changing illumination. A 2D lidar costs about
30 dollars, needs no GPU, and is insensitive to lighting; its own failure cases (glass, mirrors,
strong sunlight) are not addressed here. On the same grounds the pipeline fuses no IMU, wheel
odometry or camera: each adds calibration, time synchronization, a driver and per-robot tuning.
The cost is that motion prediction comes from the laser alone, a Kalman filter with a Singer
motion model fed only by scan matching, which is expected to struggle on fast turns and in long
featureless stretches. The position taken is that cheap geometry with sparse semantics attached to
named places (for example, a vision-language model occasionally writing "the bin is here") is
for this setting a better trade-off than dense visual SLAM.

## How it works

### Sessions and freezing

Mapping is organized in sessions. The frontend always maps into the current session; the backend
closes loops against every finished submap within reach, frozen or not, including the current
session's own, which the rolling window below keeps recent. Under gradual environmental change
the robot therefore keeps matching the world as it is now, and those recent submaps are in turn
constrained to the frozen base.

The first session becomes the frozen base once it has a finished submap. A later session is
judged only after it has added enough new area outside the frozen footprint and has stopped
growing; it freezes when it is linked to a frozen session and the marginal standard deviations
of every finished submap pose are below per-axis thresholds. Freezing makes the session's files immutable and its poses
constant in every later optimization. A session that only revisits known space is never frozen
automatically (`egs session freeze --yes` does it by hand); any unfrozen session is discarded
after more than five boots without being fed (configurable). Where sessions overlap, the composed map keeps the more confident cell.
Consequently a change inside already-mapped ground is visible while the current session is alive
but is not frozen in; re-freezing a changed room is planned, not implemented.

<p align="center">
  <img src="docs/images/webui.jpg" width="900" alt="The browser debug view: submaps, trajectory and four sessions, three of them frozen">
  <br>
  <sub>The browser debug view during a long run: four sessions, three of them frozen.</sub>
</p>

### Bounded growth

Two mechanisms keep the graph from growing with uptime.

*Trimming with sparsification.* The live session keeps a rolling window of its own submaps over
ground it re-covers: an older submap is removed once newer submaps cover its area or newer passes
cover its track. Removal is a marginalization, not a deletion: the removed nodes' Markov blanket
is taken, the joint covariance of the survivors is computed, and the resulting dense information
is sparsified to a Chow-Liu tree (the maximum spanning tree over pairwise mutual information)
whose edges replace the removed constraints; edges that would span too far become priors weighted
by the marginal covariance. At freeze, finished submaps that lie mostly inside the frozen map are
dropped the same way, keeping one if all qualify.

*Frozen sessions are constant.* Frozen poses are parameter blocks held constant in the Ceres
problem, no constraint is built between two frozen submaps, and frozen submaps are never
modified. The optimization therefore scales with the live session, not with the whole map, and
the frozen part grows linearly with mapped area. A simulated 30-loop run stays bounded. On the
robot it has run in one flat over several days, but memory and graph size were not logged then;
the numbers will be added.

### Places as anchors

A saved place is an anchor: a submap of the pose graph plus a transform relative to it. When the
graph is re-optimized the place moves with its submap; after a restart the name still resolves.
Anchors survive loop closure, trimming, freezing, restart and an interrupted write, and are what
`egs where` resolves.

### Memory as a directory

`memory/places/kitchen/table/apple` is a thing inside a place. The agent works on the tree with
`ls`, `cat`, `mv` and `grep`. Only what the live process alone knows, such as poses, saving a place,
and map or session management, goes through `egs`. Keep-out zones, notes and per-place skills live
in the same tree. `egs view here` renders the current scan over the map around the robot so the
agent can judge for itself whether localization is drifting.

### Implementation

Pure 2D, all `double`, poses as `Eigen::Affine2d`. Laser odometry with a correlative matcher and a
Ceres refinement, submaps as probability grids, a pose graph with branch-and-bound loop closure and
Ceres optimization, then session-aware trimming, freezing and persistence with atomic commits.
`core/` has no ROS dependency; ROS 2 is one adapter.

## Field test

On 2026-09-29 Claude Code drove the platform above through a flat. The agent had a small
wrapper tool (`mochibot`) around `egs` and a Nav2 navigator, plus a skill for that flat
(`robot-places`) written on top of the `evergreenslam` skill in this repository. The transcript
is in Chinese; the steps, with the `egs` calls behind each, were:

1. **"You are the agent driving the car. Wait for my orders."** It loads the skill, runs `egs here`
   and reports: localized, next to the desk, bag recording started.
2. **"Go check whether anything changed in the kitchen."** It reads the kitchen's `SKILL.md` in the
   memory directory, which says to stop at the doorway on top of the ramp. The saved place is the
   doorway, so `egs where places/kitchen/door` gives it the approach pose, which it hands to the
   navigator.
3. Crossed the threshold, about 6 m to go.
4. Just past the door it gets wedged: an air purifier 30 to 40 cm ahead, the door frame on the
   right, and Nav2 cannot turn. It backs straight up a short distance and re-plans.
5. Free, 0.6 m from the doorway. It watches the robot stop on top of the ramp.
6. 18 cm and 12° off the saved pose. It judges that sufficient, cancels navigation before the robot
   starts shuffling at the edge of the ramp, and takes a photo.
7. It compares the photo with the last one on record: same layout, but a new object in the lower
   left, probably a mop or a folding stool, partly blocking the bin. It records the observation in
   the kitchen's log with `egs observe` and waits.

<p align="center">
  <img src="docs/images/agent.jpg" width="900" alt="Claude Code driving the robot to the kitchen">
</p>

Driving, backing up, the photo and the comparison belong to the wrapper and to Nav2; `egs` only
answers where things are and keeps the record. No coordinate was typed by the user; poses passed
from `egs` to the navigator.

After the run the same agent (Claude Code with Opus 5.5) was asked for a short assessment of its
use so far, which covered more than this trip and predates the v0.2 features. In summary: it
started from an empty map on this platform, saved the kitchen and the desk, later returned by name
to check for changes, and after a restart the map and the pose were recovered without
intervention; almost everything was `egs` commands and an ordinary directory, with no raw
coordinates and no ROS details. The platform has since run in the same flat over several days;
after the fixes those days prompted, the agent has not been stuck on localization or on the map.

## Validation and limitations

This is a preview, not a product. Scope was limited to the core; known omissions follow.

Tested on macOS and Linux: unit tests, simulated-lidar end-to-end tests for the frontend, the scan
matcher and each backend stage (loop closure, trimming, freezing, persistence, relocalization,
anchors). CI runs the core and agent tests on Linux (gcc) and macOS on every push; the
ThreadSanitizer run over the backend tests and the ROS 2 adapter build are by hand. Exercised
on the platform above and on the public bags: laser odometry, loop closure, multi-session mapping
with automatic freezing and trimming, persistence with atomic commits and named maps, the agent
service and `egs`, the browser debug view, the rviz2 plugin. On the datasets the replay tool
reports the error against the reference trajectory (see the ROS README); no results table is
kept here yet.

- No IMU, wheel odometry or camera fusion, and no 3D (see Motivation).
- Frozen sessions cannot be unfrozen. A change inside mapped ground is mapped and matched by the
  current session, which stays aligned through its loop closures to the frozen base, so
  localization holds; the change is not frozen in, though (see Sessions and freezing). A map that
  diverged and was then frozen means starting a new map directory.
- No dynamic-object filtering. Odometry has no remedy for degenerate geometry (long corridors,
  open halls); loop closure rejects ambiguous matches in corridors.
- One robot, one map. No multi-robot support.
- Kidnap detection starts a global search on its own when the frontend's tracking broke, the scan
  disagrees with the frozen base and no base closure came for 3 m (`lifelong.kidnap_detector`,
  `kidnap` line in `egs here`). It finds a robot booted onto the wrong spot of the base; a robot
  carried mid-run is detected, but its session is already pinned to the base and the search's
  few edges are outvoted, so `egs here` names the recovery (`egs session rm`, then `egs
  relocalize`). A robot put down where the base has no map reads as exploring, and goes unnoticed.
  Thresholds come from one replay and the simulated room, not from robots carried on purpose.
- The freeze thresholds have not been calibrated against a reference trajectory.
- One known odometry defect: in the simulated room, a straight run from rest along a wall-aligned
  axis has been observed to stay at the origin. The regression test is disabled until the cause is
  understood.
- Everything in agent layer v0.2 (keep-out zones as a Nav2 keep-out filter mask, in-process removal
  of the session being mapped, `--json` output, `observe --attach`, `place save --offset`, the
  match score in `egs here`, the `/odom` twist) passes its tests but has not run on the robot, and
  the Nav2 side of the keep-out mask has not been checked against a running Nav2 (the KeepoutFilter
  setup is in the ROS README).

Evaluated only in one flat and on the public datasets. Issues welcome.

## Building and running

Core alone, no ROS:

```bash
cmake -S core -B build -DEVERGREENSLAM_BUILD_TESTS=ON && cmake --build build -j8 && ctest --test-dir build
```

Dependencies: Eigen, Ceres, glog, yaml-cpp, protobuf (library and `protoc`), OpenMP, gtest.

```bash
# macOS
brew install eigen ceres-solver glog yaml-cpp protobuf libomp googletest
# Ubuntu 24.04 (core links yaml-cpp 0.8; 22.04's package is too old)
sudo apt install libeigen3-dev libceres-dev libgoogle-glog-dev libyaml-cpp-dev \
                 libprotobuf-dev protobuf-compiler libgtest-dev
```

Or let [pixi](https://pixi.sh) fetch everything including ROS 2 Jazzy and rviz2 (macOS Apple
Silicon and Linux; the header of `pixi.toml` lists every task):

```bash
pixi run core-test                      # core alone
pixi run agent-test                     # agent service and egs
pixi run -e ros ros-test                # ROS 2 adapter
SCAN_TOPIC=/base_scan BASE_FRAME=base_footprint pixi run -e ros bag bags/wg_cafe   # replay, browser view at http://localhost:8642
pixi run -e ros live                    # live node: /scan in, map, tf and the agent service out
pixi run -e ros demo                    # fetch a 3 MB public bag, replay it with the browser view and egs
```

`live` expects a lidar driver publishing `sensor_msgs/LaserScan` on `/scan` (`SCAN_TOPIC` changes
it) and a TF from `base_link` to the laser frame when the two differ. It always publishes `/odom`
from laser odometry (without covariance), and with `publish_tf` both the odom to base_link and the
map to odom transforms; if the base driver already publishes odom to base_link, switch that one
off, because `publish_tf=false` drops map to odom as well, and remap one of the two `/odom`
topics. Start from `configs/evergreenslam.yaml`; the museum and carmen configs are for the museum
and CARMEN datasets. Maps persist under `runs/maps` with pixi (`MAP_ROOT` changes it; `bag`
persists only when `MAP_ROOT` is set). pixi builds for osx-arm64 and linux-64; on a Linux ARM
board use the system packages or colcon. Tested on ROS 2 Jazzy.

The repository is also a plain colcon workspace source: clone it into any `ws/src`, then
`rosdep install --from-paths src --ignore-src -y && colcon build`. Docker Compose wraps the same
targets for Linux (`docker compose run --rm core-test`).

On a board without a real-time clock, start the node after time synchronization: order its
systemd unit `After=time-sync.target` with `Wants=time-sync.target`, and enable the wait service
of the time daemon (`chrony-wait.service` or `systemd-time-wait-sync.service`). A scan timestamp
that moves backwards, or forwards by more than `max_scan_gap_seconds` (60 s by default), is
treated as a clock step: the node saves the map and exits with a non-zero status, and on restart
re-aligns from the last saved pose. It therefore needs a supervisor, either `Restart=on-failure`
in systemd or `respawn` in ROS 2 launch (`evergreenslam.launch.py` sets it). The replay tool stops
with an error in the same situation.

### Data

No data ships with the repository. Most public 2D lidar bags are ROS 1; two scripts convert them
without a ROS install:

```bash
pip install rosbags
curl -O http://download.ros.org/data/graph_slam/wg-cafe.bag
python3 tools/convert_ros1_bag.py wg-cafe.bag bags/wg_cafe
SCAN_TOPIC=/base_scan BASE_FRAME=base_footprint EXTRA="--speed 4" pixi run -e ros bag bags/wg_cafe
```

Replay is paced to real time when the browser view is on; `EXTRA="--speed 4"` runs it faster.
The agent service is on during replay only with `MAP_ROOT` set and `EXTRA="--agent_port 8643"`.

The classic datasets (Intel Research Lab, ACES, MIT Killian Court) come as CARMEN logs from the
[Freiburg SLAM evaluation page](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php);
`tools/convert_carmen_log.py` turns a `.clf` into a bag. How the replay tool scores the result
against the reference trajectory is described in
[`adapters/ros2/evergreenslam_ros/README.md`](adapters/ros2/evergreenslam_ros/README.md).

### The agent interface

With the live node running, the agent service listens on `127.0.0.1:8643` and `egs` is the
client:

```bash
export PATH="$PWD/adapters/agent/egs/bin:$PATH"
egs help agent                     # the rules and the command table, under 30 lines
egs here                           # where the robot is
egs status                         # map health: frozen base, alignment, closures
egs place save places/kitchen/door # remember the spot it stands at
egs where places/kitchen/door      # pose, precision, and the pose to approach it from
egs view here                      # scan over the map, around the robot
```

`egs` is Python 3.9+ with no dependencies; `pipx install ./adapters/agent/egs` installs it on a
machine without ROS, and `egs skill install <skills dir>` gives an agent the skill. Its first
section is the onboarding (`egs root`, read `memory/README.md` with `egs fs cat`, read the
`SKILL.md` chain down to the place). For agents that take tools rather than a shell, `egs mcp`
serves the same commands as MCP tools over stdio, generated from the CLI and one to one with it.
The service binds `127.0.0.1`; an agent on another machine reaches it through an SSH tunnel
(`ssh -NT -L 127.0.0.1:8643:127.0.0.1:8643 user@robot`), with
connection settings in `~/.config/egs/config` ([`adapters/agent/egs/README.md`](adapters/agent/egs/README.md)). The HTTP
contract is [`adapters/agent/API.md`](adapters/agent/API.md), the skill is
[`adapters/agent/skills/evergreenslam/SKILL.md`](adapters/agent/skills/evergreenslam/SKILL.md).

### Layout

```
core/            the SLAM, no ROS: sensor / mapping / lifelong (pose graph, sessions, persistence) / utils
adapters/ros2/   three ament packages: msgs, the node and bag tool, the rviz2 plugin
adapters/agent/  the agent service (HTTP), the egs CLI and the agent skill
apps/webui/      the browser debug view
configs/         yaml configs; configs/reference.yaml lists every key
tools/           bag converters, the TSan gate and the demo
```

## Related work and acknowledgements

- [Cartographer](https://github.com/cartographer-project/cartographer) (Hess et al., 2016;
  Apache 2.0). The correlative scan matchers, the precomputation grids, the probability-grid
  submaps, the voxel and motion filters and the constraint builder follow its design, and the
  rviz2 submap display is ported from `cartographer_rviz`. See `NOTICE`.
  `core/src/utils/scan_matching` is the closest to the original. Cartographer has no sessions,
  freezing or marginalizing trimmer; those are this project's.
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (Macenski and Jambrecic, 2021),
  the 2D lidar SLAM supported by Nav2. The user picks its mode by node and configuration: mapping
  (synchronous or asynchronous), continued mapping of a serialized pose graph, and localization
  against one with a rolling window of recent scans; an experimental node switches between
  mapping and localization through a service call. Nodes are removed in two places, both through
  `Mapper::RemoveNodeFromGraph`, which deletes a node and its edges without marginalizing them:
  localization drops its expired scans, and a lifelong node in `src/experimental/`, described by
  its README as highly experimental, drops nearby map nodes that score low against each new scan.
  It requires an odom-to-base transform in TF: scans without one are dropped, and its
  minimum-travel gate is measured in that frame. EvergreenSLAM has no mapping/localization split,
  decides by itself when a session freezes, marginalizes what it trims, and needs no odometry
  source. slam_toolbox has a scan matcher (Karto) that its README describes as used in companies
  across the world, Nav2 integration, multi-robot mapping, map merging and interactive graph
  editing in rviz. Checked against its `ros2` branch at commit 6efdc33 (2026-10-15).
- The trimmer's marginalization and Chow-Liu sparsification follow the pose-graph compression
  literature, in the spirit of Kretzschmar and Stachniss (2012).
- [Claude Code](https://claude.com/claude-code). Most of the code was written with Claude, under
  the author's design, review and testing.
- [cpp-httplib](https://github.com/yhirose/cpp-httplib) (MIT) and
  [stb_image_write](https://github.com/nothings/stb) (public domain), vendored in `third_party/`.
- [RoboStack](https://robostack.github.io/), which puts ROS 2 inside pixi and makes the macOS
  workflow possible.
- The Freiburg and Willow Garage datasets, used for replay and scoring, and Cartographer's
  revo_lds bag, used for the demo.

## License

Apache License 2.0. See [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).
