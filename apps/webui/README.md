# webui

Browser view of what the pipeline is doing, one scan at a time: the scan drawn at the predicted,
coarse and matched poses over the grid it was matched against, plus the trajectory so far.

`core` defines the interface (`core/src/debug/debug_sink.h`) and nothing else. The transport and
the serialisation live here, next to the page that parses them.

The HTTP server is [cpp-httplib](https://github.com/yhirose/cpp-httplib) (MIT), vendored as a
single header in `third_party/`. No other dependency, and no build step for the page.

## Running it

The viewer attaches to whatever drives the pipeline. Both ROS 2 entry points take it:

```bash
# offline, paced to real time so it is watchable
bag_laser_odometry --bag <rosbag2 dir> --config configs/evergreenslam.yaml \
  --scan_topic /base_scan --base_frame base_footprint --webui_port 8642 --speed 4

# live
ros2 run evergreenslam_ros laser_odometry_node --ros-args -p webui_port:=8642
```

Then open `http://localhost:8642`. Drag to pan, wheel to zoom, click the legend to toggle layers.
`--webui_port` implies `--speed 1`; pass `--speed 0` after it to run unpaced. The offline tool
keeps serving after the run ends so the finished map is still there to look at.

`-DEVERGREENSLAM_WITH_WEBUI=OFF` removes it, cpp-httplib included, from the build entirely.

## Wire format

| endpoint | payload |
| --- | --- |
| `GET /events` | SSE, one JSON object per scan: `time`, `score`, `poses`, `scan`, `map_seq`, `traj_len` |
| `GET /map` | `EGM1` header (width, height, resolution, origin as little endian int32/float64) then one byte per cell |
| `GET /trajectory` | float64 x, y pairs, the whole run |

The scan is sent in the **sensor frame** with the poses alongside, so the page transforms it
itself: half the bytes, and toggling a pose layer costs nothing.

Map cells arrive **already decoded** to 0..100 occupancy and 255 unknown. The raw
`0` / `1..127` / `+128` encoding stays in `probability_values.h`, which is the only place it is
defined.

## Two things to keep true

**Nothing here may change the run.** Every `DebugSink` parameter is const, the implementation
copies what it keeps, and the queue is latest-wins with a depth of one so a slow browser drops
frames instead of stalling the SLAM thread. `AttachingADebugSinkChangesNothing` asserts equality
of the recovered poses exactly, not to a tolerance.

**The grey area is the matching submap, not a global map.** The sink is handed whatever the
matcher matched against, so the trajectory runs off the edge of it. A global map arrives with
the pose graph; `bag_laser_odometry --out_prefix` writes a whole-run PGM in the meantime.
