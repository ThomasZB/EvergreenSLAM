# webui

Browser view of what the pipeline is doing. Per scan: the scan drawn at the predicted, coarse
and matched poses over the grid it was matched against, plus the track so far. Per backend
round: the pose graph itself -- sessions, optimized node poses, every constraint with the
residual it carries, submap footprints and the freeze verdict.

`core` defines the interface (`core/src/debug/debug_sink.h`) and nothing else. The transport and
the serialisation live here, next to the page that parses them.

The HTTP server is [cpp-httplib](https://github.com/yhirose/cpp-httplib) (MIT), vendored as a
single header in the repository's top-level `third_party/`, shared with the agent service. No
other dependency, and no build step for the page.

## Running it

The viewer attaches to whatever drives the pipeline. Both ROS 2 entry points take it:

```bash
# offline, paced to real time so it is watchable
evergreenslam_bag --bag <rosbag2 dir> --config configs/evergreenslam.yaml \
  --scan_topic /base_scan --base_frame base_footprint --webui_port 8642 --speed 4

# live
ros2 run evergreenslam_ros evergreenslam_node --ros-args -p webui_port:=8642
```

Then open `http://localhost:8642`. Drag to pan, wheel to zoom, `f` to follow the robot, `c` to
centre on it once. The tree on the left toggles every layer, every session and every individual
submap; alt-click solos a row, and clicking a submap's name centres on it. Expanding a session
outlines its submaps on the map, clicking a submap on the map selects it (nearest pose marker
wins where submaps overlap; `esc` clears), and the *tint submaps* layer gives every submap its own
hue so an overlap shows which two disagree.

`--webui_port` implies `--speed 1`; pass `--speed 0` after it to run unpaced. The offline tool
keeps serving after the run ends so the finished map is still there to look at.

`-DEVERGREENSLAM_WITH_WEBUI=OFF` removes it from the build entirely.

## Attaching the backend

`DebugSink` is the frontend's interface and knows nothing about a pose graph, so the two backend
views are plain methods the driver calls at its own cadence, next to each other:

```cpp
web_debug_sink->PublishGlobalMap(backend->AssembleGlobalMap());
web_debug_sink->PublishPoseGraph(*backend);   // /graph
```

`PublishPoseGraph` reads the graph on a backend task and waits for it (`Enqueue` then `Drain`),
because the graph belongs to the backend consumer thread. It blocks the calling thread, never
the backend.

**Call it from the thread that feeds the backend, after `AddInsertionResult` has returned, and
never from inside a queue task** -- a post-optimization hook or any other task is on the wrong
side of the queue, and `Drain` is not reentrant, so that is a `CHECK` failure rather than a
race. Both queue cadences are already available on the feeding side: every
keyframe, or every Nth one, the same way `PublishGlobalMap` is throttled.

## Wire format

| endpoint | payload |
| --- | --- |
| `GET /events` | SSE, one JSON object per scan: `time`, `score`, `poses`, `scan`, `map_seq`, `global_map_seq`, `graph_seq`, `traj_len` |
| `GET /map` | `EGM1` header (width, height, resolution, origin as little endian int32/float64) then one byte per cell |
| `GET /global_map` | same as `/map`, all sessions' submaps composed at their optimized poses |
| `GET /submap?s=&i=` | same again, one finished submap in its own frame; 404 once it is trimmed |
| `GET /trajectory` | float64 x, y pairs, the whole run |
| `GET /graph` | the pose graph as JSON: `sessions`, `nodes`, `submaps`, `constraints`, `freeze`, `stats`, `to_global` |

The scan is sent in the **sensor frame** with the poses alongside, so the page transforms it
itself: half the bytes, and toggling a pose layer costs nothing.

Map cells arrive **already decoded** to 0..100 occupancy and 255 unknown. The raw
`0` / `1..127` / `+128` encoding stays in `probability_values.h`, which is the only place it is
defined. The page draws unknown as fully transparent and paints the unknown colour on the page
background instead, so rasters compose rather than the upper one blanking its whole rectangle,
and so everywhere outside the map reads as what it is. Submap textures additionally fade with
confidence, which is what keeps a cell two rays grazed from covering one that was swept
properly; the whole-map rasters have no overlap to resolve and stay opaque where they are known.

`/submap` exists so the page can do what `AssembleGlobalMap` does, one texture per submap at its
`to_world` pose, and switch one off. Each finished submap is encoded once and served from a
cache the next `PublishPoseGraph` prunes to what the graph still lists -- without that pruning a
trimmed submap would keep drawing for the rest of the run. It costs roughly twice the composed
map, since submaps overlap; `WebDebugSinkOption::serve_submaps = false` drops it. The page then
draws `/global_map` instead, as it also does for the first seconds of any run: one raster, which
cannot be taken apart.

Constraints ship as four parallel flat arrays -- `type` (0 odometry, 1 loop closure, 2 prior,
3 trim summary), `ends` (world x, y of both endpoints), `session` (both endpoints', so hiding a
session hides its edges) and `residual` (metres, radians). World
endpoints rather than ids, because the page draws lines and nothing else; resolving ids there
would mean shipping a lookup table the size of the graph as well. The residual uses the same
`predicted.inverse() * target` convention as the `_constraints.csv` artifact, so the live view
and the end of run file cannot disagree about which constraint is the bad one.

## Two frames on one canvas

`to_global` is the fed session's alignment, and it is the reason it is in the payload. Everything
the `DebugSink` publishes -- the live scan, its poses, the accumulated track, **and the matching
submap's grid** -- is in the **frontend's local frame**; the pose graph, the composed map and the
submap textures are in the **global frame**. They coincide only until a map is loaded or a session
rotates. On a run with drifting odometry the two end up more than a radian apart, so the page
transforms the local layers through `to_global` before drawing. The matching submap is a raster
and not a point, so its rotation goes on the canvas transform rather than into a destination
rectangle -- it is axis aligned in the local frame, not in the world.

The whole track rides the newest alignment, so its older stretches are only roughly placed. How
far the blue track sits off the optimized nodes is the correction the backend applied.

## The page

`www/index.html` is markup and style only. Then, in load order:

| file | job |
| --- | --- |
| `view.js` | the camera, the canvas, the shared store, `sx`/`sy`, `toGlobal`, `drawGrid` |
| `raster.js` | EGM1 decode and the per submap texture cache |
| `layers.js` | what is drawn: the layer registry, session and submap visibility, the tree |
| `graph.js` | `/graph`: the backend's drawing and its stats panel |
| `app.js` | the frontend's drawing, pan and zoom, the SSE stream, the run loop |

Only `view.js` knows that world metres and screen pixels are related; everything else draws
through `sx`/`sy`. Only `layers.js` decides visibility; nothing keeps its own flag. No build step,
no dependency.

Textures are fetched two at a time, not one request per submap at once: the server's thread pool
is small and the SSE stream holds one of its threads for the whole run, so a wide fan-out parks
the rest on keep-alive and the next frame waits behind them.

## Two things to keep true

**Nothing here may change the run.** Every `DebugSink` parameter is const, the implementation
copies what it keeps, and the queue is latest-wins with a depth of one so a slow browser drops
frames instead of stalling the SLAM thread. `AttachingADebugSinkChangesNothing` asserts equality
of the recovered poses exactly, not to a tolerance. `PublishPoseGraph` is the one call that
takes a non-const reference, and only because enqueueing its own read is a mutation of the
queue; it writes nothing to the graph and costs the calling thread, not the backend.

**The matching submap is not the global map.** The sink is handed whatever the matcher matched
against, in the frontend's local frame, so the track runs off the edge of it and it has to be
aligned before it can be drawn over anything the backend produced. `/global_map` is the whole
thing composed at optimized poses; `evergreenslam_bag --out_prefix` writes the same as a PGM at
the end of a run.
