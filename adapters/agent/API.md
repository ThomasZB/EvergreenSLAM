# AgentService HTTP contract (v0.3)

`adapters/agent/service/`. An in-process `httplib::Server` owned by the host (bag main, live node)
beside, never inside, the webui. Binds host flag `--agent_bind` (default **127.0.0.1**), port 8643;
compose passes `0.0.0.0` inside the container and maps `127.0.0.1:8643:8643` on the host side. No
auth: loopback and one agent session per map. `egs` is the only intended client; an agent on
another computer reaches it through an SSH tunnel to the robot's loopback (`egs/README.md`).

## Conventions

- **Requests are flat parameters**: query string for GET, form-urlencoded body for POST (shown as
  JSON below). Exception: `fs write|append` take `path` in the query and raw bytes as the body. No
  JSON parser in the service; responses come from a small writer.
  Booleans accept `1|true` and `0|false`; an `fs` `path` of `.` is `memory/` itself.
- **Responses** are `application/json` except `/view` (PNG), `/fs/cat` and `/snapshots/file` (raw
  bytes), `/fs/tree` (text).
- **Receipt**: every JSON response carries `ok` (bool) and `reason` (code string, `null` when ok);
  every endpoint marked *task* also carries `at_num_solves` = `optimization().num_solves()` read in
  that task, so values from one response share one solve. Endpoint fields are added beside these.
- **Poses** are `{"x", "y", "theta"}` in the global frame (metres, radians CCW from +x). A submap id
  is `[session, index]`. Times are int64 Unix ns (`*_ns`); ages are seconds on the sensor clock.
- **Node paths** are relative to `memory/` and start with `places/`, exactly what `find places …`
  prints from `memory/`. **Directory** components match `[a-z0-9][a-z0-9_-]*`; `skills` and
  `attachments` are never nodes (`reserved_name`); file names are free except `/` and NUL.
- **Anchor state** as agents see it (every `state` field, `places.json`, `index.tsv`), from the
  stored `AnchorState` and `ResolvedAnchor::frozen`: `pending` (BOUND, not frozen), `frozen`
  (BOUND or REBOUND, frozen), `rebound` (REBOUND, not frozen), `orphan`.

### Task discipline (every endpoint marked *task*)

- Exactly one `PoseGraph::Enqueue` + promise/future per request; everything the answer needs is read
  in that task. The service waits without a timeout; `egs` gives up after 30 s ("outcome unknown").
  One exception: `rm/apply` of the fed session runs its plan task, then the host's drop (one more
  task, waited on from the serial executor), then a read-only task for `at_num_solves`.
- Inside a task: only the free `lifelong::AssembleGlobalMap(graph, 0.0, /*only_finished=*/true[,
  session])`, never the blocking member (it would wait on its own queue); unfinished submaps are
  being written by the frontend thread.
- Preconditions are checked in the task right before acting; a refusal changes nothing.
  `SetInitialPose`, `RelocalizeGlobally`, `FreezeFedSession` only enqueue, so they are called from
  the task after the checks. `RemoveSession` and `graph()` reads happen only inside a task.
- Never reachable from here: `PoseGraph::FreezeSession(id)`, bare `StartNewSession`,
  `mutable_graph()`, `mutable_session_manager()`.

### Error model

| HTTP | when | body |
|---|---|---|
| 200 | every well-formed request, including refusals | receipt with `ok: false` and a `reason` |
| 400 | missing / malformed parameter, path rule broken | `{ok: false, reason, detail}` |
| 404 | unknown route (`not_found`); `GET /fs/cat` of a missing path; `GET /snapshots/file` of anything it does not serve | same, `reason: "not_found"` |
| 503 | task endpoint before `OnBackendStarted()` (an atomic), or after `Stop()` began | same, `reason: "not_started"` |
| 503 | four requests already wait on the serial executor (Host injection, "Threads") | same, `reason: "busy"`: nothing was done, retry |

Refusal codes: `unknown_anchor`, `unresolvable` (orphan anchor), `no_keyframe`, `node_gone`,
`too_old`, `in_future`, `gap` (`/anchors/{id}?at=`),
`not_persisted` (the commit did not reach disk; nothing was saved), `offset_too_far`,
`offset_not_free`,
`fs_error`, `no_binding`, `no_robot_pose`, `unknown_session`, `freezing`, `frozen_session`,
`plan_changed`, `exists`, `not_empty`, `unknown_map`, `not_supported` (the host has no map root or
cannot switch maps or drop the fed session), `switching` (a map switch this service accepted has
not landed yet; freeze plan/apply, a fed-session rm and `place/save` refuse it too),
`session_changed` (a fed-session drop found another session fed: a freeze rotated it between plan
and apply), plus freeze rejections verbatim as `ToString(FreezeRejection)`
returns them (`not anchored`, `no finished submap`, …; no mapping, `egs` matches these). 400:
`bad_param`, `path_escape`, `symlink`, `not_slug`, `reserved_name`, `owned_by_process`,
`too_many_layers`, `unknown_layer`, `too_large`.

## Host injection

```cpp
struct StampedPose {
  common::Time time;
  Eigen::Affine2d local_pose = Eigen::Affine2d::Identity();  // frontend local frame
};
struct HostFrame {
  StampedPose pose;          // MatchingResult::time, ::local_pose
  sensor::PointCloud scan;   // robot frame at that pose
  double match_score = 0.0;      // MatchingResult::match_score of that scan
  double match_score_avg = 0.0;  // MatchScoreAverage over the host's scans
};
struct HostPose {                 // the newest sample, no scan copy
  StampedPose pose;               // MatchingResult::time, ::local_pose
  Eigen::Vector3d velocity;       // [vx, vy, omega], robot frame (MatchingResult::velocity rotated)
  uint64_t seq;                   // +1 per sample, never reset by a drop
  uint64_t epoch;                 // +1 per fed-session drop
  std::chrono::steady_clock::time_point received;  // host steady clock at Update
};
struct PoseAt {                   // the local pose at a past time
  enum class Refusal { NONE, NO_SAMPLE, TOO_OLD, IN_FUTURE, GAP };
  std::optional<Eigen::Affine2d> local_pose;  // set exactly when NONE
  Refusal refusal;
};
struct MapSwitchRequest { std::string name; bool create = false; };
struct SessionDropRequest { lifelong::SessionId id; };             // always the fed session
struct AgentServiceHooks {
  std::function<std::optional<HostFrame>()> current_frame;         // pose and scan under one lock
  std::function<std::optional<HostPose>()> current_pose;           // PoseHistory::Latest
  std::function<PoseAt(common::Time)> pose_at;                     // PoseHistory::At
  std::function<bool(const MapSwitchRequest&)> switch_map;         // empty: cannot switch
  // empty: cannot drop the fed one
  std::function<PoseGraph::DropFedSessionResult(const SessionDropRequest&)> drop_session;
};
struct AgentServiceOption {
  std::string bind = "127.0.0.1";
  int port = 0;              // 0 binds an ephemeral port
  std::string map_root;      // empty: no root known, /maps* answer not_supported
  std::string map_name;
};
AgentService(PoseGraph& pose_graph, AgentServiceHooks hooks, const AgentServiceOption& option);
void OnBackendStarted();  // after PoseGraph::Start
void Stop();              // stops the server and joins its threads
```

- **Lifecycle**: constructed at host init, only when `pose_graph.map_manager()` is non-null (else
  the host logs once and runs without it); `map_dir` = `map_manager()->directory()`. Construction
  creates `memory/places/`, installs `memory/README.md` (`memory_README.md` compiled in; rewritten
  each start) and listens; task endpoints answer 503 until the host calls `OnBackendStarted()`
  after `PoseGraph::Start`. The host calls `Stop()` **before** `PoseGraph::Finish()`. The host may
  rebuild the service after `switch_map`: the port closes and reopens.
- **Threads**: `new_task_queue` is a `ThreadPool(8)`; every connection serves one request
  (`set_keep_alive_max_count(1)`), so an idle client never pins a pool thread. Every route runs its
  handler through one `SerialExecutor` (`service/http/serial_executor.h`, registered through
  `service/http/routes.h`): a FIFO ticket lock taken on the pool thread itself, so the service-side
  state (`here` hysteresis, `closures_delta` baseline, sequence numbers, the `switch_map` and
  `drop_session` calls) stays as single-threaded as with one worker. The line is bounded: with
  one handler running and **four waiting**, a request is answered `503 busy` at once, so serial
  handlers occupy at most five pool threads and never starve the concurrent routes. Only `/pose`
  and `/root` (const fields) are concurrent (`Routes::ConcurrentGet`). `Stop()` closes the
  listener, then the pool joins after its queued requests: waiting handlers still get their turn
  (task routes answer `not_started` once stopping), and backend tasks drain since the host calls
  `Stop()` before `PoseGraph::Finish()`, so no handler blocks forever.
- **Two hook kinds.** `switch_map` runs on the serial executor, never inside a task: it records the
  request and returns at once (`false` = refused, reported as `not_supported`). The host then, on
  its own thread, stops this service, finishes the graph, and boots the named map with a new
  frontend, `PoseGraph`, graph publisher and service, and clears the webui's graph, submap
  textures and trajectory (open pages reload). The laser extrinsic and tf state carry over.
  `drop_session` is the other kind: it runs on the serial executor outside any task and answers
  synchronously. The host blocks its frontend (no scan reaches the builder or the graph), calls
  `PoseGraph::DropFedSession(id)`, and on success replaces the `LocalTrajectoryBuilder` with a
  fresh one, resets its pose history (`AgentHost::OnFedSessionDropped`: epoch + 1) and clears the
  webui before unblocking; it returns the `DropFedSessionResult`. It is the only way the fed
  session's local frame changes (the freeze rotation keeps the frontend's frame). The
  process, the port and this service stay. `DropFedSession` runs one backend task: refusals
  `NOT_FED` (another session is fed) and `FREEZING` (a freeze sequence is queued) change nothing;
  otherwise a fresh fed session is minted (id = dropped + 1, identity alignment, no submaps, not
  anchored; committed first, so a kill before the second commit keeps the dropped session on
  disk), then `RemoveSession(dropped)` (graph, Problem, anchors → `ORPHAN(SESSION_REMOVED)`, one
  commit), the constraint builder's caches are swept and the ingest bookkeeping restarts at node
  index 0. The robot's pose is unknown until `init-pose` or `relocalize`: `ActiveSessionToGlobal()`
  is empty until the new frontend's first keyframe, and identity after it. `last_pose.pb` still
  names a node of the dropped session until that keyframe overwrites it; a boot ignores a last pose
  whose session is gone. After a drop the next boot seeds from the newest idle session with
  nodes, exactly as any boot does.
- **`current_frame`** runs **inside the task** and takes only the host's own mutex;
  `current_pose` and `pose_at` likewise, from a task or straight from a pool thread. Both ways:
  these and `switch_map` never enqueue or wait on the `PoseGraph` (`drop_session` waits, from the
  serial executor outside any task), and the host never holds its pose/scan mutex across any
  `PoseGraph` call (it updates by copy under the lock). Empty hook or nullopt → the fields are `null`.
  Pose and scan come from one call: two calls could straddle a scan-thread update and draw scan
  k+1 at pose k.
- **Frame update**: pose, scan and `match_score` come from one non-null `AddScan` result; a scan the
  builder drops (nullptr) leaves the previous frame untouched, so a new scan is never paired with an
  old pose. `match_score_avg` comes from `MatchScoreAverage`
  (`service/graph/match_score_average.h`), sampled per scan under its own mutex, not per request:
  `avg += (1 - exp(-dt / 2 s)) * (score - avg)`, seeded by the first scan.
- Robot pose = `ComputeSessionToGlobal(fed)` (nullopt → `graph.session(fed).local_to_global`, as
  `pose_graph.cc` does) `* local_pose`, in the same task, never via `ActiveSessionToGlobal()`;
  only `/pose`, which has no task, uses `ActiveSessionToGlobal()` (below).
- **Pose history** (`service/graph/pose_history.h`, under the host's mutex): every `Update` adds
  `{time, local_pose, velocity}`; samples older than the newest − 10 s go, and a stamp not after
  the newest restarts the history. `pose_at(t)`: no sample → `NO_SAMPLE`; `t` before the oldest →
  `TOO_OLD`; up to 0.2 s past the newest → the newest, beyond → `IN_FUTURE`; on a sample → that
  sample; else the two bracketing samples, `GAP` when they are more than 0.5 s apart, otherwise
  `utils::transform::Interpolate` (linear translation, shortest-arc angle). `OnFedSessionDropped`
  clears it and advances `epoch`; `seq` keeps counting.
- **Epoch pairing for `/pose`.** A host sample is in one frontend's local frame and
  `ActiveSessionToGlobal()` in one fed session's; a drop changes both, in two steps (the backend
  republishes the new session's alignment, then the host resets its history) with a window in
  between. The service wraps `drop_session` in a seqlock (`ServiceContext::drop_seqlock`, seq_cst,
  odd while the hook runs); `/pose` reads it, then `current_pose`, then `ActiveSessionToGlobal()`,
  then the seqlock again, and composes only when both reads are the same even value, else
  `pose: null`. Race-free: the republish and the reset both happen after the opening increment and
  before the closing one, and both reach `/pose` through a mutex it then takes, so a `/pose` that
  saw either change sees at least the opening increment in its second read, and one whose first
  read saw the closing increment sees both changes. Every drop goes through `drop_session` on the
  serial executor, so `/anchors/{id}?at=`, also serial, never overlaps one: the `pose_at` samples
  and the fed session its task reads are always one frontend's.

## Endpoints

`here`, `pose`, `status`, `where`, `snapshot`, `view`, `zones`, `session ls` read; everything
else mutates.

**GET /root** — no task. `→ {ok, api_version, instance_id, map_dir, memory_dir, map_root, map}`
(absolute paths; `memory_dir` is the realpath); `map_root` and `map` are `null` when the host named
no map root. `api_version` is `"0.3"`. `instance_id` is 32 random hex digits drawn when the service
is constructed (so per process and per opened map); the service writes it, newline-terminated,
atomically to `<map_dir>/agent_instance` (a write failure is only logged). `egs` works on files
directly only when `EGS_MEMORY` / config `memory` names a directory, or when
`<dirname(memory_dir)>/agent_instance` exists on its own machine and holds this `instance_id`;
everything else goes through `/fs` and `/snapshots/file`, so a same-named directory on the agent's
computer is never read or written.

**GET /pose** — no task, concurrent (never `busy`; answers before `OnBackendStarted()` too).
For programs polling at ~10 Hz (`egs pose --watch`); `/here` scans `memory/` and queues behind
loop closure. `→ {ok, seq|null, stamp_ns|null, age_s|null, pose|null, velocity: {vx, vy,
omega}|null}`: `seq` the host's sample counter (a new value = a new scan; restarts with the host
on a map switch), `stamp_ns` the scan's sensor time, `age_s` the robot-side steady-clock age of
the sample (it grows while the scan stream is stopped), `velocity` in the robot frame (m/s,
rad/s) whenever there is a sample. `pose` = `ActiveSessionToGlobal() * local_pose` under the
epoch pairing above; `null` before the first scan, before the fed session's first keyframe (also
right after a drop), or while a drop runs. No `linked_to_base`: it needs a graph read, hence a
task; ask `/status`. `ActiveSessionToGlobal()` is republished at solves and graph changes, so it
can trail the task-side alignment `/here` uses by one batch.

**GET /status** — task. `→ {ok, at_num_solves, boot_count, fed_session, map (name|null), has_frozen_base,
aligned_to_base, phase, localization, relocalization, closures_delta, num_anchors, num_orphans,
num_place_files, duplicate_anchor_paths: [path]}`. `localization` as in `/here`; `relocalization`
is the latest request (see *Relocalization outcome*). `egs status` prints neither `phase` nor the
word "aligned": its text says `linked to base yes|no` for `aligned_to_base` (JSON unchanged). `has_frozen_base` = any `graph.sessions()` entry frozen;
`aligned_to_base` = `HasFrozenLink(graph, fed)`; `phase` from `expansion().find(fed)`, absent →
`BOOTSTRAP`; `closures_delta` = `constraint_builder().num_constraints_added()` since the previous
`/status`. Two `place.yaml` naming one anchor (a `cp -r`) are listed and ignored by `here`.

**GET /here** — task. `{relocalization?: id}`. The service scans `memory/places/**/place.yaml`
(skipping `skills/` and symlinks), then one task reads `current_frame` and `ResolveAll`.
`→ {ok, at_num_solves, robot: pose|null, keyframe_age_s|null, has_frozen_base, aligned_to_base,
localization: {since_base_match_m|null, base: {score|null, coverage}|null, scan_match: {score,
avg}|null}, relocalization, current: {path, anchor, dist_m}|null, closest: [{path, anchor,
dist_m, state}]}` — `relocalization` is the request named by the parameter, or the latest without
it, `null` when there is none or the id is no longer tracked (malformed id → 400). `closest` is the
top 2 non-orphan referenced anchors by straight-line distance. `current` = nearest within 2.5 m;
once current, a place stays current until 3.75 m. `keyframe_age_s` = robot stamp − newest node time.
`has_frozen_base` / `aligned_to_base` as in `/status`, read in the same task: with a frozen base but
no link to it, the robot pose is in the fed session's own frame and `current` / `closest` compare it
with frozen anchors in another frame (`egs here` says so on its first line, `linked to base: no`).

`localization` (replaces the former top-level `scan_match`) carries three signals,
read in the same task as `robot`:
- `scan_match` comes from the same `current_frame` call as `robot` (`null` with it): `score` is the
  frontend's match score of the newest scan, the mean occupancy probability of its points in the
  active local submap (0..1), and `avg` its ~2 s average. It measures agreement with the **local**
  map only, so it can stay high while the global pose is wrong. Measured on the `museum_b2` replay
  (333 `egs here` reads): `score` 0.58–0.79 (5th–95th percentile, median 0.72); `avg` starts low for
  a few seconds, since the first scans have no map to match (score 0).
- `base` (`null` while the fed session has no node): the fed session's newest node — its
  `constant_data.point_cloud` (the adaptively filtered cloud loop closure matches) at its
  `global_pose` — looked up in one grid of every frozen session's finished submaps
  (`SessionManager::base_map()`, a `FrozenBaseMap` rebuilt in full at load and after each freeze).
  It does not use the host frame: it lags the robot by up to one keyframe (`keyframe_age_s`).
  `coverage` = fraction of the node's points on known cells; `score` = mean occupancy probability
  over those points only (0..1), `null` when no point lands on a known cell (no frozen base, or the
  node outside the frozen map). It is consistency with the frozen base, not proof the pose is
  right: furniture moved since the freeze and people in view lower it, and a pose that slides along
  a corridor can keep it high. `coverage` says how much of the cloud could be checked at all; a low
  `coverage` makes `score` say little. No time average. Measured 2026-10-16 (long_update cross-day, museum_b2, scored
  per keyframe): correct poses at `coverage` ≥ 0.5 score 0.41–0.79 (5th–95th percentile, median
  0.62–0.72); the same nodes moved 0.25 m or turned 5° or more score 0.20–0.45 with `coverage`
  barely lower. A 20 s median below 0.45 never occurred on a correct pose. `egs here --sample S`
  takes that median client side; the service keeps none. It polls `/events` alongside and drops
  the reads taken before a `relocalization` (matched) or `pose_correction` event (`restarted`).
- `since_base_match_m`: keyframe-polyline distance the fed session has driven since its last
  accepted loop closure to a frozen session (`DriftTracker::SinceBaseMatch`; closures the
  constraint builder added, not trim-recovered edges or priors; a match arriving late never moves
  it back). `null` before the first such closure: no frozen base, a fresh boot, a fed session just
  dropped. A relocalization request by itself resets nothing. The freeze rotation hands it on: the
  successor continues the predecessor's distance.

`aligned_to_base` remains the structural global signal (a frozen link exists at all); `base` and
`since_base_match_m` say how well and how recently the current pose agrees with that base.

**POST /place/save** — task, mutating. `{path, scan: true, offset?}`. `offset=dx,dy,dtheta`
(m, rad; robot frame, x forward, y left, as `egs observe --offset`) binds the place to that pose
instead of the robot's (the host's current pose, composed onto the last keyframe; dtheta wrapped to
(-π, π], echoed wrapped); malformed → 400 `bad_param`. Path rules first (400), then the
service reads any existing `<path>/place.yaml`. Task: `last_ingested_node()` empty → `no_keyframe`;
`hypot(dx, dy) > 3` → `offset_too_far` (the service's policy; a well-formed request, so 200);
an id from that file that `anchors().Get` knows is **rebound in place** (`SaveAnchorOnTask(scan,
id)`: `place.yaml` untouched, children's `offset_from` stays valid), else `SaveAnchorOnTask(scan)`
mints one. A path whose id another `place.yaml` also names (a `cp -r`, listed by `/status`) always
mints: rebinding would move the other place too, and a fresh id ends the duplicate while the
other path keeps its id. `SaveAnchorResult::Refusal` maps to `no_keyframe`, `node_gone`,
`offset_not_free` (the target cell is not known free space, probability < 0.5, in the submap the
anchor binds to; unknown counts as not free) or `not_persisted` (the commit failed: the table is rolled back, so a rebound place keeps its old
binding, and no id is returned and no `place.yaml` written). For a new id the service thread then does
`mkdir -p <path>` and writes `place.yaml` = `anchor: <id>\n` atomically; failure → `fs_error`, `anchor` still returned (a
kill there too leaves an unreferenced anchor, never a dangling file). `→ {ok, reason,
at_num_solves, anchor, submap_id, state, keyframe_age_s, rebound_existing: bool, offset: [dx, dy,
dtheta]|null}`.

**GET /anchors/{id}[?robot=1][&at=<ns>]** — task; `egs where` (walk-up, `offset_from`,
`precision`, `approach` are the CLI's job). `anchors().Resolve(graph, id)` `→ {ok, reason
(unknown_anchor), at_num_solves, anchor, state, orphan_reason|null, pose|null, submap_id,
saved_at_ns, robot?: pose|null, has_frozen_base?, aligned_to_base?, at_ns?}`; `robot=1` adds the
robot pose and the two alignment flags (as in `/status`) from the same solve (`egs observe
--offset`, `egs zone add`). `at=<Unix ns>` (implies `robot=1`; malformed → 400 `bad_param`)
makes `robot` the pose at that sensor time: `pose_at(at)` times the fed session's current
alignment, composed as above, in the same task as the resolve (`egs observe --offset --at`).
Refusals (`ok: false`, anchor fields still there, `robot: null`): `too_old`, `in_future`, `gap`
as in "Pose history", `no_robot_pose` (no sample, no fed session, or no `pose_at` hook);
`unknown_anchor` wins over them. `at` is the robot's sensor clock (scan stamps), not the agent
computer's: `/pose` `stamp_ns` is the robot's time now. `egs where` and `egs session ls` print
`saved_at_ns`, `start_ns` and `last_node_ns` as ages against `stamp_ns + age_s` of one `/pose` read
(this computer's clock only while `/pose` has no sample).

**GET /anchors[?robot=1]** — task. `→ {ok, at_num_solves, robot?, anchors: [{anchor, state,
orphan_reason, pose|null}]}`. **find** and **observe** are CLI only, on these two endpoints.

**GET /zones** — task (read). The service scans `memory/places/**/zone.yaml` (skipping `skills/`
and symlinks) and walks each up from its parent to the nearest `place.yaml` (a `place.yaml` in the
zone's own directory is `holds_place`); one task resolves those anchors and applies each
`global_pose` to every vertex. A walk that cannot read part of the tree (EACCES, a directory moved
mid-walk) refuses `fs_error` with a `detail` instead of returning a shorter list.
`→ {ok, at_num_solves, zones: [{path, kind, frame|null, frame_path|null, anchor|null,
state|null, polygon, polygon_xy|null, reason|null}]}`, sorted by path. `frame` is the file's,
`frame_path` the walk-up's; `polygon` is the anchor frame, `polygon_xy` the map frame. `reason`:
`frame_mismatch` (the two differ; `polygon_xy` still resolves through `frame_path`), else why
`polygon_xy` is null: `orphan`, `unknown_anchor` (`state: orphan`), `unknown_kind` (only
`keepout` is used), `no_place`, `holds_place` (the directory also has `place.yaml`),
`unreadable_place`, `bad_zone_file` (not `kind` + `polygon` of ≥ 3 `[x, y]`), `too_large` (over
100 m wide or tall in the anchor frame; `egs zone add` refuses the same). `egs zone add|rm`
write through the tree (or `/fs`), not through the service.

**POST /snapshot** — task, then writes. One task copies out the assembled map, every node's `{time,
global_pose, session}`, sessions, `ResolveAll`, robot pose, `boot_count`; the service thread writes
`<map_dir>/snapshots/NNNNNN/` (`.tmp`, renamed) and regenerates `memory/index.tsv` atomically.
`→ {ok, at_num_solves, seq, dir, files: [name]}`. `egs snapshot -o` copies client side in local
mode and downloads each file through `/snapshots/file` otherwise.

**GET /snapshots/file** `{seq, name}` — no task, read only. Raw bytes of
`<map_dir>/snapshots/NNNNNN/<name>` (`NNNNNN` = `seq`, 6 digits). `name` must be one of the file
names below and the file a regular file (no symlink) inside that directory; anything else,
including a name not written for that snapshot (`map.*` without cells), answers 404
`not_found`. `seq` outside 1..999999 or missing parameters: 400 `bad_param`.

| file | content |
|---|---|
| `map.pgm` + `map.yaml` | nav2 trinary: known and p > 0.5 → 0, known → 254, unknown → 205; yaml has map_server keys only |
| `map.png` | same classes, long side ≤ 1024 px |
| `map.json` | `boot_count, num_solves, generated_at_ns, newest_node_ns, fed_session, resolution r, origin [ox, oy], width, height h`; origin = lower-left corner of cell (0,0); `x = ox + (col + 0.5) * r`, `row = h - 1 - floor((y - oy) / r)` |
| `trajectory.csv` | `stamp_ns,x,y,theta,session`, ≥ 1 m arc-length decimation, first and last kept |
| `places.json` | `[{path, anchor, state, orphan_reason, pose|null}]`, places only (objects are `egs find`'s) |
| `summary.txt` | bounds, unknown fraction, sessions, place table with straight-line distance from the robot |
| `memory/index.tsv` | `#path anchor state x y theta num_solves`, one row per referenced `place.yaml` |

**GET /view** — task (read). Params: `preset` = `map|here|route|trail|session|custom`;
`target=<path>` (route; must hold its own `place.yaml`); `session=<id>` (session; default fed);
`layers=a,b,…` (custom, at most 4); `ego=<r>` (optional crop radius in m; `egs --ego` and `egs view here` send 7.5, `--full` none).

| preset | layers |
|---|---|
| `map` | map, places, zones |
| `here` | map, robot, scan, zones |
| `route` | map, robot, target, places, zones (no path line: routing is the navigator's) |
| `trail` | map, trail, robot |
| `session` | session=<id>, trail (that session's nodes) |

Layers: `map` (trinary grid), `robot`, `scan` (the `current_frame` scan at that frame's robot pose), `trail` (nodes of
sessions from `PoseGraph::boot_first_session()` on), `places` (numbered badges with leader lines),
`target=<path>` (one highlighted `T` badge; not numbered again under `places`), `session=<id>` (`AssembleGlobalMap(graph, 0.0, true, id)`), `submaps` (outlines),
`zones` (hatched keep-out polygons as `/zones` resolves them, no label; the legend adds `zones: 2
keepout[, 1 unresolvable][, incomplete (...)]`, the last when the zone walk was partial).
One task reads everything drawn; the service thread renders (5×7 bitmap font, `stb_image_write`):
north-up, grid 1/2/5 m labelled every 5 m, scale bar, long side ≤ 1024 px (a smaller view is
upscaled to `clamp(floor(1024 / cells), 1, 4)` px per cell; glyphs, text and grid keep their pixel
sizes; a badge never covers the robot glyph), also saved as
`<map_dir>/views/NNNNNN_<preset>.png`. `→ image/png` with headers `X-EGS-Layers` (the equivalent
`custom --layers`; a preset's list may exceed custom's 4, e.g. `route`), `X-EGS-Legend` (`1 places/dock 3.2m; 2 places/kitchen 5.1m`), `X-EGS-View`
(relative to `map_dir`, e.g. `views/000005_map.png`),
`X-EGS-Solve`. JSON refusals: `no_binding`, `unresolvable` (target), `no_robot_pose` (`ego` only;
otherwise robot and scan layers drop, noted in the legend), 400 `too_many_layers`, `unknown_layer`.

**POST /init-pose** — task, mutating. `{x, y, theta}` or `{anchor}` (`egs init-pose --place` reads
`<path>/place.yaml`, no walk-up). Resolve; orphan → `unresolvable` (no old-XY fallback); then
`SetInitialPose(pose)`. `→ {ok, reason, at_num_solves, request_id|null, pose}` (`request_id`
null on a refusal). The service remembers `kind` `place` (with `anchor`) or `pose` for the id.

**POST /relocalize** — task, mutating. `RelocalizeGlobally()`. `→ {ok, reason, at_num_solves,
request_id}`, kind `global`.

### Relocalization outcome

`SetInitialPose` / `RelocalizeGlobally` mint the id on the calling thread (counter from 1 per
process) and return it; the search runs later on the backend task. The object in `/here` and
`/status`:
`{id, kind: pose|place|global, anchor|null, state: pending|matched|no_match|superseded, candidates,
constraints, correction_m|null, correction_rad|null, requested_ns|null, resolved_ns|null,
superseded_by|null}`.
- The request's task supersedes every older request still pending (`superseded_by` = its id). A
  superseded round's matches still land as ordinary constraints but no longer count.
- With no node in the fed session yet the request stays `pending` (`requested_ns` null) until the
  first keyframe runs it. Otherwise one candidate round runs against the fed session's newest node
  (`requested_ns` = that node's sensor time): every finished submap of another session the prior
  reaches within the relocalization window, or all of them without a prior. `candidates` = pairs
  selected after sampling. 0 candidates → `no_match` at once (the prior is outside the saved map).
  Each match, inline or from a worker, reports on the backend task whether
  `AddConstraintIfEndpointsLive` accepted it; when all have reported: ≥ 1 accepted → `matched`,
  else `no_match`.
- `matched`: unless a solve already ran after the last accepted constraint (the first-anchoring
  reseed solves), one `OptimizeOnTask()`, then `RepublishActiveSessionToGlobal()`. `correction_m` /
  `correction_rad` = how far the newest node at request time moved (its local pose under the
  session-to-global before the round vs after the solve). `matched` means applied and republished,
  not that the pose is right: on a session already linked to the base the round adds only a few
  Huber-lossed edges, which a session pinned by more wrong closures outvotes (core test
  `AHintOnASessionPinnedBySixWrongClosuresMatchesButLeavesItWrong`); the recovery is `session rm`
  of the fed session, then `init-pose`.
- A fed-session drop marks the open request `superseded` with `superseded_by` null.
- Times are sensor time (`resolved_ns` = newest node time across sessions at settlement).
- Core reads `relocalization(id)` / `latest_relocalization()` are thread safe (own mutex; the last 16
  requests); the service reads them inside its task so they match `robot`.

**GET /events** — no task (the log has its own mutex), 503 before start like task routes.
`{since: seq = 0}` (negative → 400). `→ {ok, reason, instance_id, first_seq, next_seq, lost, events: [{seq,
time_ns, kind, ...}]}`: the `PoseGraph` event ring (256, `EventsSince`), events with `seq ≥ since`.
`lost` = `since < first_seq` (overrun) or `since > next_seq` (a restarted process); then every held
event is returned; a changed `instance_id` (a map switch or restart) also means start over from 0. `time_ns` is sensor time. Kinds and fields:
- `relocalization`: `request_id, state, candidates, constraints, correction_m, correction_rad,
  superseded_by` — on every settlement, `superseded` included.
- `pose_correction`: `session, correction_m, correction_rad` — a `RepublishActiveSessionToGlobal()`
  moved the fed session's newest node by more than `lifelong.pose_correction_event_translation`
  (0.3 m) or `pose_correction_event_rotation` (0.2 rad), comparing the previous and new transform of
  the same fed session (a rotation or drop to a new fed session never logs one).
- `session_frozen`, `session_removed`: `session`. A drop logs `session_removed` then
  `fed_session_dropped` (`session, fed_now`), then `relocalization` for a superseded request.

egs: `init-pose` / `relocalize` poll `/here?relocalization=<id>` every 0.5 s up to `--wait S`
(default 30; `--no-wait` returns at once). Printed outcome: `matched: corrected D m A°; check egs
here` (exit 0), `no_match (N candidates): ...` with the next step, `superseded by request M`
(exit 1 for both); still pending at the deadline, or the id unknown to the process → `outcome
unknown` (exit 3). `--json` adds the status object as `relocalization` (`ok` false and `reason` the
state unless matched). `egs here` / `egs status` print a `relocalization ...` line when there is a
request. `egs events [--since N] [--follow] [--json]`: one line per event and `next_seq N`;
`--follow` polls every second, one line (with `--json`, one JSON object) per event; `events lost`
goes to stderr.

**POST /checkpoint** — task, mutating. `CheckpointOnTask()` (one commit: all unfrozen session files, the
anchors file if changed, the manifest); a failed commit → `not_persisted`. `→ {ok, reason,
at_num_solves, num_checkpoints_written}`.

### Sessions

**GET /sessions** — task. `→ {ok, at_num_solves, fed, sessions: [{id, role: fed|idle|frozen,
nodes, submaps, anchors, phase|null, anchored|null, start_ns, last_node_ns}]}`; `anchors` = live
anchors bound there; `phase` / `anchored` (`HasFrozenLink`) as in `/status`, `null` when frozen.
`egs session ls` needs the running process: no offline listing in v0.1 (zero-dependency CLI).

Destructive operations are plan → apply. A plan returns a **Report**:

```
{ok, rejection|null, would_delete: [[s,i]…], sessions_affected: [id…], anchors_orphaned: [id…],
 at_num_solves, plan_token}
```

`plan_token` = hex FNV-1a 64 of `op|sorted sessions_affected|sorted would_delete|sorted
anchors_orphaned`. Apply re-runs the plan in one task; a rejection or another token → `plan_changed`,
nothing done. `num_solves` is not the lock: solves move poses, not which ids an operation touches.

**POST /sessions/freeze/plan** `{force: false}` — task, **not read-only**: `OptimizeOnTask()` and
`RepublishActiveSessionToGlobal()` first (a manual freeze is rare, a fresh solve is worth it; bumps
`num_solves`; map->odom follows it even when no keyframe comes), then `Judge(graph(),
mutable_optimization(), CovarianceEvaluator(mutable_optimization()), fed)` via
`session_manager().freeze_judge()`. Not eligible → `rejection = ToString(verdict.rejection)`;
`force` (humans only, never in `SKILL.md`) waives `not anchored` only. `sessions_affected = [fed]`,
`would_delete = []` (the freeze-time trim is not predicted). Adds `verdict: {eligible, bootstrap,
rejection}`.
The freeze-time trim drops every finished submap the frozen map already covers; when that takes all
of them the session is removed instead of frozen (`SessionManager::num_sessions_removed()`), its
unfinished submaps and the nodes only they hold handed to the successor as usual. The successor is
fed either way.

**POST /sessions/freeze/apply** `{force, plan_token}` — task, mutating. Re-plan (solving again),
then `FreezeFedSession(planned fed)`: the queued freeze task logs and does nothing if the fed
session changed meanwhile. The receipt reports intent: `→ {ok, reason, at_num_solves,
frozen_session, pending: true}`: it cannot tell a freeze from a removal, the trim has not run
yet. `egs` polls `GET /sessions` (≤ 10 s) until the session is `frozen`, or gone with another session
fed: removed, exit 0 too.
`egs session new` is a CLI alias of freeze: a fed freeze always rotates to a fresh fed session.

**POST /sessions/rm/plan** `{id}` — task, read-only. Rejections: `unknown_session`, `freezing`
(a freeze sequence is queued; its tasks assume every session they name outlives them: retry),
`frozen_session`, and for the fed session `not_supported` (no `drop_session` hook) or `switching`.
`would_delete` = the session's submaps, `sessions_affected = [id]`, `anchors_orphaned` = live
anchors bound there. `drops_fed` is true exactly for the fed session, with `note: "this is the
session being mapped now: it is replaced by a fresh one; the robot's pose is lost"`.

**POST /sessions/rm/apply** `{id, plan_token}` — task, mutating. Re-plan, then `RemoveSession(id)`
(graph, Problem, `OnSessionRemoved`, then one `MapManager` commit of the files and anchors), then
`RepublishActiveSessionToGlobal()`.
`→ {ok, reason, at_num_solves, removed, fed_session, anchors_orphaned, pose_lost: false}`. For the
fed session (`drops_fed`) the task changes nothing; after it, on the worker thread,
`drop_session({id})`: `NOT_FED` → `session_changed`, `FREEZING` → `freezing`, else `{ok,
at_num_solves, removed, fed_session: <the new one>, anchors_orphaned: <the plan's>, pose_lost:
true}`, `at_num_solves` read after the drop. Synchronous: nothing is pending and nothing is refused
as `switching` afterwards. A refusal answers `{ok: false, reason, at_num_solves, removed: null,
anchors_orphaned: []}`.

### Maps (no task)

On disk: `<map_root>/<name>/` is one map directory (manifest, session files, anchors,
`last_pose.pb`, `memory/`, `views/`, `snapshots/`); `<map_root>/current` holds the name the host
boots when it is given none (else `default`). Names match `[a-z0-9][a-z0-9_-]*`. Nothing here
deletes a map.

**GET /maps** `→ {ok, map_root, current, maps: [{name, current}]}`, sorted by name. No root →
`not_supported`.

**POST /maps/new** `{name}`. 400 `not_slug`; no root or no `switch_map` hook → `not_supported`;
a switch already accepted → `switching` (nothing created); the directory exists → `exists`; else the service creates `<map_root>/<name>/` (a repeated `new`
sees `exists` before the switch lands) and calls `switch_map({name, create: true})` →
`{ok, map, pending: true}`. A hook returning false → `not_supported` (the directory stays).

**POST /maps/open** `{name}`. 400 `not_slug`; no root / no hook → `not_supported`; a switch
already accepted → `switching`; no such
directory → `unknown_map`; `name` is the current map → `{ok, map, pending: false}` without the
hook; else `switch_map({name, create: false})` → `{ok, map, pending: true}`.

The host records `<map_root>/current` only once the map has booted (right after
`PoseGraph::Start`, on the first scan). Opening a damaged map directory kills the host on that
first scan, exactly like booting on one. An opened map seeds its pose from its own `last_pose.pb`:
if the robot moved while another map was open, follow with `init-pose` or `relocalize`.

`pending: true` is intent: the host swaps the map seconds later, the port is closed meanwhile, and
`egs` polls `GET /root` until its `map` is the new name (transport errors count as "still
switching", ≤ max(poll, 30) s).

### fs passthrough (no task)

For agents without the directory. Paths relative to `memory/`. No index; `grep` is the only search.

- Rejected (400): absolute or empty paths, `..`, NUL, any existing symlink component (`lstat`
  each), a result outside `memory/` after `realpath`; `not_slug` on every component of a `mkdir`
  path and on the new name of a directory `mv`. `reserved_name`: an `mv` whose `to` has a
  `skills` or `attachments` component while `from` is, or holds anywhere below it, a `place.yaml`
  or `node.yaml` (the scans skip those subtrees, so the place would vanish). `mkdir
  .../skills/<name>` and moving skill directories stay legal.
- Process-owned (400 `owned_by_process`): any `place.yaml`, `memory/index.tsv`, `memory/README.md`
  cannot be written, appended, moved or removed as files. Moving or `rm -r` of a directory that
  contains them is allowed: that is how a place is renamed or deleted.

| call | params | effect / response |
|---|---|---|
| `GET /fs/ls` | `path` | `{ok, entries: [{name, type: dir|file, size}]}`, sorted |
| `GET /fs/tree` | `path, depth=3` (≤ 6) | `text/plain`, like `tree -L depth`; a non-empty directory cut off by the depth gets one `└── …` child line (not counted) |
| `GET /fs/cat` | `path` | raw bytes; missing → 404 `not_found`; not a file → `fs_error`; > 8 MiB (the request body limit, so every attachment reads back) → 400 `too_large` |
| `GET /fs/grep` | `pattern`, `path=.`, `include` | below |
| `POST /fs/write` | `path`, body | temp + rename (`WriteFileAtomically`); parent must exist |
| `POST /fs/append` | `path`, body | `open(O_WRONLY\|O_APPEND\|O_CREAT)`, one `write()` |
| `POST /fs/mkdir` | `path` | `mkdir -p` |
| `POST /fs/mv` | `from, to` | `rename(2)`; `to` existing → `exists` |
| `POST /fs/rm` | `path, recursive=0` | file or empty dir; non-empty without `recursive=1` → `not_empty` |

`GET /fs/grep` searches lines for `pattern` as a fixed string, ASCII case-insensitive (no regex);
missing or empty `pattern` → 400 `bad_param`. `path` (default `.`) is a directory, walked in
byte order of names, or one file; missing → `fs_error`. In a walk, a file is searched when its
name matches one of `include`'s comma-separated globs (`fnmatch`; default
`node.yaml,notes.md,SKILL.md,*.md,*.yaml`). Always skipped: `observations.jsonl`, directories
named `attachments`, symlinks, files over 1 MiB. Lines split on `\n`, a trailing `\r` dropped,
numbered from 1. `→ {ok, matches: [{path, line, text}], truncated}`, `path` relative to `memory/`;
at most 200 matches, `truncated: true` when a 201st exists. `egs fs grep WORDS... [--path P]` joins
the words with single spaces into `pattern`, prints `path:line:text` and does the same walk itself
in local mode.

## Anchor hooks inside PoseGraph

PoseGraph owns `AnchorStore anchor_store_` next to `map_manager_` and is its only writer. A hook
runs in the task of the graph change it follows, or a reader in between sees a vanished id:
`ApplyTrim` (every trim, cadence or `TrimCoveredSubmapsOnTask`), after `data_.ApplyTrim` →
`OnTrim(report)`; `TransferUnfinishedSubmaps` (the freeze rotation), per handed-over submap →
`OnNodesTransferred(renamed)` (the nodes only it held move to the successor and get its ids), then
after the `submap_translation_` re-key and before the freeze → `OnSubmapsTransferred(adoption)`;
`RemoveSession` (agent, `GcFloatingSessions`, a trim that emptied an idle session or a freezing one), after
`data_.RemoveSession` → `OnSessionRemoved(id)`. A freeze needs no hook
and there is no anchor observer: `frozen` is derived at Resolve.

**Persistence**: `MapManager` holds `const AnchorStore&` and alone decides when the table is
written: inside a commit, only when `revision()` moved since its last write or `ReadAnchors()`
(`message AnchorFile` in `lifelong/map_manager/proto/anchors.proto`). A commit writes every file it
changes under a new generation-suffixed name (`session_NNNNNN.gG.pb`, `anchors.gG.pb`, each
`common::WriteFileAtomically`), then renames the manifest (`generation`, `anchors_file_name`) into
place last; the manifest is the only truth, so a kill anywhere leaves exactly one generation. A
failed file write abandons the commit (its files removed, the manifest untouched), and every later
commit first reconciles the manifest with the graph (drops removed sessions, writes frozen ones not
yet frozen on disk), so a failed freeze or removal lands with the next commit. `Load` is
read-only; `RemoveUnreferencedFiles()`, which `Start` calls right after it, removes
`session_*.pb`, `anchors*.pb` and `*.tmp` the manifest does not name. Commits: `Checkpoint` (fed
session), `OnSessionFrozen` (frozen file, every unfrozen session), `OnSessionStarted` (new
session), `RemoveSession` (every unfrozen session), `Commit(graph)` (at Start: reconcile, anchors),
`RecordBoot` (manifest only; if it fails, Start stops: "map directory is not writable"). One process per map directory: `RemoveUnreferencedFiles` must never
run against a directory another process is committing to. No manifest-named anchors file reads as
an empty table; a named one that is missing or unparsable fails Start rather than reissue ids.
Trim, rotation and removal change the table in the same task as the graph, before the commit that
persists both.

**Start**: `Load`, `RemoveUnreferencedFiles`, `RecordBoot`, `FinishSubmaps`, `BuildFrom`,
`Restore(ReadAnchors())`, `GcFloatingSessions`, `Reconcile(data_)`, `Commit(data_)`. With a `map_manager_`, Restore **always**
runs, on a fresh directory with an empty table; a damaged directory never gets this far. The
anchors file is never rewritten with a lower `next_id` than the one read.

**SaveAnchorOnTask(keep_scan, rebind, node_from_anchor)** = `Save` (or `Rebind`) on
`last_ingested_node_`, at `node pose * node_from_anchor` (default identity; a non-zero translation
must land on known free space in `AnchorStore::BindingSubmap(node)`, read via `SnapshotCopy`, else
`OFFSET_NOT_FREE` before the table is touched; the stored scan is in the anchor frame) (valid
right after a rotation too; `NO_KEYFRAME` when `last_ingested_node_index_ < 0`, since the graph's
newest node may be a previous boot's) → `CheckpointSessions()`, whose commit carries the new
table: a returned anchor survives a kill. A failed commit returns `NOT_PERSISTED` and no anchor,
and rolls the table back to what the disk has (`AnchorStore::Replace` for a rebind, `Erase` for a
new row); `next_id` stays past the refused id, which is never issued.

## Core entry points this needs (contract; implemented with the service)

- `PoseGraph`, task-only: `const AnchorStore& anchors() const`; `std::optional<NodeId>
  last_ingested_node() const` (empty when `last_ingested_node_index_ < 0`); `SaveAnchorResult
  SaveAnchorOnTask(bool keep_scan, std::optional<AnchorId> rebind = std::nullopt, const
  Eigen::Affine2d& node_from_anchor = Identity())`; public
  `bool CheckpointOnTask()`; `SessionId boot_first_session() const` (the fed session right after Start).
  Any thread but the backend task: `DropFedSessionResult DropFedSession(SessionId id)` (blocks;
  `DropFedSessionOnTask` is the task-side body).
  Anywhere: `FreezeFedSession(std::optional<SessionId> expected = std::nullopt)` → the new
  `SessionManager::FreezeFedSession(expected)`; `Start(..., bool seed_from_previous_boot = true)`.
- `PoseGraph`, any thread: `RelocalizationId SetInitialPose(pose)`, `RelocalizationId
  RelocalizeGlobally()`, `relocalization(id)`, `latest_relocalization()`, `EventsSince(seq)`.
- `AnchorStore::Rebind`. `AssembleGlobalMap(..., std::optional<SessionId>)` already exists.
- Zones (`service/memory/zone_store.h`), for a host's own backend task, never a host thread waiting on
  one: `std::optional<std::vector<ResolvedZone>> ResolveZones(const PoseGraph&, const ZoneStore&)`
  (scans, then resolves; nullopt when the scan was incomplete); `ResolveZonesOnTask(graph,
  scan.zones)` on a `ZoneStore::Scan()` taken earlier (check `scan.error` first) keeps the file
  reads off the graph thread. `AgentHost::zones()` gives the ROS node its `const ZoneStore&`.
- `MapManager::ReadAnchors` and the commit, `lifelong/map_manager/proto/anchors.proto`; the protoc
  rule in `core/CMakeLists.txt` (single file today) becomes a loop over both protos.

## Build and hosts

- **httplib moves to a top-level `third_party/`** with `stb_image_write.h` (in the service commit;
  webui CMake, `compile_flags.txt`, README follow). Static `evergreenslam_agent` (links
  `evergreenslam`, yaml-cpp, Threads) joins `evergreenslam_ros` via `add_subdirectory` behind
  `EVERGREENSLAM_WITH_AGENT` (default ON); `adapters/agent/` gets a `COLCON_IGNORE`.
- Host flags on the bag main and the live node: `map_root` / `map` replace `map_dir` (bag:
  `--map_root DIR --map NAME`, default the root's `current` or `default`, no runtime switch; live
  node: `-p map_root:=` default `~/.evergreenslam/maps`, empty = no persistence and no agent
  service; `-p map:=` default empty = `current` or `default`; it wires `switch_map` and
  `drop_session`, the bag main neither), `--agent_port`
  (0 = off; live node default 8643), `--agent_bind` (default `127.0.0.1`), `--ignore_last_pose` (Start seeds from neither `last_pose.pb` nor the checkpoint
  node, so the demo's `init-pose --place` correction is controlled). The TSan run (`tools/run_tsan.sh`) covers the lifelong tests only, not the service.
- A damaged map directory (unreadable or pre-v4 manifest, a missing or corrupt session file, or
  session/anchor files with no manifest) refuses to boot: `Start` stops naming the directory and
  the reason, and touches no file. Only a directory with neither boots as a new map. A directory
  the boot's manifest cannot be written to stops too ("map directory is not writable").

## Keep-out mask (ROS live node)

`adapters/ros2/evergreenslam_ros/src/keepout_mask_publisher.{h,cc}`, with
`EVERGREENSLAM_WITH_AGENT` and an agent host. On the graph timer (`map_publish_period`) the
executor thread scans the zone files and enqueues one task: skipped (nav2 keeps the last mask)
while `has_frozen_base && !aligned_to_base` or when the scan was incomplete; otherwise the keepout
zones with a `polygon_xy` become a
`nav_msgs/OccupancyGrid` on `keepout_mask_topic` (`keepout_filter_mask`), `map_frame`, QoS
transient_local + reliable, 0.05 m over the polygons' bounding boxes + 0.5 m, cell centre inside
→ `100`, else `0`; the resolution doubles while the grid exceeds 4e6 cells, and past 0.8 m the
last mask stays (logged). No zone → a 1x1 grid of `0` (clears a previous mask); unchanged masks
are not republished. The publisher sends that clearing grid when it is created, so a map switch
drops the previous map's zones at once; nothing else is published until the new
map's zones resolve (and, with a frozen base, until it is aligned). nav2's `costmap_filter_info_server` is the user's (package README, "nav2 keepout").
The bag replay has no ROS node and publishes no mask.

## Tests

Simulated-laser e2e in `core/test/lifelong/anchors/` on `core/test/lifelong/testing/`:
`AnchorSurvivesLoopClosure`, `AnchorSurvivesTrimBySuccession`, `AnchorSurvivesFreezeRotation`,
`SaveLoadRestartIsByteIdentical` (through `Finish`), `SaveKillRestart` (kill right after
`SaveAnchorOnTask` returns, no `Finish`: resolves after restart), `GcOfIdleSessionFlagsOrphan`,
`SetInitialPoseByPlaceThenRelocalize`, `DeleteIdleSessionKeepsOthers`,
`TrimThenKillNoLongerOrphans` (a kill inside the commit that trims the anchor's submap restarts on
the previous generation, still BOUND), `UnpersistedSaveIsRefused` (a save whose commit fails
returns no id and changes nothing), `StartCollectsAnInterruptedCommit` (Start loads the previous
generation and removes the killed commit's files).

## Deviations from the proposal

- Frozen is derived at Resolve, not stored; stored states are BOUND/REBOUND/ORPHAN.
- Re-saving a place rebinds the same anchor id; `place.yaml` never changes after its first write.
- The anchors file is also committed at save (with the checkpoint), rotation and removal.
- Bind address is a flag (`--agent_bind`), so compose can map it from inside a container.
- `find` / `observe` / `session new` are CLI-only; `session ls` has no offline mode.
- Plan → apply is locked by `plan_token`; `freeze/plan` solves first and does not predict the trim.
- Eleven named e2e tests instead of six.
