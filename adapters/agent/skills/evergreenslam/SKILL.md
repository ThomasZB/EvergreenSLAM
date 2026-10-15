---
name: evergreenslam
description: Spatial memory of a robot running EvergreenSLAM. Use when you need to know where the robot is, where a place or thing is, remember a place or an object, look at the map, or manage mapping sessions. memory/ is a directory tree of places and things on the robot that you work in with `egs fs`; `egs` also answers what only the live SLAM process knows.
---

# EvergreenSLAM spatial memory

## Start here

`egs help agent` (CLI) prints the rules and command table in 30 lines; read the rest of this file once.
With MCP (`egs mcp`), `egs <cmd> [<sub>]` is the tool `egs_<cmd>[_<sub>]` (`init-pose` is
`egs_init_pose`), flags are its parameters, results are the `--json` objects, `egs_fs_cat`
returns the file itself, and `egs_view` also returns the image.

1. `egs root`. It checks the connection and says where `memory/` lives: usually
   `remote http://127.0.0.1:8643  map <name>` (the robot, through an SSH tunnel). If it cannot
   connect, it prints the URL it tried; ask the human to open the tunnel or set `EGS_URL`.
2. `memory/` is your workspace: directories are places and things, the path is containment
   (`places/kitchen/table/apple`). Work in it with `egs fs ls|tree|cat|grep|write|append|mkdir|mv|rm`,
   paths relative to `memory/` (`egs fs tree places`; `-L 6` for all levels; `…` marks a cut-off
   directory). Only if `egs root` printed a local path may you also use the shell there.
3. Read `memory/README.md` (`egs fs cat README.md`) before you change anything. The process
   rewrites it at every start.
4. Before acting at a place, read the files `egs where places/<p>` lists as `read first:`: each
   `SKILL.md` and `notes.md` that exists from `places/` down to the place (most are absent; that is
   normal). Skills under those directories apply; the closest wins; the user beats them all.
5. Where the robot is comes only from `egs here`.

## Problem → command

Usage layer: call freely.

| problem | command |
|---|---|
| where am I | `egs here` |
| where is a place or a thing | `egs where places/<path>` |
| find a thing by name, alias or kind | `egs find <words>` |
| search notes, skills and node.yaml | `egs fs grep <words> [--path places/<p>]` (the words as one fixed phrase, any case; never the observation logs) |
| remember the spot the robot stands at | `egs place save places/<path>` (saving again updates it) |
| remember a spot 1 m ahead without driving there | `egs place save places/<path> --offset 1 0 0` (≤ 3 m, free space) |
| remember a thing you see (a cup, an extinguisher) | **Remember a thing you see** below |
| I saw it / looked and it is not there | `egs observe places/<path>` / `--result absent` (`--note ...`) |
| keep a photo or file with what I saw | `egs observe places/<path> --attach <file>` (≤ 8 MiB; `egs fs cat <f> -o <file>` reads it back) |
| it is 0.8 m ahead, 0.3 m left of the robot | `egs observe places/<path> --offset 0.8 0.3 0` (only within 10 m of its place) |
| it was in a photo taken a moment ago | `egs observe places/<path> --offset 0.8 0.3 0 --at <stamp_ns>` (the robot's pose when the photo was taken, ≤ 10 s ago) |
| robot pose for a program, at high rate | `egs pose --watch 10 --json` (one JSON line per new scan; Ctrl-C stops) |
| go somewhere | `egs where` → give its `approach` pose to your navigator |
| the navigator must avoid an area | `egs zone add places/<place>/<name> --rect X0 Y0 X1 Y1` (robot frame: x ahead, y left; within 10 m of the place) |
| list or delete keep-out zones | `egs zone ls` / `egs zone rm places/<place>/<name>` |
| is the map healthy, linked to the base | `egs status` |
| a picture | `egs view <preset>` (next table) |
| export the map as files | `egs snapshot -o <dir>` (copies or downloads them; dir outside `memory/`: exports carry XY) |
| the robot's pose is wrong | it stands at a saved place now (within ~2 m): `egs init-pose --place places/<p>`; it moved since, or you do not know where it is: `egs relocalize` (searches the whole saved map); see **Recovery** below |
| what happened to the pose | `egs events` (`--since N`, `--follow`) |
| make sure it is on disk now | `egs checkpoint` |
| new container, rename, re-parent, delete | `egs fs mkdir`, `egs fs mv`, `egs fs rm -r` (a place's binding moves with its directory) |
| notes and skills | `egs fs write places/<p>/notes.md --file <f>` (or stdin; replaces the file atomically; also `SKILL.md`, `skills/<name>/SKILL.md`); `egs fs append` adds lines |

Management layer: only when a human asks; the one exception is the fed-session `rm` in
**Recovery when the pose is wrong**. Every session command prints a plan and changes nothing
without `--yes`; show the plan to the human before you add `--yes`. Map commands take no `--yes`:
they delete nothing.

| problem | command |
|---|---|
| list maps | `egs map ls` |
| start over on a fresh map (the old one stays on disk) | `egs map new <name>` |
| switch to a saved map | `egs map open <name>`; if the robot moved since that map was last open, then `egs init-pose --place places/<p>` or `egs relocalize` |
| list sessions | `egs session ls` |
| make the current session part of the permanent map | `egs session freeze`, then `egs session freeze --yes` |
| start a fresh session | `egs session new`, then `--yes` (same as freeze) |
| delete an idle session | `egs session rm <id>`, then `egs session rm <id> --yes` |
| the last stretch of mapping is broken (a fall, a carried robot) | `egs session rm <fed id>`, then `--yes`: the whole current session goes at once and a fresh one is fed; the robot's pose is lost, so then `egs init-pose --place places/<p>` or `egs relocalize` |

After `egs map new` / `egs map open` the workspace changes: run `egs root` again.
The other map's places are not visible from the new one, and the odometry frame restarts.
`switching`: a map switch is still in progress; wait for it, then `egs map ls`.
`session_changed`: a freeze replaced the fed session between plan and apply; `egs session ls`, then
plan the rm again.
`plan_changed`: the map moved between plan and apply, nothing was done; plan again. `not anchored`:
the session has no link to the frozen map yet; drive where they overlap. Freezing cannot be undone.
A freeze can end in `removed, not frozen`: the session only retraced the frozen map, so there was
nothing to add; the fresh fed session carries on as usual.

## Question → view preset

| question | command |
|---|---|
| is the map right? | `egs view map` |
| is localization drifting? (scan vs map) | `egs view here` (7.5 m around the robot; `--full` for the whole map) |
| where are the keep-out zones? | `egs view map` (hatched) |
| how far is X, what lies between? | `egs view route places/<x>` |
| where have I been? | `egs view trail` |
| is this session sane? | `egs view session [id]` |

Prefer presets; use `egs view custom --layers a,b,...` only when no preset can answer (at most 4
layers; the first output line of any preset shows its layers, so you learn the names there).
`--ego [R]` crops any preset to R m (default 7.5) around the robot; `view here` does so unless
`--ego 0` or `--full`. Colors: walls black, free space white, unknown grey, scan magenta, robot
green, places orange, keep-out zones hatched yellow. One image per task. The last output line is
the PNG path on this machine;
`-o <file>` saves it elsewhere, `-o -` writes the PNG bytes to stdout.

## Dead rules

1. No map-frame XY in any file you write; ask `egs where` for a current position.
2. `place.yaml` belongs to the process: never write, edit or `cp -r` it; `egs place save` instead.
   Save places standing at them, or within 3 m in free space you can see (`--offset`).
3. Things you can stand in front of (table, fridge, door) are places; small movable things are
   plain directories with a `node.yaml` (`kind`, `aliases`, `labels`) and never a `place.yaml`.
4. To find an object use `egs find`; to search text use `egs fs grep`, never the observation logs.
5. `egs` paths are relative to `memory/` and start with `places/`.
6. `skills` and `attachments` are reserved names, never a place or a thing. No symbolic links.
7. `observations.jsonl` is append-only; retract a line with `egs observe <path> --invalidates <id>`.
8. There is no `egs goto`. `egs` never moves the robot.
9. Keep-out zones go through `egs zone add`; never write `zone.yaml` by hand.

## After a start, before the first move

1. `egs here`. A first line `linked to base: no` means the pose is not yet tied to the saved map.
2. While the robot drives through mapped area, `egs here --sample 20`: it adds the median and
   minimum `base` over the keyframes seen at `cover` >= 50%. One read is never enough.
3. Healthy: median 0.5–0.8. Probably wrong: median below 0.45 from 5 or more reads, more so
   with `since base match -` or `linked to base: no` (a pose 0.25 m or 5° off already scores
   0.20–0.45). `base` lags a correction by 30–45 s, and a first one is often refined by a second
   (`egs events` shows them land): `--sample` drops the reads from before one and says
   `restarted N`; sample again until the median settles (up to a minute). Then `egs view here`
   (the scan should lie on the map's walls); if it does not, go to **Recovery**.

## Recovery when the pose is wrong

1. The robot stands at a saved place now (within ~2 m): `egs init-pose --place places/<p>` (or
   `x y theta`). It moved since (a "switched on at the dock" is stale once it drove), or you do
   not know where it is: `egs relocalize`, which searches the whole saved map. Both wait up to
   30 s (`--wait S`) and print the outcome:
   - `matched: corrected D m A°`: applied. `egs here --sample 20` until the median settles:
     `linked to base yes` and `base` healthy means done. `matched` says the hint was used, not
     that the pose is now right.
   - `no_match (0 candidates)`: that place or pose lies outside the saved map; check the place
     name, or `egs relocalize` to search the whole map.
   - `no_match (N candidates)`, or `matched` but `base` low after a minute: the session holds a
     wrong constraint that one hint cannot pull back. Go to step 2.
   - `superseded by request M`: a newer request replaced it; `egs here` shows the newest.
   - `outcome unknown`: still pending; the `relocalization` line of `egs here` shows it later.
2. `egs session rm <fed id>` (the `fed` id from `egs session ls`), then the same with `--yes`. You
   may do this without asking a human, but only for the fed session and only here: it drops the
   current session, a fresh one is fed, and the pose is lost.
3. Step 1 again (`init-pose --place` or `relocalize`, by the same rule), then `egs here`.

## Remember a thing you see

1. `egs here`: if no `closest` place is within 10 m, first `egs place save places/<area>/<spot>`
   where the robot stands (e.g. `aisle-3-end`). The thing itself never gets a `place.yaml`.
2. `egs fs mkdir places/<place>/<thing>`, then `egs fs write places/<place>/<thing>/node.yaml`
   with `kind: <word>` and `aliases: [<other names>]`.
3. `egs observe places/<place>/<thing> --offset DX DY 0` (where it is from the robot *now*: x
   ahead, y left), or without `--offset` when the place's own position is close enough. Do 1–3
   right away: the robot keeps moving, and egs stores the offset from the place, not the robot.

## Reading answers

- `current -` in `egs here`: no saved place within 2.5 m of the robot; `closest` still names the
  nearest two.
- `egs here` starting with `linked to base: no`: the robot has not yet been matched to the saved
  map, so `current` and `closest` are not to be trusted. Wait, drive through an area that is
  already mapped, or follow **Recovery**, then ask again.
- `relocalization N <kind> <state>` in `egs here` / `egs status`: the latest `init-pose` or
  `relocalize` and how it ended (`pending`, `matched: corrected ...`, `no_match`, `superseded`).
- `egs events`: one line per backend event, oldest first: `relocalization`, `pose_correction` (a
  re-optimization moved the robot's pose more than 0.3 m or 0.2 rad), `session_frozen`,
  `session_removed`, `fed_session_dropped`. Resume with `--since <next_seq>`; `events lost` on
  stderr means older ones fell off the log.
- The localization line, `base S (cover C%)  since base match Dm  match M (avg A)`:
  - `base`: how well the latest keyframe scan, at its pose, fits the saved (frozen) map, 0..1.
    It is consistency, not proof: moved furniture and people lower it, and single reads down to
    0.40 occur on a correct pose. `cover` is how much of the scan could be checked at all; below
    50% (an area the saved map lacks) or `base -`, it says nothing either way. It only changes
    with a new keyframe: a robot standing still repeats one read.
  - `since base match`: metres driven since the robot was last matched to the saved map. It grows
    in new areas and drops back when a match lands; `-` means no match yet.
  - `match` is 0.6–0.8 on a healthy run. Below ~0.5 for more than a few seconds (right after start
    is normal): the scan does not fit the map; drive slowly through a mapped area, or `egs view
    here`. It only compares the scan with the local map: a high `match` does not mean aligned.
  - `since base match` can drop a few seconds before the correction lands, so a low `base` right
    after a match is not yet a verdict. A median `base` below 0.45 at `cover` >= 50% with
    `since base match` growing in an area that is mapped means the pose is probably wrong:
    `egs view here`, then **Recovery**.
- `--at` is the robot's sensor clock (its scan stamps), not this computer's: take the photo's time
  on the robot, or read the robot's time now from `egs pose --json` (`stamp_ns`) to calibrate.
  `too_old` (over 10 s ago), `in_future` (ahead of the newest scan: wrong clock) and `gap` (no
  scans around then) write nothing.
- `where` precision: `own` (its own place), `offset` (measured from the place above), `inherited`
  (the position of the place or offset above; not observed), `none` (no bound ancestor). Navigate
  to `approach` (the place the thing hangs from), never to `xy` (the thing itself, which can be a
  few metres from `approach`).
- State `pending`: navigable now; its coordinates can move when the map re-optimizes, ask
  `egs where` again before each trip. `frozen`: fixed for good. `rebound`: pending again after a
  re-save.
- Every command but `fs cat` takes `--json` (`fs`, `zone` and `session` too); prefer it over
  parsing text. Under `--json` every pose is `[x, y, theta]`, `robot` from `egs here` included.
- `not linked to the base map` from `observe --offset` or `zone add`: the robot pose and the
  place's anchor are in different frames until `egs status` says `linked to base yes`; drive
  through mapped area first, then retry.
- `unresolvable ... orphan`: that part of the map is gone. Go there and `egs place save` it again.
- `frame_mismatch` in `egs zone ls`: the zone's directory moved under another place. `egs zone rm`
  it, stand near the new place and `egs zone add` it again.
- `@solve N` says which optimization the numbers came from; they move when the map re-optimizes.
- Times are ages (`35s`, `20m`, `6d`). `find` puts things last seen `absent` at the bottom and
  says what matched when it was not the name: `places/a/pallet-jack  (alias "hand truck")`.
- Exit codes: 0 done (a session plan printed without `--yes` also exits 0: `plan only`), 1 refused,
  plan rejected, `no_match` or `superseded`, 2 bad usage or path, 3 process unreachable. `outcome unknown`
  means a change may or may not have happened: check with `egs status`, `egs here` or
  `egs session ls` before retrying.
