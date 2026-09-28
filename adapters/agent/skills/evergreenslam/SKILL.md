---
name: evergreenslam
description: Spatial memory of a robot running EvergreenSLAM. Use when you need to know where the robot is, where a place or thing is, remember a place or an object, look at the map, or manage mapping sessions. memory/ is a directory tree you work in with the shell; `egs` answers what only the live SLAM process knows.
---

# EvergreenSLAM spatial memory

## Start here

1. `cd "$(egs root)"`. `memory/` is your workspace: directories are places and things, the path is
   containment (`places/kitchen/table/apple`). Work in it with `ls`, `find`, `grep`, `cat`, `mv`.
   If `egs root` says the directory is not readable here, use `egs fs ls|tree|cat|write|append|mkdir|mv|rm`
   with the same paths instead of the shell.
2. Read `memory/README.md` before you change anything. The process rewrites it at every start.
3. Before acting at a place, read its `SKILL.md` chain: `places/SKILL.md`, then each directory down
   to the place. Skills under those directories apply; the closest wins; the user beats them all.
4. Where the robot is comes only from `egs here`. Your shell stays in `memory/`.

## Problem → command

Usage layer: call freely.

| problem | command |
|---|---|
| where am I | `egs here` |
| where is a place or a thing | `egs where places/<path>` |
| find a thing by name, alias or kind | `egs find <words>` (never `grep` observation logs) |
| remember the spot the robot stands at | `egs place save places/<path>` (saving again updates it) |
| I saw it / looked and it is not there | `egs observe places/<path>` / `--result absent` (`--note ...`) |
| it is 0.8 m ahead, 0.3 m left of the robot | `egs observe places/<path> --offset 0.8 0.3 0` (only within 10 m of its place) |
| go somewhere | `egs where` → give its `approach` pose to your navigator |
| is the map healthy, aligned to the base | `egs status` |
| a picture | `egs view <preset>` (next table) |
| export the map as files | `egs snapshot [-o dir]` (dir outside `memory/`: exports carry XY) |
| the robot's pose is wrong | `egs init-pose --place places/<p>` (or `x y theta`), then `egs here`; else `egs relocalize` |
| make sure it is on disk now | `egs checkpoint` |
| new container, rename, re-parent, delete | `mkdir`, `mv`, `rm -r` (a place's binding moves with its directory) |
| notes and skills | `cat > f.tmp` then `mv f.tmp notes.md` (or `SKILL.md`, `skills/<name>/SKILL.md`) |

Management layer: only when a human asks. Every session command prints a plan and changes nothing
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
| delete a floating session | `egs session rm <id>`, then `egs session rm <id> --yes` |

After `egs map new` / `egs map open` the workspace changes: run `egs root` and `cd` there again.
The other map's places are not visible from the new one, and the odometry frame restarts.
`switching`: a map switch is still in progress; wait for it, then `egs map ls`.
`plan_changed`: the map moved between plan and apply, nothing was done; plan again. `not anchored`:
the session has no link to the frozen map yet; drive where they overlap. Freezing cannot be undone.

## Question → view preset

| question | command |
|---|---|
| is the map right? | `egs view map` |
| is localization drifting? (scan vs map) | `egs view here` |
| how far is X, what lies between? | `egs view route places/<x>` |
| where have I been? | `egs view trail` |
| is this session sane? | `egs view session [id]` |

Prefer presets; use `egs view custom --layers a,b,...` only when no preset can answer (at most 4
layers; the first output line of any preset shows its layers, so you learn the names there).
`--ego` crops to 7.5 m around the robot. One image per task. The last output line is the PNG path.

## Dead rules

1. No map-frame XY in any file you write; ask `egs where` for a current position.
2. `place.yaml` belongs to the process: never write, edit or `cp -r` it; `egs place save` instead.
   Save places only while standing at them.
3. Things you can stand in front of (table, fridge, door) are places; small movable things are
   plain directories with a `node.yaml` (`kind`, `aliases`, `labels`) and no `place.yaml`.
4. `grep -r` always takes `--include=node.yaml`; to find an object use `egs find`.
5. `egs` paths are relative to `memory/` and start with `places/`.
6. `skills` is a reserved name, never a place or a thing. No symbolic links.
7. `observations.jsonl` is append-only; retract a line with `egs observe <path> --invalidates <id>`.
8. There is no `egs goto`. `egs` never moves the robot.

## Reading answers

- `egs here` starting with `aligned to base: no`: the robot has not yet been matched to the saved
  map, so `current` and `closest` are not to be trusted. Wait, drive through an area that is
  already mapped, or `egs init-pose --place places/<p>` / `egs relocalize`, then ask again.
- `where` precision: `own` (its own place), `offset` (measured from the place above), `inherited`
  (the position of the place or offset above; not observed), `none` (no bound ancestor). Navigate
  to `approach`, never to `xy`.
- `unresolvable ... orphan`: that part of the map is gone. Go there and `egs place save` it again.
- `@solve N` says which optimization the numbers came from; they move when the map re-optimizes.
- Times are ages (`35s`, `20m`, `6d`). `find` puts things last seen `absent` at the bottom.
- Exit codes: 0 done (a session plan printed without `--yes` also exits 0: `plan only`), 1 refused
  or plan rejected, 2 bad usage or path, 3 process unreachable. `outcome unknown`
  means a change may or may not have happened: check with `egs status`, `egs here` or
  `egs session ls` before retrying.
