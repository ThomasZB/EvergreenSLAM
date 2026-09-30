# memory/

This directory is the robot's spatial memory. Directories are places and things; the path is
containment (`places/kitchen/table/apple` = the apple on the kitchen table). You work here with
plain shell (`ls`, `find`, `grep`, `cat`, `mv`, `rm`). Use `egs` only for what the running SLAM
process alone knows: coordinates, saving a place, maps, sessions.

Read this file before you change anything here. The SLAM process installs it and overwrites it at
every start, so do not edit it.

## Layout

```
memory/
  README.md                          this contract (process)
  index.tsv                          places at the last `egs snapshot` (process, derived)
  places/
    SKILL.md                         home index: rules for every place
    skills/return-to-dock/SKILL.md   a skill valid everywhere
    dock/place.yaml                  binding: {anchor: 7} (process)
    dock/node.yaml                   kind, aliases, labels (you)
    kitchen/place.yaml
    kitchen/SKILL.md                 what to know in the kitchen
    kitchen/skills/clean/SKILL.md    a kitchen skill, may carry scripts/
    kitchen/notes.md                 free notes
    kitchen/slope/zone.yaml          a keep-out area, in kitchen's anchor frame (egs zone add)
    kitchen/fridge/place.yaml        a fixed thing you can stand at is a place
    kitchen/table/node.yaml          no place.yaml: a container, no geometry of its own
    kitchen/table/apple/node.yaml
    kitchen/table/apple/observations.jsonl
    kitchen/table/apple/attachments/<id>.jpg   a file an observation line names; never a node
```

Directory names are ASCII slugs, `[a-z0-9][a-z0-9_-]*`. Names in other languages go into
`node.yaml: aliases`, which `grep` finds. File names are free (no `/`, no NUL).

## Who writes what

| file | writer | how |
|---|---|---|
| `place.yaml` | SLAM process, via `egs place save <path>` | atomic replace; never edit it |
| `index.tsv` | SLAM process, via `egs snapshot` | atomic replace; never edit it |
| `README.md` | SLAM process, at start | never edit it |
| `node.yaml` | you or a human; `egs observe --offset` | `cat > f.tmp` then `mv f.tmp node.yaml` |
| `notes.md` | you or a human | `cat > f.tmp` then `mv` |
| `SKILL.md`, `skills/**` | you or a human | `cat > f.tmp` then `mv` |
| `observations.jsonl` | you, or `egs observe` | append one line per write (`>>`); never edit |
| `attachments/` | you, via `egs observe --attach <file>` | copied before the line that names it; never edit |
| `zone.yaml` | you, via `egs zone add` | relative to the nearest place's anchor; `egs zone rm` deletes it |

One agent session per map at a time: every file has one writer *class* (the process, or you and
humans), so nothing is locked.

## Dead rules

1. **No map-frame XY in any file you write.** Coordinates move every time the map is re-optimized.
   The only XY on disk is `index.tsv` and `<map_dir>/snapshots/`, both derived, stamped with the
   solve they came from, and stale by design. For a current position run `egs where <path>`.
2. **`place.yaml` is `{anchor: <id>}` and nothing else.** The process writes it when you stand
   somewhere and run `egs place save <path>`; saving an existing place again keeps its anchor id and
   updates the pose. It always means "a pose the robot stood at" (or could stand at), never the
   centre of an object. Save places standing at them, or within 3 m in free space you can see
   (`egs place save <path> --offset DX DY DTHETA`, robot frame: x forward, y left).
3. **Things you can stand in front of (table, stove, fridge, door, bed) are places**: save them.
   Small movable things are plain subdirectories with a `node.yaml`, never a `place.yaml`. An
   object's position is inherited from its nearest ancestor with a `place.yaml`, or given by
   `node.yaml: offset` + `offset_from` (below).
4. **`grep -r` always takes `--include=node.yaml`**, or it drowns in observation logs. To find an
   object, use `egs find <query>` only: it ranks by name, alias, kind, and sinks things last seen
   absent.
5. **`egs` paths are relative to `memory/` and start with `places/`**, exactly as `find places ...`
   prints them. Your shell stays in `memory/`; it does not follow the robot. Where the robot is
   comes only from `egs here`.
6. **`skills` and `attachments` are reserved names**: never a place or a thing, and never contain
   `place.yaml`.
7. **No symbolic links.** `find`, `tree` and `grep` disagree about them and `egs` refuses them. A
   second name for the same thing goes into `aliases`; two things with one name are two directories.
8. **There is no `egs goto`.** `egs` never moves the robot. To go somewhere, hand the `approach`
   pose from `egs where` to the navigation stack your host gives you.
9. **Keep-out zones go through `egs zone add`**; the polygon is in the anchor's frame and moves
   with the map.

Changing the tree: `mv` renames or re-parents (a place's binding moves with its directory);
`rm -r` deletes; `mkdir` makes a container with no geometry. Never `cp -r` a directory that holds
a `place.yaml`: the copy would claim the same anchor. Stand there and `egs place save` instead.

## node.yaml

```yaml
kind: fridge               # required; one lowercase word (room, spot, fridge, table, cup, ...)
aliases: [冰箱, icebox]     # optional; other names, any language
labels: [food]             # optional; cross-cutting tags, never inherited
offset: [0.8, 0.3, 0.0]    # optional; [dx, dy, dtheta] in m and rad
offset_from: 7             # required with offset: anchor id of the nearest place.yaml above
```

`offset` is relative to the robot pose stored by `offset_from` (x forward, y left). It is valid only
while that anchor is still the nearest `place.yaml` above this directory; after a `mv` it is
ignored and the position falls back to inherited. Prefer `egs observe <path> --offset dx dy dtheta`
(given in the robot's current frame), which computes and writes both fields.

## zone.yaml

```yaml
kind: keepout                        # the only kind; others are ignored
frame: places/kitchen                # the nearest place.yaml above, at the time of `egs zone add`
polygon: [[1.0, -1.0], [3.0, -1.0], [3.0, 1.0], [1.0, 1.0]]   # m, that place's anchor frame
```

An area the navigator must avoid (a slope, a wet floor), in its own directory below a place,
never in the place's own directory. Its position comes from the nearest `place.yaml` above; after
a `mv` under another place, `egs zone ls` reports `frame_mismatch`: `egs zone rm` it and add it
again. Give the points in the robot's current frame (`egs zone add --rect` / `--polygon`); egs
converts them, and refuses while the robot is not yet aligned to the base map. At most 100 m
wide or tall (`too_large`).

## observations.jsonl

One JSON object per line, appended, never rewritten:

```json
{"recorded_at": 1756300000123456789, "source": "agent", "result": "seen", "note": "left of the sink"}
{"recorded_at": 1756300900000000000, "source": "agent", "invalidates": 1756300000123456789}
```

- `recorded_at`: Unix ns, strictly increasing within a file; also the line's id.
- `source`: `agent` or `human`.
- `result`: `seen` (it is there), `not_observed` (could not check: occluded, not looked),
  `absent` (checked, not there). One `not_observed` is not `absent`.
- `invalidates`: retracts an earlier line by its `recorded_at`; nothing is ever deleted.
- `note`: optional free text.
- `attachment`: optional, `attachments/<id>.<ext>` relative to this directory (`egs observe --attach`).

## Where things are: precision

`egs where <path>` walks up from `<path>` to the nearest directory A with a `place.yaml`, and
answers with a position, `approach` (A's robot pose: the one to navigate to) and a precision:

| precision | meaning |
|---|---|
| `own` | `<path>` has its own `place.yaml` |
| `offset` | `<path>`'s own `node.yaml` has an offset from A |
| `inherited` | position of A, or of an offset set on a directory between A and `<path>`; not observed |
| `none` | no ancestor has a `place.yaml`: no position; bound children one level down are listed |

An orphaned binding (its part of the map was deleted) resolves to nothing, not to an old
position: go there and `egs place save` again.

State `pending`: navigable now; its coordinates can move when the map re-optimizes, ask `egs where`
again before each trip. `frozen`: fixed for good. `rebound`: pending again after a re-save.

## Places as workspaces

Any directory, place or container, may hold:

- `notes.md`: free memory about this place or thing.
- `SKILL.md`: the index for this place: what to know here, which skills exist. Frontmatter
  `name`, `description`, `metadata.author: human|agent`, `metadata.verified: <date>|never`
  (new agent files say `never`); body at most 40 lines.
- `skills/<name>/SKILL.md`: a skill for this place, Agent Skills format (`name` = directory name,
  same frontmatter as above), optionally with `scripts/` for waypoints and `egs` calls.

Scope is the ancestor chain: for `places/kitchen/fridge` read `places/SKILL.md`, then
`places/kitchen/SKILL.md`, then `places/kitchen/fridge/SKILL.md`. Skills under any of those
directories apply. On conflict the closest wins, and the user's direct instruction beats them all.
Find them with `cat` and `find places -name SKILL.md`; `egs` does not load them for you.

A skill refers only to its own directory (`./...`), never to another place's path or anchor id:
paths break on `mv`, while a skill nested in its place moves with it. A skill that spans places
lives at their common ancestor and takes the place as an argument.
