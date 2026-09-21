# rviz

The map as rviz sees it: not one raster, but every submap drawn at the pose the backend last
optimized it to. Overlaps stay visible, so two submaps that disagree show up as a doubled wall
instead of whichever one the compositor happened to write last -- the same x-ray view
Cartographer's plugin gives, ported onto this repo's messages.

`evergreenslam_rviz` is an ament package holding one rviz 2 display,
`evergreenslam_rviz/SubmapsDisplay`. The node side is the adapter's `PoseGraphPublisher`
(`adapters/ros2/evergreenslam_ros`); nothing here talks to `core` beyond one header
of pose helpers.

## What it draws

A `submap_list` message is the index: session id, submap index, version, global pose, finished
and frozen flags. The display keeps one textured quad per entry, and whenever an entry's version
moves it asks `submap_query` for that submap's cells and re-uploads the texture. A submap that
stops being listed is removed, and a version that goes *backwards* means the node restarted, so
everything drawn so far is dropped.

Cells arrive as 0..100 occupancy in percent, 255 unknown, row major with x fastest and cell
(0, 0) at `slice_pose` -- the grid's corner in the submap frame. Unknown is fully transparent;
a known cell is grey by probability (occupied dark, free white) and fades toward transparent as
it approaches even odds, so a cell two rays grazed cannot cover one that was swept properly.
That is the same rule `apps/webui` draws submaps with, and the reason the textures compose
instead of blanking each other.

The texture carries the grey value in R and the confidence in G (Ogre's `loadRawData` will not
take a two channel texture); the fragment shader reads exactly those, and the pass blends with
plain source alpha, not premultiplied.

## Properties

| property | default | what it does |
| --- | --- | --- |
| Topic | `submap_list` | the latched list to subscribe to (transient local, depth 1) |
| Submap query service | `submap_query` | where the textures come from |
| Map frame | `map` | frame the submap poses are in; looked up against rviz's fixed frame |
| Tracking frame | `base_link` | reference for the fade-out |
| Fade-out distance | 1 m | z-distance from the tracking frame before a submap starts fading |
| Submaps > All | on | every session's submaps at once |
| Submaps > All Submap Pose Markers | on | the axes and the `<session>/<index>` label |
| Submaps > Session N | on | one session, with its own pose-marker toggle |
| Submaps > Session N > `<index>.<version>` | on | one individual submap |

All names are relative, so a node namespace or a remap carries through. The fade is Cartographer's
rule kept as is: it measures the z distance between a submap and the tracking frame, and this is a
2D system where both are at z = 0 -- it does nothing unless something publishes a tracking frame
off the ground plane.

## Build and run

One of the three packages of the `adapters/ros2/` colcon workspace: in any `ws/src` checkout,
`colcon build`, `source install/setup.bash`, then

```bash
rviz2 -d $(ros2 pkg prefix evergreenslam_rviz)/share/evergreenslam_rviz/config/evergreenslam.rviz
```

Through pixi, `pixi run -e ros rviz` does the build, the sourcing and that launch. The sourcing
matters on macOS: rmw finds a message package's typesupport with a bare `dlopen`, and only the
`DYLD_LIBRARY_PATH` that colcon's setup script exports lets rviz2 find `evergreenslam_msgs`'
library. In Docker the package builds as part of `ros-build` for the gcc gate only; there is no
rviz2 in the image.

The config opens with the fixed frame `map`, a top-down view, TF, `/scan`, the composed `/map`
behind the submaps, and the `/trajectory_node_list` and `/constraint_list` marker arrays. The
marker and map subscriptions are transient local to match the publisher, which is what lets an
rviz started late still see the current graph.

The shaders and the material script are found through the package share directory at run time, so
the display works only from an install tree, not from the build directory.

## Licensing

This display is a port of `cartographer_rviz` (`submaps_display`, `drawable_submap`, `ogre_slice`
and the ogre_media material and glsl120 shaders), Copyright 2016 The Cartographer Authors,
Apache License 2.0. Each ported file says so, and the copied shader and material files keep the
Cartographer license header.

What changed: one slice per submap instead of the high/low resolution pair, this repo's wire
format for the texture (which puts cell (0, 0) at `slice_pose` with rows ascending, not
Cartographer's opposite corner), sessions instead of trajectories, `std::mutex` instead of absl,
no cartographer or absl dependency at all, and no landmarks, trajectory states or gzip-packed
textures.
