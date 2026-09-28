# EvergreenSLAM

EvergreenSLAM is a lightweight, always-on 2D LiDAR SLAM framework for embodied agents and DIY robots.

Instead of separating mapping and localization, it continuously builds, localizes, freezes well-constrained submaps, and expands into unknown areas when needed.

Its design philosophy is simple:

- **Always-on**
- **Zero-touch**
- **Freeze only when stable**
- **Grow from trusted maps**
- **Simple by design**
- **Agent-ready**

The goal is to make SLAM a background spatial service rather than a manually operated mapping tool.

## Build

```bash
cmake -S core -B build -DEVERGREENSLAM_BUILD_TESTS=ON && cmake --build build -j8 && ctest --test-dir build
```

Dependencies: Eigen, Ceres, glog, yaml-cpp, protobuf (library and `protoc`), OpenMP, gtest.
`core/` does not depend on ROS.

```bash
# macOS
brew install eigen ceres-solver glog yaml-cpp protobuf libomp googletest

# Debian / Ubuntu
sudo apt install libeigen3-dev libceres-dev libgoogle-glog-dev libyaml-cpp-dev \
                 libprotobuf-dev protobuf-compiler libgtest-dev
```

Alternatively, let [pixi](https://pixi.sh) fetch everything, ROS 2 Jazzy and rviz2 included, into `.pixi/`
(macOS Apple Silicon and Linux; see the header of `pixi.toml` for the full task list):

```bash
pixi run core-test                      # core alone, no ROS
pixi run -e ros ros-test                # ROS 2 adapter
pixi run -e ros bag bags/wg_cafe        # offline replay, webui at http://localhost:8642
pixi run -e ros rviz
```

### ROS 2

The repo doubles as a colcon workspace source (ROS 2 Jazzy): clone it into any `ws/src`, then
`rosdep install --from-paths src --ignore-src -y && colcon build`. It builds three packages, the
live node `evergreenslam_node` and the offline bag tool `evergreenslam_bag` among them; see
[`adapters/ros2/evergreenslam_ros/README.md`](adapters/ros2/evergreenslam_ros/README.md#build).
