# Build/dev image for EvergreenSLAM on ROS 2 Jazzy (Ubuntu 24.04).
#
# Jazzy rather than Humble: core/CMakeLists.txt links the namespaced CMake
# target yaml-cpp::yaml-cpp, which only exists from yaml-cpp 0.8 (24.04).
# 22.04 ships 0.7 and exports the unnamespaced `yaml-cpp` target only.
#
# Build (arm64 native on Apple Silicon, do not pass --platform):
#   docker build -t evergreenslam:ros2 adapters/ros2/evergreenslam_ros
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# ros-base already provides eigen, yaml-cpp, gtest, rclcpp, sensor_msgs,
# nav_msgs, tf2_ros, rosbag2_cpp and the sqlite3 + mcap storage plugins.
# rviz_common/rviz_rendering are here only so the evergreenslam_rviz plugin
# (adapters/ros2/evergreenslam_rviz) gets the same gcc gate as the rest; rviz2 itself is not installed.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      ninja-build \
      git \
      libceres-dev \
      libeigen3-dev \
      libgoogle-glog-dev \
      libyaml-cpp-dev \
      libgtest-dev \
      libgmock-dev \
      libprotobuf-dev \
      protobuf-compiler \
      ros-jazzy-rclcpp \
      ros-jazzy-sensor-msgs \
      ros-jazzy-nav-msgs \
      ros-jazzy-tf2-ros \
      ros-jazzy-rosbag2-cpp \
      ros-jazzy-rosbag2-storage-sqlite3 \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-visualization-msgs \
      ros-jazzy-rosidl-default-generators \
      ros-jazzy-rviz-common \
      ros-jazzy-rviz-rendering \
      ros-jazzy-rviz-ogre-vendor \
      ros-jazzy-pluginlib \
    && rm -rf /var/lib/apt/lists/*

# Make `bash -lc` (and any interactive shell) see ROS without an explicit
# source, so `docker run ... bash -lc "cmake ..."` works as-is.
RUN printf '%s\n' 'source /opt/ros/jazzy/setup.bash' > /etc/profile.d/ros.sh \
    && printf '%s\n' 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

WORKDIR /ws
