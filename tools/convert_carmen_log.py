#!/usr/bin/env python3
"""CARMEN log (.clf/.log) to rosbag2, for the classic 2D SLAM datasets.

FLASER lines become sensor_msgs/LaserScan on /scan (frame_id "laser") and their odometry pose
fields become nav_msgs/Odometry on /odom. Angles follow the CARMEN convention: the beams span
180 degrees starting at -90 degrees relative to the laser's heading, one beam every
pi/num_readings.

CARMEN logs carry one timestamp per scan and nothing per beam, so time_increment and scan_time
are zero and the pipeline's undistortion degenerates to a no-op. That is acceptable for these
old 180-degree scanners; it would not be for a fast-spinning modern lidar.

The log's robot_frontlaser_offset PARAM is reported at the end: pass it to the consumer as
--base_from_laser <offset>,0,0 (the frames coincide when it is 0, which intel and aces have).

    pip install rosbags
    python3 tools/convert_carmen_log.py intel.clf bags/intel
"""

import argparse
import math
import sys
from pathlib import Path

import numpy
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

ROS2 = get_typestore(Stores.LATEST)

# SICK no-return readings sit at the range cap (81.92 or 80.99 m); anything indoors is far
# below. Everything at or above this is dropped by the consumer's range_max.
RANGE_MAX = 79.9


def stamp_of(timestamp):
    Time = ROS2.types["builtin_interfaces/msg/Time"]
    sec = int(timestamp)
    return Time(sec=sec, nanosec=int(round((timestamp - sec) * 1e9)))


def header(timestamp, frame_id):
    Header = ROS2.types["std_msgs/msg/Header"]
    return Header(stamp=stamp_of(timestamp), frame_id=frame_id)


def laser_scan(timestamp, ranges):
    num = len(ranges)
    increment = math.pi / num
    return ROS2.types["sensor_msgs/msg/LaserScan"](
        header=header(timestamp, "laser"),
        angle_min=-math.pi / 2.0,
        angle_max=-math.pi / 2.0 + (num - 1) * increment,
        angle_increment=increment,
        time_increment=0.0,
        scan_time=0.0,
        range_min=0.001,
        range_max=RANGE_MAX,
        ranges=numpy.asarray(ranges, dtype=numpy.float32),
        intensities=numpy.zeros(0, dtype=numpy.float32),
    )


def odometry(timestamp, x, y, theta):
    types = ROS2.types
    zero3 = types["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)
    return types["nav_msgs/msg/Odometry"](
        header=header(timestamp, "odom"),
        child_frame_id="base_link",
        pose=types["geometry_msgs/msg/PoseWithCovariance"](
            pose=types["geometry_msgs/msg/Pose"](
                position=types["geometry_msgs/msg/Point"](x=x, y=y, z=0.0),
                orientation=types["geometry_msgs/msg/Quaternion"](
                    x=0.0, y=0.0, z=math.sin(theta / 2.0), w=math.cos(theta / 2.0)
                ),
            ),
            covariance=numpy.zeros(36),
        ),
        twist=types["geometry_msgs/msg/TwistWithCovariance"](
            twist=types["geometry_msgs/msg/Twist"](linear=zero3, angular=zero3),
            covariance=numpy.zeros(36),
        ),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("src", type=Path, help="CARMEN log file (.clf/.log)")
    parser.add_argument("dst", type=Path, help="output rosbag2 directory (must not exist)")
    parser.add_argument(
        "--min_interval",
        type=float,
        default=0.05,
        help="drop scans logged closer than this to the previous one (burst flushes), seconds",
    )
    args = parser.parse_args()

    records = []
    num_unsorted = 0
    num_duplicates = 0
    front_laser_offset = 0.0
    with open(args.src) as log:
        previous = float("-inf")
        for line in log:
            fields = line.split()
            if not fields:
                continue
            if fields[0] == "PARAM" and len(fields) >= 3 and fields[1] == "robot_frontlaser_offset":
                front_laser_offset = float(fields[2])
            if fields[0] != "FLASER":
                continue
            num = int(fields[1])
            if len(fields) < 2 + num + 6 + 1:
                print(f"short FLASER line skipped: {line[:60]}...", file=sys.stderr)
                continue
            ranges = [float(value) for value in fields[2 : 2 + num]]
            odom = tuple(float(value) for value in fields[2 + num + 3 : 2 + num + 6])
            timestamp = float(fields[2 + num + 6])
            if timestamp < previous:
                num_unsorted += 1
            previous = timestamp
            records.append((timestamp, ranges, odom))

    # These logs carry logging-time, not acquisition-time: some (intel) are not even ordered,
    # and a third of the lines arrive in sub-20 ms bursts that a 5 Hz scanner cannot have
    # produced. An out-of-order or burst timestamp is poison downstream -- the motion filter
    # reads a metres-per-second velocity out of two scans logged 11 ms apart and the prediction
    # runs away. Sort, then drop anything closer than min_interval to its predecessor.
    records.sort(key=lambda record: record[0])
    num_scans = 0
    with Writer(args.dst, version=8) as writer:
        scan_conn = writer.add_connection("/scan", "sensor_msgs/msg/LaserScan", typestore=ROS2)
        odom_conn = writer.add_connection("/odom", "nav_msgs/msg/Odometry", typestore=ROS2)
        previous = float("-inf")
        for timestamp, ranges, odom in records:
            if timestamp - previous < args.min_interval:
                num_duplicates += 1
                continue
            previous = timestamp
            nanoseconds = int(round(timestamp * 1e9))
            writer.write(
                scan_conn, nanoseconds, ROS2.serialize_cdr(laser_scan(timestamp, ranges), scan_conn.msgtype)
            )
            writer.write(
                odom_conn, nanoseconds, ROS2.serialize_cdr(odometry(timestamp, *odom), odom_conn.msgtype)
            )
            num_scans += 1

    print(f"/scan and /odom: {num_scans}")
    if num_unsorted or num_duplicates:
        print(
            f"reordered {num_unsorted} out-of-order scans, "
            f"dropped {num_duplicates} within {args.min_interval}s of their predecessor"
        )
    print(f"robot_frontlaser_offset: {front_laser_offset}")
    if front_laser_offset != 0.0:
        print(f"pass --base_from_laser {front_laser_offset},0,0 to the consumer")
    if num_scans == 0:
        sys.exit("nothing converted")


if __name__ == "__main__":
    main()
