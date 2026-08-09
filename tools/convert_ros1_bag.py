#!/usr/bin/env python3
"""ROS 1 bag to rosbag2, with the fixes ~2010 bags need and rosbags-convert lacks.

Topics gain the leading '/' ROS 2 requires, frame_ids lose the leading '/' tf2 rejects, and
messages are decoded by their type name against a modern typestore, sidestepping embedded
definitions that reference the long-gone roslib/Header. Only the types this project consumes
are converted: LaserScan, tf (old tf/tfMessage included), and Odometry.

    pip install rosbags
    python3 tools/convert_ros1_bag.py wg-cafe.bag bags/wg_cafe
"""

import argparse
import sys
from collections import Counter
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

ROS1 = get_typestore(Stores.ROS1_NOETIC)
ROS2 = get_typestore(Stores.LATEST)
ROS1.register(
    get_types_from_msg("geometry_msgs/TransformStamped[] transforms", "tf2_msgs/msg/TFMessage")
)

# Old tf/tfMessage has the identical wire layout, so it decodes as the modern type.
DECODE_AS = {
    "tf/msg/tfMessage": "tf2_msgs/msg/TFMessage",
    "tf2_msgs/msg/TFMessage": "tf2_msgs/msg/TFMessage",
    "sensor_msgs/msg/LaserScan": "sensor_msgs/msg/LaserScan",
    "nav_msgs/msg/Odometry": "nav_msgs/msg/Odometry",
}


def header2(header1):
    Time = ROS2.types["builtin_interfaces/msg/Time"]
    Header = ROS2.types["std_msgs/msg/Header"]
    return Header(
        stamp=Time(sec=header1.stamp.sec, nanosec=header1.stamp.nanosec),
        frame_id=header1.frame_id.lstrip("/"),
    )


def convert_laser_scan(msg):
    return ROS2.types["sensor_msgs/msg/LaserScan"](
        header=header2(msg.header),
        angle_min=msg.angle_min,
        angle_max=msg.angle_max,
        angle_increment=msg.angle_increment,
        time_increment=msg.time_increment,
        scan_time=msg.scan_time,
        range_min=msg.range_min,
        range_max=msg.range_max,
        ranges=msg.ranges,
        intensities=msg.intensities,
    )


def convert_tf(msg):
    types = ROS2.types
    transforms = [
        types["geometry_msgs/msg/TransformStamped"](
            header=header2(t.header),
            child_frame_id=t.child_frame_id.lstrip("/"),
            transform=types["geometry_msgs/msg/Transform"](
                translation=types["geometry_msgs/msg/Vector3"](
                    x=t.transform.translation.x,
                    y=t.transform.translation.y,
                    z=t.transform.translation.z,
                ),
                rotation=types["geometry_msgs/msg/Quaternion"](
                    x=t.transform.rotation.x,
                    y=t.transform.rotation.y,
                    z=t.transform.rotation.z,
                    w=t.transform.rotation.w,
                ),
            ),
        )
        for t in msg.transforms
    ]
    return types["tf2_msgs/msg/TFMessage"](transforms=transforms)


def convert_odometry(msg):
    types = ROS2.types
    return types["nav_msgs/msg/Odometry"](
        header=header2(msg.header),
        child_frame_id=msg.child_frame_id.lstrip("/"),
        pose=types["geometry_msgs/msg/PoseWithCovariance"](
            pose=types["geometry_msgs/msg/Pose"](
                position=types["geometry_msgs/msg/Point"](
                    x=msg.pose.pose.position.x,
                    y=msg.pose.pose.position.y,
                    z=msg.pose.pose.position.z,
                ),
                orientation=types["geometry_msgs/msg/Quaternion"](
                    x=msg.pose.pose.orientation.x,
                    y=msg.pose.pose.orientation.y,
                    z=msg.pose.pose.orientation.z,
                    w=msg.pose.pose.orientation.w,
                ),
            ),
            covariance=msg.pose.covariance,
        ),
        twist=types["geometry_msgs/msg/TwistWithCovariance"](
            twist=types["geometry_msgs/msg/Twist"](
                linear=types["geometry_msgs/msg/Vector3"](
                    x=msg.twist.twist.linear.x,
                    y=msg.twist.twist.linear.y,
                    z=msg.twist.twist.linear.z,
                ),
                angular=types["geometry_msgs/msg/Vector3"](
                    x=msg.twist.twist.angular.x,
                    y=msg.twist.twist.angular.y,
                    z=msg.twist.twist.angular.z,
                ),
            ),
            covariance=msg.twist.covariance,
        ),
    )


CONVERTERS = {
    "tf2_msgs/msg/TFMessage": convert_tf,
    "sensor_msgs/msg/LaserScan": convert_laser_scan,
    "nav_msgs/msg/Odometry": convert_odometry,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("src", type=Path, help="ROS 1 .bag file")
    parser.add_argument("dst", type=Path, help="output rosbag2 directory (must not exist)")
    args = parser.parse_args()

    written = Counter()
    skipped = Counter()
    with Reader(args.src) as reader, Writer(args.dst, version=8) as writer:
        connections = {}
        for src_conn in reader.connections:
            msgtype = DECODE_AS.get(src_conn.msgtype)
            if msgtype is None:
                skipped[f"{src_conn.topic} ({src_conn.msgtype})"] = src_conn.msgcount
                continue
            topic = src_conn.topic if src_conn.topic.startswith("/") else "/" + src_conn.topic
            connections[src_conn.id] = (
                writer.add_connection(topic, msgtype, typestore=ROS2),
                msgtype,
            )
        for src_conn, timestamp, raw in reader.messages():
            if src_conn.id not in connections:
                continue
            dst_conn, msgtype = connections[src_conn.id]
            msg = CONVERTERS[msgtype](ROS1.deserialize_ros1(raw, msgtype))
            writer.write(dst_conn, timestamp, ROS2.serialize_cdr(msg, msgtype))
            written[dst_conn.topic] += 1

    for topic, count in sorted(written.items()):
        print(f"{topic}: {count}")
    for topic, count in sorted(skipped.items()):
        print(f"skipped {topic}: {count}", file=sys.stderr)
    if not written:
        sys.exit("nothing converted")


if __name__ == "__main__":
    main()
