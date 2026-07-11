#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
play_ros2_robotdog_to_ros1.py

读取机器狗 ROS2 rosbag2 sqlite3(.db3)，在线发布为 ROS1 标准话题。

本脚本只做 ROS2 CDR -> ROS1 标准消息的在线发布：
  /front_lidar                    sensor_msgs/PointCloud2
  /front_lidar/imu                sensor_msgs/Imu
  /front_camera/image_compressed  sensor_msgs/CompressedImage

不生成 ROS1 中间 bag，不转换成 Livox CustomMsg，不解压图像。
FAST-LIVO2 的机器狗输入分支负责解析 PointCloud2，CompressedImage 由
LIVMapper 内部解码。
"""

from __future__ import print_function

import argparse
import glob
import os
import sqlite3
import struct
import sys
import time

import rospy
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import CompressedImage, Imu, PointCloud2, PointField
from std_msgs.msg import Header


TOPIC_MAP = {
    "/front_lidar": PointCloud2,
    "/front_lidar/imu": Imu,
    "/front_camera/image_compressed": CompressedImage,
}


class CDRReader(object):
    """最小 CDR 读取器，仅解析机器狗 bag 中用到的标准消息字段。"""

    def __init__(self, data):
        # ROS2 sqlite 中 data 是 CDR 字节流，前 4 字节是封装头。
        self.data = bytes(data)
        self.base = 4
        self.pos = 4

    def align(self, size):
        # CDR 字段对齐从封装头之后的 payload 起点计算，而不是从整个 blob 起点计算。
        rem = (self.pos - self.base) % size
        if rem:
            self.pos += size - rem

    def read_u8(self):
        self.align(1)
        val = struct.unpack_from("<B", self.data, self.pos)[0]
        self.pos += 1
        return val

    def read_bool(self):
        return bool(self.read_u8())

    def read_i32(self):
        self.align(4)
        val = struct.unpack_from("<i", self.data, self.pos)[0]
        self.pos += 4
        return val

    def read_u32(self):
        self.align(4)
        val = struct.unpack_from("<I", self.data, self.pos)[0]
        self.pos += 4
        return val

    def read_f64(self):
        self.align(8)
        val = struct.unpack_from("<d", self.data, self.pos)[0]
        self.pos += 8
        return val

    def read_string(self):
        self.align(4)
        length = self.read_u32()
        raw = self.data[self.pos:self.pos + length]
        self.pos += length
        if raw.endswith(b"\x00"):
            raw = raw[:-1]
        return raw.decode("utf-8", "replace")

    def read_u8_sequence(self):
        self.align(4)
        length = self.read_u32()
        raw = self.data[self.pos:self.pos + length]
        self.pos += length
        return raw


def make_header(reader):
    sec = reader.read_i32()
    nsec = reader.read_u32()
    frame_id = reader.read_string()
    header = Header()
    header.stamp = rospy.Time(sec, nsec)
    header.frame_id = frame_id
    return header


def parse_pointcloud2(data):
    reader = CDRReader(data)
    msg = PointCloud2()
    msg.header = make_header(reader)
    msg.height = reader.read_u32()
    msg.width = reader.read_u32()

    field_count = reader.read_u32()
    msg.fields = []
    for _ in range(field_count):
        field = PointField()
        field.name = reader.read_string()
        field.offset = reader.read_u32()
        field.datatype = reader.read_u8()
        field.count = reader.read_u32()
        msg.fields.append(field)

    msg.is_bigendian = reader.read_bool()
    msg.point_step = reader.read_u32()
    msg.row_step = reader.read_u32()
    # 保留 PointCloud2 原始二进制点数据，字段由 FAST-LIVO2 的 ROBOTDOG 分支解析。
    msg.data = reader.read_u8_sequence()
    msg.is_dense = reader.read_bool()
    return msg


def parse_imu(data):
    reader = CDRReader(data)
    msg = Imu()
    msg.header = make_header(reader)

    msg.orientation = Quaternion(
        x=reader.read_f64(),
        y=reader.read_f64(),
        z=reader.read_f64(),
        w=reader.read_f64(),
    )
    msg.orientation_covariance = [reader.read_f64() for _ in range(9)]

    msg.angular_velocity = Vector3(
        x=reader.read_f64(),
        y=reader.read_f64(),
        z=reader.read_f64(),
    )
    msg.angular_velocity_covariance = [reader.read_f64() for _ in range(9)]

    msg.linear_acceleration = Vector3(
        x=reader.read_f64(),
        y=reader.read_f64(),
        z=reader.read_f64(),
    )
    msg.linear_acceleration_covariance = [reader.read_f64() for _ in range(9)]
    return msg


def parse_compressed_image(data):
    reader = CDRReader(data)
    msg = CompressedImage()
    msg.header = make_header(reader)
    msg.format = reader.read_string()
    # 保留 JPEG 压缩字节，FAST-LIVO2 内部 CompressedImage 回调负责解码。
    msg.data = reader.read_u8_sequence()
    return msg


PARSERS = {
    "/front_lidar": parse_pointcloud2,
    "/front_lidar/imu": parse_imu,
    "/front_camera/image_compressed": parse_compressed_image,
}


def resolve_db3_path(input_path):
    if os.path.isfile(input_path):
        return input_path
    if not os.path.isdir(input_path):
        raise RuntimeError("输入路径不存在: {}".format(input_path))

    db3_files = sorted(glob.glob(os.path.join(input_path, "*.db3")))
    if not db3_files:
        raise RuntimeError("输入目录中没有 .db3 文件: {}".format(input_path))
    if len(db3_files) > 1:
        rospy.logwarn("发现多个 .db3 文件，默认使用第一个: %s", db3_files[0])
    return db3_files[0]


def build_topic_table(conn, skip_images):
    rows = conn.execute("select id, name, type from topics").fetchall()
    topic_info = {}
    for topic_id, name, msg_type in rows:
        if name not in TOPIC_MAP:
            continue
        if skip_images and name == "/front_camera/image_compressed":
            continue
        topic_info[topic_id] = (name, msg_type)
    return topic_info


def validate_topics(topic_info):
    names = set(name for name, _ in topic_info.values())
    required = set(TOPIC_MAP.keys())
    missing = sorted(required - names)
    if missing:
        rospy.logwarn("以下话题未在 bag 中启用或不存在: %s", ", ".join(missing))


def create_publishers(topic_info, queue_size):
    publishers = {}
    for _, (topic, _) in topic_info.items():
        publishers[topic] = rospy.Publisher(topic, TOPIC_MAP[topic], queue_size=queue_size)
    return publishers


def wait_for_subscribers(publishers):
    rospy.loginfo("等待 FAST-LIVO2 订阅者连接...")
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        connected = True
        for pub in publishers.values():
            if pub.get_num_connections() == 0:
                connected = False
                break
        if connected:
            rospy.loginfo("订阅者已连接，开始发布。")
            return
        rate.sleep()


def play(args):
    db3_path = resolve_db3_path(args.input)
    conn = sqlite3.connect(db3_path)
    topic_info = build_topic_table(conn, args.skip_images)
    validate_topics(topic_info)
    if not topic_info:
        raise RuntimeError("没有可发布的话题，请检查 bag 内容。")

    rospy.init_node("robotdog_ros2bag_standard_player", anonymous=True)
    publishers = create_publishers(topic_info, args.queue_size)

    if args.wait_subscribers:
        wait_for_subscribers(publishers)
    else:
        # 给 ROS publisher 一点注册时间，避免开头几帧在订阅建立前丢失。
        rospy.sleep(1.0)

    topic_ids = sorted(topic_info.keys())
    placeholders = ",".join(["?"] * len(topic_ids))
    min_ts = conn.execute(
        "select min(timestamp) from messages where topic_id in ({})".format(placeholders),
        topic_ids,
    ).fetchone()[0]
    if min_ts is None:
        raise RuntimeError("bag 中没有可发布消息。")

    start_ns = min_ts + int(args.start_offset * 1e9)
    end_ns = None
    if args.duration > 0:
        end_ns = start_ns + int(args.duration * 1e9)

    query = (
        "select topic_id, timestamp, data from messages "
        "where topic_id in ({}) and timestamp >= ? ".format(placeholders)
    )
    params = list(topic_ids) + [start_ns]
    if end_ns is not None:
        query += "and timestamp <= ? "
        params.append(end_ns)
    query += "order by timestamp"

    stats = dict((topic, 0) for topic in TOPIC_MAP.keys())
    first_bag_ts = None
    first_wall = None
    last_report = time.time()

    rospy.loginfo("开始在线发布 ROS2 bag: %s", db3_path)
    rospy.loginfo("发布话题: %s", ", ".join(sorted(publishers.keys())))

    for topic_id, bag_ts, raw in conn.execute(query, params):
        if rospy.is_shutdown():
            break

        if first_bag_ts is None:
            first_bag_ts = bag_ts
            first_wall = time.time()

        if args.rate > 0:
            target_elapsed = (bag_ts - first_bag_ts) / 1e9 / args.rate
            sleep_time = target_elapsed - (time.time() - first_wall)
            if sleep_time > 0:
                rospy.sleep(sleep_time)

        topic, _ = topic_info[topic_id]
        try:
            msg = PARSERS[topic](raw)
        except Exception as exc:
            rospy.logwarn("解析失败 topic=%s ts=%d: %s", topic, bag_ts, exc)
            continue

        publishers[topic].publish(msg)
        stats[topic] = stats.get(topic, 0) + 1

        now = time.time()
        if args.stats_interval > 0 and now - last_report >= args.stats_interval:
            rospy.loginfo(
                "已发布 LiDAR=%d IMU=%d Image=%d",
                stats.get("/front_lidar", 0),
                stats.get("/front_lidar/imu", 0),
                stats.get("/front_camera/image_compressed", 0),
            )
            last_report = now

    rospy.loginfo(
        "发布完成: LiDAR=%d IMU=%d Image=%d",
        stats.get("/front_lidar", 0),
        stats.get("/front_lidar/imu", 0),
        stats.get("/front_camera/image_compressed", 0),
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(description="机器狗 ROS2 db3 -> ROS1 标准话题在线发布器")
    parser.add_argument("input", help="ROS2 bag 目录或 .db3 文件路径")
    parser.add_argument("--rate", type=float, default=1.0, help="回放倍率，默认 1.0；<=0 表示尽快发布")
    parser.add_argument("--start-offset", type=float, default=0.0, help="从 bag 起点后多少秒开始发布")
    parser.add_argument("--duration", type=float, default=-1.0, help="发布时长，默认发布到结尾")
    parser.add_argument("--queue-size", type=int, default=1000, help="ROS1 publisher 队列长度")
    parser.add_argument("--skip-images", action="store_true", help="只发布 LiDAR 和 IMU，跳过压缩图像")
    parser.add_argument("--wait-subscribers", action="store_true", help="等待 FAST-LIVO2 订阅者连接后再发布")
    parser.add_argument("--stats-interval", type=float, default=5.0, help="统计输出间隔，<=0 关闭")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    try:
        play(args)
    except Exception as exc:
        print("错误: {}".format(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
