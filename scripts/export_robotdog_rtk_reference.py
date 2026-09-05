#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Export a raw or quality-gated RobotDog RTK reference trajectory as CSV and PCD.

This parses recorded ``/rtk_pvh`` CDR using the supplied UniRtkPvh,
UniHeading, and UniBestNav schemas. In ``quality`` mode it applies the exact
quality gate in the 2026-08-15 session snapshot's rtk_map_proxy.py; in ``raw``
mode it exports every finite RTK position. It intentionally does not calculate
absolute ATE/RMSE; output is for map-overlay/reference visualization.
"""

import argparse
import csv
import math
import sqlite3
import struct
import sys
from pathlib import Path


# From the 2026-08-15 session snapshot rtk_map_proxy.py.
EARTH_RADIUS_M = 6378137.0
ANCHOR_LAT_DEG = 39.9883760587
ANCHOR_LON_DEG = 116.3506190002
ANCHOR_ALT_M = 46.3658621842
T00, T01, TX = -0.97192701, -0.23394911, 0.41201010
T10, T11, TY = 0.23394911, -0.97192701, -1.85952049
TZ = -4.92500000
HEADING_SHIFT_DEG = -166.466
TOPIC = "/rtk_pvh"
MESSAGE_TYPE = "robots_dog_msgs/msg/UniRtkPvh"


class CdrReader(object):
    """Minimal little-endian CDR reader for the supplied RobotDog RTK schemas."""

    def __init__(self, payload):
        self.data = bytes(payload)
        if len(self.data) < 4 or self.data[:2] != b"\x00\x01":
            raise ValueError("不是预期的 ROS2 小端 CDR 数据")
        self.base = 4
        self.pos = 4

    def align(self, size):
        self.pos += (-(self.pos - self.base)) % size

    def read(self, fmt, size):
        self.align(size)
        if self.pos + size > len(self.data):
            raise ValueError("CDR 数据在偏移 %d 截断" % self.pos)
        value = struct.unpack_from(fmt, self.data, self.pos)[0]
        self.pos += size
        return value

    def int32(self):
        return self.read("<i", 4)

    def uint32(self):
        return self.read("<I", 4)

    def uint8(self):
        return self.read("<B", 1)

    def float32(self):
        return self.read("<f", 4)

    def float64(self):
        return self.read("<d", 8)

    def header(self):
        sec = self.int32()
        nanosec = self.uint32()
        length = self.uint32()
        if self.pos + length > len(self.data):
            raise ValueError("Header frame_id 字符串截断")
        self.pos += length
        return sec, nanosec


def parse_heading(reader):
    # UniHeading.msg
    reader.header()
    reader.float64()  # utc_time_s
    reader.uint8()    # sol_status
    heading_type = reader.uint8()
    reader.float32()  # base_line
    heading_deg = reader.float32()
    reader.float32()  # pitch_deg
    heading_std = reader.float32()
    reader.float32()  # pitch_std
    reader.uint8()    # svs_num
    reader.uint8()    # soln_svs_num
    return heading_type, heading_deg, heading_std


def parse_best_nav(reader):
    # UniBestNav.msg
    reader.header()
    reader.float64()  # utc_time_s
    p_sol_status = reader.uint8()
    pos_type = reader.uint8()
    latitude_deg = reader.float64()
    longitude_deg = reader.float64()
    altitude_m = reader.float64()
    reader.float32()  # undulation
    lat_std = reader.float32()
    lon_std = reader.float32()
    hgt_std = reader.float32()
    diff_age_s = reader.float32()
    reader.float32()  # sol_age_s
    reader.uint8()    # svs_num
    reader.uint8()    # soln_svs_num
    reader.uint8()    # v_sol_status
    reader.uint8()    # vel_type
    reader.float64()  # hor_spd
    reader.float64()  # trk_gnd
    reader.float64()  # ver_spd
    reader.float32()  # ver_spd_std
    reader.float32()  # hor_spd_std
    return {
        "p_sol_status": p_sol_status, "pos_type": pos_type,
        "latitude_deg": latitude_deg, "longitude_deg": longitude_deg,
        "altitude_m": altitude_m, "lat_std": lat_std, "lon_std": lon_std,
        "hgt_std": hgt_std, "diff_age_s": diff_age_s,
    }


def parse_pvh(payload):
    reader = CdrReader(payload)
    stamp_sec, stamp_nanosec = reader.header()
    heading_type, heading_deg, heading_std = parse_heading(reader)
    nav = parse_best_nav(reader)
    if reader.pos != len(reader.data):
        raise ValueError("UniRtkPvh CDR 有 %d 个未解析字节" % (len(reader.data) - reader.pos))
    nav.update({"stamp_sec": stamp_sec, "stamp_nanosec": stamp_nanosec,
                "heading_type": heading_type, "heading_deg": heading_deg,
                "heading_std": heading_std})
    return nav


def quality_pass(item):
    """Exactly the init_compatible gate in the supplied rtk_map_proxy.py."""
    values = (item["latitude_deg"], item["longitude_deg"], item["altitude_m"],
              item["lat_std"], item["lon_std"], item["hgt_std"],
              item["diff_age_s"], item["heading_std"])
    return (all(math.isfinite(value) for value in values)
            and item["p_sol_status"] == 0 and item["pos_type"] >= 34
            and item["lat_std"] < 0.5 and item["lon_std"] < 0.5
            and item["hgt_std"] < 1.0 and item["diff_age_s"] < 5.0
            and item["heading_type"] >= 40 and item["heading_std"] < 5.0)


def map_coordinates(item):
    latitude0_rad = math.radians(ANCHOR_LAT_DEG)
    east = math.radians(item["longitude_deg"] - ANCHOR_LON_DEG) * EARTH_RADIUS_M * math.cos(latitude0_rad)
    north = math.radians(item["latitude_deg"] - ANCHOR_LAT_DEG) * EARTH_RADIUS_M
    up = item["altitude_m"] - ANCHOR_ALT_M
    return T00 * east + T01 * north + TX, T10 * east + T11 * north + TY, up + TZ


def map_yaw_deg(heading_deg):
    return (90.0 - ((heading_deg + HEADING_SHIFT_DEG) % 360.0)) % 360.0


def open_topic(database):
    connection = sqlite3.connect(str(database))
    row = connection.execute("SELECT id, type, serialization_format FROM topics WHERE name = ?", (TOPIC,)).fetchone()
    if row is None:
        connection.close()
        raise RuntimeError("袋文件中没有 %s" % TOPIC)
    topic_id, message_type, serialization_format = row
    if message_type != MESSAGE_TYPE or serialization_format != "cdr":
        connection.close()
        raise RuntimeError("%s 类型不符合预期：%s / %s" % (TOPIC, message_type, serialization_format))
    return connection, int(topic_id)


def write_pcd(path, points):
    packed_blue = (120 << 8) | 255
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
              "FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
              "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
              % (len(points), len(points)))
    with path.open("wb") as output:
        output.write(header.encode("ascii"))
        for x, y, z in points:
            output.write(struct.pack("<fffI", x, y, z, packed_blue))


def export_reference(database, output_dir, prefix, mode):
    connection, topic_id = open_topic(database)
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / (prefix + ".csv")
    pcd_path = output_dir / (prefix + "_blue.pcd")
    summary_path = output_dir / (prefix + "_summary.txt")
    fields = ("record_time_ns", "timestamp_sec", "latitude_deg", "longitude_deg", "altitude_m",
              "p_sol_status", "pos_type", "lat_std", "lon_std", "hgt_std", "diff_age_s",
              "heading_type", "heading_deg", "heading_std", "map_x", "map_y", "map_z", "map_yaw_deg")
    total, points = 0, []
    try:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fields)
            writer.writeheader()
            for record_time_ns, payload in connection.execute(
                    "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (topic_id,)):
                total += 1
                item = parse_pvh(payload)
                if mode == "quality" and not quality_pass(item):
                    continue
                position_values = (item["latitude_deg"], item["longitude_deg"], item["altitude_m"])
                if not all(math.isfinite(value) for value in position_values):
                    continue
                map_x, map_y, map_z = map_coordinates(item)
                timestamp = item["stamp_sec"] + item["stamp_nanosec"] * 1e-9
                writer.writerow({
                    "record_time_ns": record_time_ns, "timestamp_sec": "%.9f" % timestamp,
                    "latitude_deg": "%.12f" % item["latitude_deg"], "longitude_deg": "%.12f" % item["longitude_deg"],
                    "altitude_m": "%.6f" % item["altitude_m"], "p_sol_status": item["p_sol_status"],
                    "pos_type": item["pos_type"], "lat_std": "%.6f" % item["lat_std"],
                    "lon_std": "%.6f" % item["lon_std"], "hgt_std": "%.6f" % item["hgt_std"],
                    "diff_age_s": "%.6f" % item["diff_age_s"], "heading_type": item["heading_type"],
                    "heading_deg": "%.6f" % item["heading_deg"], "heading_std": "%.6f" % item["heading_std"],
                    "map_x": "%.6f" % map_x, "map_y": "%.6f" % map_y, "map_z": "%.6f" % map_z,
                    "map_yaw_deg": "%.6f" % map_yaw_deg(item["heading_deg"]),
                })
                points.append((map_x, map_y, map_z))
    finally:
        connection.close()
    if not points:
        raise RuntimeError("没有可导出的 RTK 位置，未生成空轨迹。")
    write_pcd(pcd_path, points)
    if mode == "quality":
        label = "quality-gated RTK reference trajectory"
        filter_description = ("quality_gate: p_sol_status=0; pos_type>=34; lat/lon_std<0.5m; "
                              "hgt_std<1.0m; diff_age_s<5s; heading_type>=40; heading_std<5deg")
    else:
        label = "raw unfiltered RTK reference trajectory"
        filter_description = "quality_gate: none; all finite /rtk_pvh positions were exported"
    summary_path.write_text("\n".join((
        "label: %s (visualization/reference only)" % label,
        "source_database: %s" % database.resolve(), "source_topic: %s" % TOPIC,
        "export_mode: %s" % mode, "total_rtk_pvh: %d" % total,
        "exported_positions: %d" % len(points),
        "export_ratio_percent: %.3f" % (100.0 * len(points) / total),
        "csv: %s" % csv_path.name, "pcd: %s" % pcd_path.name,
        "coordinate_transform: 2026-08-15 snapshot rtk_map_proxy.py", "tz: %.8f" % TZ,
        filter_description,
        "purpose: map-overlay/reference trajectory; do not report absolute ATE/RMSE without lever-arm calibration.",
        "")), encoding="utf-8")
    return total, len(points), csv_path, pcd_path, summary_path


def main():
    parser = argparse.ArgumentParser(description="导出机器狗 RTK 蓝色参考轨迹。")
    parser.add_argument("database", type=Path, help="ROS2 sqlite bag，例如 .../rosbag/rosbag_0.db3")
    parser.add_argument("--output-dir", required=True, type=Path, help="输出目录")
    parser.add_argument("--mode", choices=("raw", "quality"), default="quality",
                        help="raw 导出全部有限位置；quality 使用 RTK 质量门限。默认 quality")
    parser.add_argument("--prefix", default="", help="输出文件名前缀；默认由 --mode 决定")
    args = parser.parse_args()
    if not args.database.is_file():
        parser.error("找不到数据库文件：%s" % args.database)
    try:
        prefix = args.prefix or ("rtk_reference_trajectory" if args.mode == "raw" else "rtk_reference_quality_pass")
        total, exported, csv_path, pcd_path, summary_path = export_reference(
            args.database, args.output_dir, prefix, args.mode)
    except (OSError, RuntimeError, ValueError, sqlite3.Error) as error:
        print("ERROR: %s" % error, file=sys.stderr)
        return 1
    print("Parsed %d /rtk_pvh messages; exported %d %s RTK positions (%.3f%%)." %
          (total, exported, args.mode, 100.0 * exported / total))
    print("CSV: %s" % csv_path)
    print("Blue PCD: %s" % pcd_path)
    print("Summary: %s" % summary_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
