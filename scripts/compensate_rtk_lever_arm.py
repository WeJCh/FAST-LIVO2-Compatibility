#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compensate a map-frame RTK antenna trajectory to a body reference point.

The RobotDog raw RTK CSV contains antenna positions in ``map_x,map_y,map_z``
and a map-frame yaw in degrees.  This tool rotates a configured antenna-to-
target lever arm at every timestamp.  Its default target is ``imu_front``,
which matches FAST-LOCALIZATION's exported ``state_point.pos`` trajectory.

The source CSV is never changed.  The output retains the original fields and
adds the original antenna coordinates so that the reference-point conversion is
fully auditable.
"""

from __future__ import print_function

import argparse
import csv
import math
import struct
import sys
from pathlib import Path


REQUIRED_FIELDS = ("timestamp_sec", "map_x", "map_y", "map_z", "map_yaw_deg")


def parse_finite(value, path, row_number, field):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s 第 %d 行的 %s 不是数值" % (path, row_number, field))
    if not math.isfinite(number):
        raise ValueError("%s 第 %d 行的 %s 不是有限数" % (path, row_number, field))
    return number


def read_antenna_trajectory(path, lever_x, lever_y):
    """Read all rows before writing anything, preventing partial output files."""
    records = []
    with path.open("r", newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames is None:
            raise ValueError("%s 不是带表头的 CSV" % path)
        missing = [field for field in REQUIRED_FIELDS if field not in reader.fieldnames]
        if missing:
            raise ValueError("%s 缺少字段：%s" % (path, ", ".join(missing)))
        input_fields = list(reader.fieldnames)
        for row_number, row in enumerate(reader, start=2):
            timestamp = parse_finite(row["timestamp_sec"], path, row_number, "timestamp_sec")
            antenna_x = parse_finite(row["map_x"], path, row_number, "map_x")
            antenna_y = parse_finite(row["map_y"], path, row_number, "map_y")
            antenna_z = parse_finite(row["map_z"], path, row_number, "map_z")
            yaw_deg = parse_finite(row["map_yaw_deg"], path, row_number, "map_yaw_deg")
            yaw_rad = math.radians(yaw_deg)
            # The lever arm is expressed in the robot body frame: +X forward,
            # +Y left.  map_yaw_deg is map +X, counter-clockwise positive.
            correction_x = lever_x * math.cos(yaw_rad) - lever_y * math.sin(yaw_rad)
            correction_y = lever_x * math.sin(yaw_rad) + lever_y * math.cos(yaw_rad)
            corrected = dict(row)
            # Keep map_x/map_y in the standard evaluator-compatible names.  The
            # original antenna point is retained below under explicit names.
            corrected["map_x"] = "%.9f" % (antenna_x + correction_x)
            corrected["map_y"] = "%.9f" % (antenna_y + correction_y)
            corrected["map_z"] = "%.9f" % antenna_z
            corrected["antenna_map_x"] = "%.9f" % antenna_x
            corrected["antenna_map_y"] = "%.9f" % antenna_y
            corrected["antenna_map_z"] = "%.9f" % antenna_z
            corrected["lever_arm_x_m"] = "%.5f" % lever_x
            corrected["lever_arm_y_m"] = "%.5f" % lever_y
            corrected["xy_correction_x_m"] = "%.9f" % correction_x
            corrected["xy_correction_y_m"] = "%.9f" % correction_y
            corrected["reference_point"] = "imu_front"
            records.append((timestamp, antenna_x + correction_x, antenna_y + correction_y,
                            antenna_z, corrected))
    if not records:
        raise ValueError("%s 没有数据行" % path)
    return input_fields, records


def write_blue_pcd(path, records):
    """Write binary blue PCD; Z is intentionally left unchanged by XY-only correction."""
    packed_blue = (120 << 8) | 255
    header = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
              "FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
              "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
              % (len(records), len(records)))
    with path.open("wb") as output:
        output.write(header.encode("ascii"))
        for _, x, y, z, _ in records:
            output.write(struct.pack("<fffI", x, y, z, packed_blue))


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="将 RTK 天线轨迹按地图航向补偿到 FAST-LOCALIZATION 的 IMU 参考点。")
    parser.add_argument("--input-csv", required=True,
                        help="--mode raw 导出的 rtk_reference_trajectory.csv")
    parser.add_argument("--output-dir", required=True, help="新的补偿结果目录")
    parser.add_argument("--lever-arm-x-m", type=float, default=0.40000,
                        help="天线到目标参考点的机体前向补偿，默认 IMU 的 +0.40000 m")
    parser.add_argument("--lever-arm-y-m", type=float, default=0.0,
                        help="天线到目标参考点的机体左向补偿，默认 0 m")
    parser.add_argument("--prefix", default="rtk_reference_imu_trajectory",
                        help="输出文件名前缀")
    parser.add_argument("--input-quality-gated", action="store_true",
                        help="标记输入 CSV 已按 RTK 质量门限筛选；仅写入摘要，不会再次筛选")
    args = parser.parse_args()
    if not args.prefix or "/" in args.prefix or "\\" in args.prefix:
        parser.error("--prefix 必须是单个文件名前缀")
    return args


def main():
    args = parse_arguments()
    input_path = Path(args.input_csv).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    if not input_path.is_file():
        raise ValueError("找不到输入 CSV：%s" % input_path)

    input_fields, records = read_antenna_trajectory(input_path, args.lever_arm_x_m, args.lever_arm_y_m)
    extra_fields = ("antenna_map_x", "antenna_map_y", "antenna_map_z", "lever_arm_x_m", "lever_arm_y_m",
                    "xy_correction_x_m", "xy_correction_y_m", "reference_point")
    fields = input_fields + [field for field in extra_fields if field not in input_fields]
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / (args.prefix + ".csv")
    pcd_path = output_dir / (args.prefix + "_blue.pcd")
    summary_path = output_dir / (args.prefix + "_summary.txt")

    with csv_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for _, _, _, _, row in records:
            writer.writerow(row)
    write_blue_pcd(pcd_path, records)
    quality_description = ("inherited quality-gated input; no additional filtering was applied"
                           if args.input_quality_gated else
                           "none; every finite input row was retained")
    summary_path.write_text("\n".join((
        "label: RTK antenna trajectory XY-compensated to imu_front",
        "source_csv: %s" % input_path,
        "sample_count: %d" % len(records),
        "reference_point: imu_front",
        "lever_arm_antenna_to_imu_front_body_m: [%.5f, %.5f, 0.00000]" %
        (args.lever_arm_x_m, args.lever_arm_y_m),
        "formula: map_xy_target = map_xy_antenna + R_map_body(map_yaw_deg) * lever_arm_xy",
        "z_handling: unchanged; this is an XY-only correction",
        "quality_filter: %s" % quality_description,
        "csv: %s" % csv_path.name,
        "pcd: %s" % pcd_path.name,
        "")), encoding="utf-8")

    print("RTK lever-arm compensation complete: %s" % output_dir)
    print("Reference point: imu_front; lever arm: [+%.5f, %+.5f] m; samples: %d" %
          (args.lever_arm_x_m, args.lever_arm_y_m, len(records)))
    print("CSV: %s" % csv_path)
    print("Blue PCD: %s" % pcd_path)
    print("Summary: %s" % summary_path)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        print("ERROR: %s" % error, file=sys.stderr)
        sys.exit(2)
