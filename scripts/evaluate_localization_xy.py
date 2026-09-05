#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Evaluate a map-frame localization trajectory against raw RTK XY samples.

The script intentionally ignores Z.  It evaluates only the interval in which
both the localization trajectory and the RTK reference exist.  It is designed
for the RobotDog data exports already produced in FAST-LOCALIZATION/Log.

The resulting values are *XY discrepancies relative to unfiltered RTK*, not
surveyed absolute ATE: no GNSS antenna-to-IMU lever-arm correction is applied.
"""

from __future__ import print_function

import argparse
import bisect
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


def finite_float(value, filename, row_number, field):
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError("%s 第 %d 行的 %s 不是数值" % (filename, row_number, field))
    if not math.isfinite(result):
        raise ValueError("%s 第 %d 行的 %s 不是有限数" % (filename, row_number, field))
    return result


def load_csv(path, required_fields, label):
    """Read a timestamped CSV, rejecting incomplete or invalid records."""
    records = []
    with path.open("r", newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames is None:
            raise ValueError("%s 不是带表头的 CSV" % path)
        missing = [name for name in required_fields if name not in reader.fieldnames]
        if missing:
            raise ValueError("%s 缺少字段：%s" % (path, ", ".join(missing)))
        for row_number, row in enumerate(reader, start=2):
            try:
                timestamp = finite_float(row["timestamp_sec"], path, row_number, "timestamp_sec")
                x = finite_float(row["x" if label == "localization" else "map_x"], path, row_number,
                                 "x" if label == "localization" else "map_x")
                y = finite_float(row["y" if label == "localization" else "map_y"], path, row_number,
                                 "y" if label == "localization" else "map_y")
            except ValueError:
                raise
            records.append((timestamp, x, y))
    if not records:
        raise ValueError("%s 没有有效数据行" % path)
    records.sort(key=lambda item: item[0])
    return deduplicate_timestamps(records, label, path)


def deduplicate_timestamps(records, label, path):
    """Keep the last record for duplicate stamps, preserving safe interpolation."""
    unique = []
    duplicates = 0
    for item in records:
        if unique and item[0] == unique[-1][0]:
            unique[-1] = item
            duplicates += 1
        else:
            unique.append(item)
    if len(unique) < 2:
        raise ValueError("%s 的 %s 时间戳不足两个，无法进行时间匹配" % (path, label))
    if duplicates:
        print("[WARN] %s 中忽略了 %d 个重复的 %s 时间戳" % (path, duplicates, label), file=sys.stderr)
    return unique


def percentile_nearest_rank(values, percent):
    """Return the conventional nearest-rank percentile without dependencies."""
    if not values:
        return float("nan")
    sorted_values = sorted(values)
    index = int(math.ceil(percent / 100.0 * len(sorted_values))) - 1
    index = max(0, min(index, len(sorted_values) - 1))
    return sorted_values[index]


def calculate_metrics(errors):
    if not errors:
        return None
    count = len(errors)
    mae = sum(errors) / count
    rmse = math.sqrt(sum(error * error for error in errors) / count)
    return {
        "count": count,
        "mae_m": mae,
        "rmse_m": rmse,
        "p95_m": percentile_nearest_rank(errors, 95.0),
    }


def calculate_component_metrics(pairs):
    """Calculate signed bias and absolute X/Y component error statistics."""
    dx_values = [item["dx"] for item in pairs]
    dy_values = [item["dy"] for item in pairs]
    abs_dx_values = [abs(value) for value in dx_values]
    abs_dy_values = [abs(value) for value in dy_values]
    dx_metrics = calculate_metrics(abs_dx_values)
    dy_metrics = calculate_metrics(abs_dy_values)
    return {
        "signed_mean_dx_m": sum(dx_values) / len(dx_values),
        "signed_mean_dy_m": sum(dy_values) / len(dy_values),
        "abs_dx_mae_m": dx_metrics["mae_m"],
        "abs_dx_rmse_m": dx_metrics["rmse_m"],
        "abs_dx_p95_m": dx_metrics["p95_m"],
        "abs_dy_mae_m": dy_metrics["mae_m"],
        "abs_dy_rmse_m": dy_metrics["rmse_m"],
        "abs_dy_p95_m": dy_metrics["p95_m"],
    }


def fmt(value, digits=6):
    return ("%%.%df" % digits) % value


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="将红色定位轨迹与未筛选 RTK 的共同时间段进行 XY 平面误差统计。")
    parser.add_argument("--localization-csv", required=True,
                        help="FAST-LOCALIZATION 导出的 localization_trajectory_*.csv")
    parser.add_argument("--rtk-csv", required=True,
                        help="未筛选 RTK 的 rtk_reference_trajectory.csv（使用 map_x/map_y）")
    parser.add_argument("--output-dir", required=True, help="新建或复用的评估结果目录")
    parser.add_argument("--max-rtk-gap-sec", type=float, default=0.5,
                        help="用于插值的 RTK 相邻采样最大允许间隔，默认 0.5 秒")
    parser.add_argument("--time-segment-sec", type=float, default=30.0,
                        help="时间分段长度，默认 30 秒")
    parser.add_argument("--spatial-grid-m", type=float, default=20.0,
                        help="空间网格边长，默认 20 米")
    parser.add_argument("--timestamp-offset-sec", type=float, default=0.0,
                        help="加到 RTK 时间戳的固定偏移，默认 0；未确认偏移时不要修改")
    parser.add_argument("--rtk-reference-label", default="未筛选 RTK",
                        help="写入结果的 RTK 参考标签，例如“质量筛选 RTK”")
    parser.add_argument("--output-prefix", default="xy_error",
                        help="输出文件名前缀，默认 xy_error；用于让多组结果共存于同一目录")
    args = parser.parse_args()
    for name in ("max_rtk_gap_sec", "time_segment_sec", "spatial_grid_m"):
        if getattr(args, name) <= 0:
            parser.error("--%s 必须大于 0" % name.replace("_", "-"))
    if not args.output_prefix or "/" in args.output_prefix or "\\" in args.output_prefix:
        parser.error("--output-prefix 必须是单个文件名前缀")
    return args


def interpolate_rtk(rtk_records, rtk_times, timestamp, max_gap_sec):
    """Return linear XY interpolation, or None if this time cannot be matched."""
    right = bisect.bisect_left(rtk_times, timestamp)
    if right < len(rtk_records) and rtk_records[right][0] == timestamp:
        stamp, x, y = rtk_records[right]
        return x, y, stamp, stamp, 0.0
    if right == 0 or right >= len(rtk_records):
        return None
    before = rtk_records[right - 1]
    after = rtk_records[right]
    gap = after[0] - before[0]
    if gap <= 0 or gap > max_gap_sec:
        return None
    ratio = (timestamp - before[0]) / gap
    return (before[1] + ratio * (after[1] - before[1]),
            before[2] + ratio * (after[2] - before[2]),
            before[0], after[0], ratio)


def write_pairs(path, pairs):
    fields = ("timestamp_sec", "elapsed_from_common_start_sec", "localization_x_m", "localization_y_m",
              "rtk_x_m", "rtk_y_m", "dx_m", "dy_m", "xy_error_m",
              "rtk_time_before_sec", "rtk_time_after_sec", "rtk_interpolation_ratio")
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        for item in pairs:
            writer.writerow({
                "timestamp_sec": fmt(item["timestamp"], 9),
                "elapsed_from_common_start_sec": fmt(item["elapsed"]),
                "localization_x_m": fmt(item["loc_x"]), "localization_y_m": fmt(item["loc_y"]),
                "rtk_x_m": fmt(item["rtk_x"]), "rtk_y_m": fmt(item["rtk_y"]),
                "dx_m": fmt(item["dx"]), "dy_m": fmt(item["dy"]),
                "xy_error_m": fmt(item["error"]),
                "rtk_time_before_sec": fmt(item["rtk_before"], 9),
                "rtk_time_after_sec": fmt(item["rtk_after"], 9),
                "rtk_interpolation_ratio": fmt(item["ratio"]),
            })


def write_summary(path, metadata, metrics, component_metrics):
    rows = [
        ("paired_sample_count", metrics["count"], "samples", "成功时间匹配的定位样本数"),
        ("xy_mae", fmt(metrics["mae_m"]), "m", "每个时刻的 XY 欧氏误差的平均值，越小越好"),
        ("xy_rmse", fmt(metrics["rmse_m"]), "m", "对较大 XY 误差更敏感的均方根值，越小越好"),
        ("xy_p95", fmt(metrics["p95_m"]), "m", "95% 的已匹配样本不超过此 XY 误差"),
        ("x_signed_mean_error", fmt(component_metrics["signed_mean_dx_m"]), "m",
         "带符号 X 偏差：定位 X - RTK X；负值表示定位轨迹位于 RTK 的地图 -X 方向"),
        ("y_signed_mean_error", fmt(component_metrics["signed_mean_dy_m"]), "m",
         "带符号 Y 偏差：定位 Y - RTK Y；负值表示定位轨迹位于 RTK 的地图 -Y 方向"),
        ("x_mae", fmt(component_metrics["abs_dx_mae_m"]), "m", "|dx| 的平均值"),
        ("x_rmse", fmt(component_metrics["abs_dx_rmse_m"]), "m", "X 分量均方根误差"),
        ("x_p95", fmt(component_metrics["abs_dx_p95_m"]), "m", "95% 的 |dx| 不超过此值"),
        ("y_mae", fmt(component_metrics["abs_dy_mae_m"]), "m", "|dy| 的平均值"),
        ("y_rmse", fmt(component_metrics["abs_dy_rmse_m"]), "m", "Y 分量均方根误差"),
        ("y_p95", fmt(component_metrics["abs_dy_p95_m"]), "m", "95% 的 |dy| 不超过此值"),
        ("localization_sample_count", metadata["localization_count"], "samples", "红色定位 CSV 的总样本数"),
        ("rtk_sample_count", metadata["rtk_count"], "samples",
         "%s CSV 的总样本数" % metadata["rtk_reference_label"]),
        ("common_start_sec", fmt(metadata["common_start"], 9), "s", "红蓝轨迹共同时间段起点"),
        ("common_end_sec", fmt(metadata["common_end"], 9), "s", "红蓝轨迹共同时间段终点"),
        ("common_duration", fmt(metadata["common_end"] - metadata["common_start"]), "s", "共同时间段长度"),
        ("rtk_before_localization", fmt(metadata["rtk_before_localization"]), "s", "RTK 已存在而红色定位尚未开始的时长；不计入误差"),
        ("localization_before_rtk", fmt(metadata["localization_before_rtk"]), "s",
         "红色定位已开始而 RTK 参考尚未出现的时长；不计入误差"),
        ("unmatched_localization_count", metadata["unmatched_count"], "samples", "共同时间段内因 RTK 间隔过大而未匹配的定位样本数"),
        ("timestamp_offset_applied", fmt(metadata["timestamp_offset"]), "s", "加到 RTK 时间戳的固定偏移"),
    ]
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(("metric", "value", "unit", "meaning"))
        writer.writerows(rows)


def grouped_metrics(pairs, key_function):
    groups = defaultdict(list)
    for pair in pairs:
        groups[key_function(pair)].append(pair["error"])
    return groups


def write_time_segments(path, pairs, common_start, segment_sec):
    groups = grouped_metrics(pairs, lambda item: int(math.floor(item["elapsed"] / segment_sec)))
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(("segment_index", "segment_start_sec", "segment_end_sec", "sample_count",
                         "xy_mae_m", "xy_rmse_m", "xy_p95_m"))
        for index in sorted(groups):
            metrics = calculate_metrics(groups[index])
            start = common_start + index * segment_sec
            writer.writerow((index, fmt(start, 9), fmt(start + segment_sec, 9), metrics["count"],
                             fmt(metrics["mae_m"]), fmt(metrics["rmse_m"]), fmt(metrics["p95_m"])))


def write_spatial_grid(path, pairs, grid_m):
    def cell(item):
        return (int(math.floor(item["loc_x"] / grid_m)), int(math.floor(item["loc_y"] / grid_m)))

    groups = grouped_metrics(pairs, cell)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow(("cell_ix", "cell_iy", "x_min_m", "x_max_m", "y_min_m", "y_max_m", "sample_count",
                         "xy_mae_m", "xy_rmse_m", "xy_p95_m"))
        for ix, iy in sorted(groups):
            metrics = calculate_metrics(groups[(ix, iy)])
            writer.writerow((ix, iy, fmt(ix * grid_m), fmt((ix + 1) * grid_m),
                             fmt(iy * grid_m), fmt((iy + 1) * grid_m), metrics["count"],
                             fmt(metrics["mae_m"]), fmt(metrics["rmse_m"]), fmt(metrics["p95_m"])))


def write_readme(path, args, metadata):
    content = """# XY 定位误差评估结果

该目录比较红色 FAST-LOCALIZATION 定位轨迹和%s轨迹的 **XY 平面差异**。
Z 坐标没有参与计算；全局初始化前、红色轨迹尚未产生的时段不计入误差。

## 文件

- `%s_summary.csv`：整体的 XY MAE、XY RMSE、XY P95、X/Y 分量误差和时间覆盖信息。
- `%s_pairs.csv`：每个成功匹配时刻的红色轨迹、插值后的 RTK、dx、dy 与 XY 欧氏误差。
- `%s_time_segments.csv`：每 %g 秒的统计。
- `%s_spatial_grid.csv`：按红色定位位置划分的 %g m × %g m 空间网格统计。

## 口径与限制

- RTK 参考：%s；位置取自 CSV 的 `map_x,map_y`。
- 对每个红色定位时间戳，在相邻 RTK 点之间线性插值；相邻 RTK 间隔大于 %g 秒时该定位样本不计入统计。
- 结果的含义是“相对 %s 的 XY 偏差”，**不是**可直接对外报告的绝对真值 ATE：RTK 参考、时间同步和地图坐标变换仍应按具体数据来源审计。
- `rtk_before_localization` 与 `localization_before_rtk` 只描述双方轨迹的前段覆盖缺失，不应解释为连续定位误差。

## 输入

- 定位 CSV：`%s`
- RTK CSV：`%s`
- RTK 时间戳固定偏移：%.6f s
""" % (args.rtk_reference_label, args.output_prefix, args.output_prefix, args.output_prefix,
       args.time_segment_sec, args.output_prefix, args.spatial_grid_m, args.spatial_grid_m,
       args.rtk_reference_label, args.max_rtk_gap_sec, args.rtk_reference_label,
       args.localization_csv, args.rtk_csv, metadata["timestamp_offset"])
    path.write_text(content, encoding="utf-8")


def main():
    args = parse_arguments()
    localization_path = Path(args.localization_csv).expanduser().resolve()
    rtk_path = Path(args.rtk_csv).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    if not localization_path.is_file():
        raise ValueError("找不到定位 CSV：%s" % localization_path)
    if not rtk_path.is_file():
        raise ValueError("找不到 RTK CSV：%s" % rtk_path)

    localization = load_csv(localization_path, ("timestamp_sec", "x", "y"), "localization")
    rtk = load_csv(rtk_path, ("timestamp_sec", "map_x", "map_y"), "rtk")
    if args.timestamp_offset_sec:
        rtk = [(stamp + args.timestamp_offset_sec, x, y) for stamp, x, y in rtk]
        rtk.sort(key=lambda item: item[0])

    common_start = max(localization[0][0], rtk[0][0])
    common_end = min(localization[-1][0], rtk[-1][0])
    if common_start > common_end:
        raise ValueError("定位轨迹与 RTK 轨迹没有共同时间段")

    rtk_times = [item[0] for item in rtk]
    pairs = []
    unmatched_count = 0
    for timestamp, loc_x, loc_y in localization:
        if timestamp < common_start or timestamp > common_end:
            continue
        interpolated = interpolate_rtk(rtk, rtk_times, timestamp, args.max_rtk_gap_sec)
        if interpolated is None:
            unmatched_count += 1
            continue
        rtk_x, rtk_y, rtk_before, rtk_after, ratio = interpolated
        dx, dy = loc_x - rtk_x, loc_y - rtk_y
        pairs.append({
            "timestamp": timestamp, "elapsed": timestamp - common_start,
            "loc_x": loc_x, "loc_y": loc_y, "rtk_x": rtk_x, "rtk_y": rtk_y,
            "dx": dx, "dy": dy, "error": math.hypot(dx, dy),
            "rtk_before": rtk_before, "rtk_after": rtk_after, "ratio": ratio,
        })
    metrics = calculate_metrics([item["error"] for item in pairs])
    if metrics is None:
        raise ValueError("共同时间段内没有成功的 RTK 时间匹配；可检查 --max-rtk-gap-sec 或时间戳")
    component_metrics = calculate_component_metrics(pairs)

    metadata = {
        "localization_count": len(localization), "rtk_count": len(rtk),
        "common_start": common_start, "common_end": common_end,
        "rtk_before_localization": max(0.0, localization[0][0] - rtk[0][0]),
        "localization_before_rtk": max(0.0, rtk[0][0] - localization[0][0]),
        "unmatched_count": unmatched_count, "timestamp_offset": args.timestamp_offset_sec,
        "rtk_reference_label": args.rtk_reference_label,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_pairs(output_dir / (args.output_prefix + "_pairs.csv"), pairs)
    write_summary(output_dir / (args.output_prefix + "_summary.csv"), metadata, metrics, component_metrics)
    write_time_segments(output_dir / (args.output_prefix + "_time_segments.csv"), pairs,
                        common_start, args.time_segment_sec)
    write_spatial_grid(output_dir / (args.output_prefix + "_spatial_grid.csv"), pairs,
                       args.spatial_grid_m)
    write_readme(output_dir / (args.output_prefix + "_README.md"), args, metadata)

    print("XY evaluation complete: %s" % output_dir)
    print("Output prefix: %s" % args.output_prefix)
    print("matched samples: %d; common duration: %.3f s; unmatched in common interval: %d" %
          (metrics["count"], common_end - common_start, unmatched_count))
    print("XY MAE: %.3f m; XY RMSE: %.3f m; XY P95: %.3f m" %
          (metrics["mae_m"], metrics["rmse_m"], metrics["p95_m"]))
    print("Note: these are XY discrepancies relative to %s, not absolute ground-truth ATE." %
          args.rtk_reference_label)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        print("ERROR: %s" % error, file=sys.stderr)
        sys.exit(2)
