#!/usr/bin/env python3
"""Compare isolated raw and filtered FAST-LIVO2 dynamic-removal mapping runs.

This is deliberately a front-end comparison: both bags must be produced from
the same ROS2 source window, while the mapper's optional loop and dense RGB
products are disabled.  It proves input identity, odometry continuity and
relative-trajectory stability.  It does not claim map quality automatically:
the paired PCDs still need the documented CloudCompare visual review.
"""

import argparse
import bisect
import json
import math
import os
import re
from collections import defaultdict

import rosbag


INPUT = "/front_lidar"
IMU = "/front_lidar/imu"
ODOM = "/aft_mapped_to_init"
MAPPER_ERROR_PATTERN = re.compile(
    r"Failed to find match for field|IMU and LiDAR not synced|lidar loop back|"
    r"missing.*field|field.*missing", re.IGNORECASE)


def stamp(message):
    return message.header.stamp.to_nsec()


def vector_norm(vector):
    return math.sqrt(sum(component * component for component in vector))


def subtract(first, second):
    return tuple(a - b for a, b in zip(first, second))


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = int(math.ceil(fraction * len(ordered))) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def monotonic_failures(values):
    return sum(later <= earlier for earlier, later in zip(values, values[1:]))


def interpolate_position(poses, value, max_gap_ns):
    """Return a position at value only when a safe nearest/bracketing pose exists."""
    if not poses:
        return None
    stamps = [item[0] for item in poses]
    index = bisect.bisect_left(stamps, value)
    if index == 0:
        return poses[0][1] if stamps[0] - value <= max_gap_ns else None
    if index == len(poses):
        return poses[-1][1] if value - stamps[-1] <= max_gap_ns else None
    before_stamp, before = poses[index - 1]
    after_stamp, after = poses[index]
    if value - before_stamp <= max_gap_ns:
        return before
    if after_stamp - value <= max_gap_ns:
        return after
    if after_stamp - before_stamp > 2 * max_gap_ns:
        return None
    ratio = float(value - before_stamp) / float(after_stamp - before_stamp)
    return tuple(a + ratio * (b - a) for a, b in zip(before, after))


def pcd_summary(path):
    result = {"path": path, "exists": bool(path) and os.path.isfile(path)}
    if not result["exists"]:
        return result
    result["size_bytes"] = os.path.getsize(path)
    points = None
    try:
        with open(path, "rb") as source:
            for _ in range(128):
                line = source.readline()
                if not line:
                    break
                text = line.decode("ascii", errors="replace").strip()
                if text.upper().startswith("POINTS "):
                    try:
                        points = int(text.split()[1])
                    except (IndexError, ValueError):
                        pass
                if text.upper().startswith("DATA "):
                    result["data"] = text.split(maxsplit=1)[1] if " " in text else ""
                    break
    except OSError as error:
        result["error"] = str(error)
    result["points"] = points
    return result


def mapper_log_matches(path):
    if not path:
        return []
    matches = []
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as source:
            for line_number, line in enumerate(source, 1):
                if MAPPER_ERROR_PATTERN.search(line):
                    matches.append({"line": line_number, "text": line.rstrip()})
    except OSError as error:
        return [{"error": str(error)}]
    return matches


def read_run(path, max_gap_sec):
    counts = defaultdict(int)
    input_stamps = []
    imu_stamps = []
    poses = []
    with rosbag.Bag(path, "r") as bag:
        for topic, message, _ in bag.read_messages(topics=[INPUT, IMU, ODOM]):
            counts[topic] += 1
            if topic == INPUT:
                input_stamps.append(stamp(message))
            elif topic == IMU:
                imu_stamps.append(stamp(message))
            else:
                position = message.pose.pose.position
                poses.append((stamp(message), (position.x, position.y, position.z)))

    max_gap_ns = int(max_gap_sec * 1e9)
    usable = sum(interpolate_position(poses, value, max_gap_ns) is not None
                 for value in input_stamps)
    path_length = sum(
        vector_norm(subtract(later[1], earlier[1]))
        for earlier, later in zip(poses, poses[1:]))
    duration = (input_stamps[-1] - input_stamps[0]) / 1e9 if len(input_stamps) > 1 else 0.0
    return {
        "bag": path,
        "topic_counts": dict(counts),
        "input_stamps": input_stamps,
        "imu_stamps": imu_stamps,
        "poses": poses,
        "input_frames": len(input_stamps),
        "imu_messages": len(imu_stamps),
        "odom_messages": len(poses),
        "input_duration_sec": duration,
        "input_non_monotonic": monotonic_failures(input_stamps),
        "imu_non_monotonic": monotonic_failures(imu_stamps),
        "odom_non_monotonic": monotonic_failures([item[0] for item in poses]),
        "usable_odom_frames": usable,
        "odom_coverage": float(usable) / len(input_stamps) if input_stamps else 0.0,
        "trajectory_length_m": path_length,
    }


def comparison(baseline, filtered, max_gap_sec):
    max_gap_ns = int(max_gap_sec * 1e9)
    if not baseline["poses"] or not filtered["poses"]:
        return {"samples": 0}
    raw_origin = baseline["poses"][0][1]
    filtered_origin = filtered["poses"][0][1]
    deltas = []
    last_delta = None
    for value in baseline["input_stamps"]:
        raw_position = interpolate_position(baseline["poses"], value, max_gap_ns)
        filtered_position = interpolate_position(filtered["poses"], value, max_gap_ns)
        if raw_position is None or filtered_position is None:
            continue
        raw_relative = subtract(raw_position, raw_origin)
        filtered_relative = subtract(filtered_position, filtered_origin)
        last_delta = vector_norm(subtract(raw_relative, filtered_relative))
        deltas.append(last_delta)
    raw_length = baseline["trajectory_length_m"]
    filtered_length = filtered["trajectory_length_m"]
    return {
        "alignment": "each run subtracts its first odometry position; no loop/global pose is used",
        "samples": len(deltas),
        "relative_translation_delta_m_mean": sum(deltas) / len(deltas) if deltas else None,
        "relative_translation_delta_m_p95": percentile(deltas, 0.95),
        "relative_translation_delta_m_max": max(deltas) if deltas else None,
        "relative_translation_delta_m_last": last_delta,
        "trajectory_length_ratio_filtered_over_raw": (
            filtered_length / raw_length if raw_length > 1e-9 else None),
        "trajectory_length_relative_difference": (
            abs(filtered_length - raw_length) / raw_length if raw_length > 1e-9 else None),
    }


def compact(run):
    return {key: value for key, value in run.items()
            if key not in ("input_stamps", "imu_stamps", "poses")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-bag", required=True)
    parser.add_argument("--filtered-bag", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--baseline-map", default="")
    parser.add_argument("--filtered-map", default="")
    parser.add_argument("--baseline-mapper-log", default="")
    parser.add_argument("--filtered-mapper-log", default="")
    parser.add_argument("--min-input-frames", type=int, default=1)
    parser.add_argument("--min-odom-coverage", type=float, default=0.95)
    parser.add_argument("--odom-max-gap-sec", type=float, default=0.60)
    parser.add_argument("--max-relative-path-length-diff", type=float, default=-1.0,
                        help="Fail if normalized raw/filtered path-length difference is larger; negative disables")
    parser.add_argument("--max-final-relative-position-delta-m", type=float, default=-1.0,
                        help="Fail if final origin-normalized position delta is larger; negative disables")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    baseline = read_run(args.baseline_bag, args.odom_max_gap_sec)
    filtered = read_run(args.filtered_bag, args.odom_max_gap_sec)
    trajectories = comparison(baseline, filtered, args.odom_max_gap_sec)
    baseline_log_matches = mapper_log_matches(args.baseline_mapper_log)
    filtered_log_matches = mapper_log_matches(args.filtered_mapper_log)
    baseline_map = pcd_summary(args.baseline_map)
    filtered_map = pcd_summary(args.filtered_map)

    identical_input = baseline["input_stamps"] == filtered["input_stamps"]
    both_healthy = all(
        run["input_frames"] >= args.min_input_frames
        and run["odom_messages"] > 0
        and run["input_non_monotonic"] == 0
        and run["imu_non_monotonic"] == 0
        and run["odom_non_monotonic"] == 0
        and run["odom_coverage"] >= args.min_odom_coverage
        for run in (baseline, filtered))
    path_difference = trajectories.get("trajectory_length_relative_difference")
    final_delta = trajectories.get("relative_translation_delta_m_last")
    threshold_ok = (
        (args.max_relative_path_length_diff < 0 or
         (path_difference is not None and path_difference <= args.max_relative_path_length_diff))
        and (args.max_final_relative_position_delta_m < 0 or
             (final_delta is not None and final_delta <= args.max_final_relative_position_delta_m)))
    maps_ok = ((not args.baseline_map or baseline_map["exists"])
               and (not args.filtered_map or filtered_map["exists"]))
    report = {
        "comparison": "dynamic_removal_frontend_raw_vs_filtered",
        "baseline": compact(baseline),
        "filtered": compact(filtered),
        "same_input_stamp_sequence": identical_input,
        "trajectory_comparison": trajectories,
        "baseline_map": baseline_map,
        "filtered_map": filtered_map,
        "baseline_mapper_log_matches": baseline_log_matches,
        "filtered_mapper_log_matches": filtered_log_matches,
        "criteria": {
            "min_input_frames": args.min_input_frames,
            "min_odom_coverage": args.min_odom_coverage,
            "odom_max_gap_sec": args.odom_max_gap_sec,
            "max_relative_path_length_diff": args.max_relative_path_length_diff,
            "max_final_relative_position_delta_m": args.max_final_relative_position_delta_m,
            "manual_map_review_required": True,
        },
    }
    report["pass"] = (both_healthy and identical_input and maps_ok and threshold_ok
                      and not baseline_log_matches and not filtered_log_matches)
    report_path = os.path.join(args.output_dir, "dynamic_removal_comparison_report.json")
    with open(report_path, "w", encoding="utf-8") as output:
        json.dump(report, output, ensure_ascii=False, indent=2)
        output.write("\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("\nDynamic-removal comparison report: {0}".format(report_path))
    raise SystemExit(0 if report["pass"] else 2)


if __name__ == "__main__":
    main()
