#!/usr/bin/env python3
"""Offline safety checks for dynamic-removal recordings.

This script verifies the byte-identical static baseline, the field-preserving
filtered subset and ordered runtime diagnostics.  It also exports diagnostic
XYZ samples for CloudCompare.
"""

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from collections import Counter, defaultdict

import rosbag
from sensor_msgs import point_cloud2


INPUT = "/front_lidar"
IMU = "/front_lidar/imu"
RAW = "/dynamic_removal/raw_points"
STATIC = "/dynamic_removal/static_points"
FILTERED = "/dynamic_removal/filtered_points"
MOVING = "/dynamic_removal/moving_object_points"
GROUND = "/dynamic_removal/protected_ground_points"
CANDIDATE = "/dynamic_removal/static_candidate_points"
ODOM = "/aft_mapped_to_init"
BOXES = "/dynamic_removal/boxes"
MOVING_BOXES = "/dynamic_removal/moving_boxes"
GROUND_DIAGNOSTICS = "/dynamic_removal/ground_diagnostics"
FRAME_DIAGNOSTICS = "/dynamic_removal/frame_diagnostics"


def stamp(message):
    return message.header.stamp.to_nsec()


def digest(message):
    """Hash all PointCloud2 metadata that must survive a passthrough."""
    layout = {
        "frame_id": message.header.frame_id,
        "height": message.height,
        "width": message.width,
        "is_bigendian": message.is_bigendian,
        "point_step": message.point_step,
        "row_step": message.row_step,
        "is_dense": message.is_dense,
        "fields": [(f.name, f.offset, f.datatype, f.count) for f in message.fields],
    }
    hasher = hashlib.sha256()
    hasher.update(json.dumps(layout, sort_keys=True).encode("utf-8"))
    hasher.update(message.data)
    return hasher.hexdigest()


def compatible_subset(source, subset):
    """Check field-level compatibility of a binary-record filtered cloud."""
    return (
        subset.header.frame_id == source.header.frame_id
        and subset.point_step == source.point_step
        and subset.is_bigendian == source.is_bigendian
        and [(f.name, f.offset, f.datatype, f.count) for f in subset.fields]
        == [(f.name, f.offset, f.datatype, f.count) for f in source.fields]
        and subset.width * subset.height <= source.width * source.height
    )


def binary_record_subset(source, subset):
    """Verify that filtered records are an in-order binary subset of input.

    Layout equality alone cannot prove preservation of tag/line/timestamp.
    The runtime safety contract requires each output record to be copied from this input message,
    in order, so inspect complete point_step-sized records rather than decoded
    XYZ values.
    """
    if not compatible_subset(source, subset) or source.point_step == 0:
        return False
    source_count = source.width * source.height
    subset_count = subset.width * subset.height
    if len(subset.data) != subset_count * subset.point_step:
        return False
    if source.width == 0 or source.row_step < source.width * source.point_step:
        return subset_count == 0

    source_index = 0
    for subset_index in range(subset_count):
        target_begin = subset_index * subset.point_step
        target = subset.data[target_begin:target_begin + subset.point_step]
        found = False
        while source_index < source_count:
            row = source_index // source.width
            column = source_index % source.width
            source_begin = row * source.row_step + column * source.point_step
            if source_begin + source.point_step > len(source.data):
                return False
            if source.data[source_begin:source_begin + source.point_step] == target:
                found = True
                source_index += 1
                break
            source_index += 1
        if not found:
            return False
    return True


def append_xyz(message, destination, limit):
    for point in point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True):
        if len(destination) >= limit:
            return
        destination.append(point)


def write_pcd(path, points):
    with open(path, "w", encoding="utf-8") as output:
        output.write("# .PCD v0.7 - Point Cloud Data file format\n")
        output.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        output.write("WIDTH {0}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n".format(len(points)))
        output.write("POINTS {0}\nDATA ascii\n".format(len(points)))
        for x, y, z in points:
            output.write("{0:.6f} {1:.6f} {2:.6f}\n".format(x, y, z))


def write_message_pcd(path, message):
    """Export one original ROS point-cloud frame as XYZ for CloudCompare."""
    points = []
    append_xyz(message, points, float("inf"))
    write_pcd(path, points)


def normalize_quaternion(quaternion):
    x, y, z, w = quaternion
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def slerp(first, second, ratio):
    """Quaternion interpolation without a NumPy dependency."""
    ax, ay, az, aw = normalize_quaternion(first)
    bx, by, bz, bw = normalize_quaternion(second)
    dot = ax * bx + ay * by + az * bz + aw * bw
    if dot < 0.0:
        bx, by, bz, bw = -bx, -by, -bz, -bw
        dot = -dot
    if dot > 0.9995:
        return normalize_quaternion((
            ax + ratio * (bx - ax), ay + ratio * (by - ay),
            az + ratio * (bz - az), aw + ratio * (bw - aw),
        ))
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    left = math.sin((1.0 - ratio) * theta) / sin_theta
    right = math.sin(ratio * theta) / sin_theta
    return (left * ax + right * bx, left * ay + right * by,
            left * az + right * bz, left * aw + right * bw)


def rotate(quaternion, point):
    """Rotate a point by an xyzw quaternion."""
    x, y, z, w = normalize_quaternion(quaternion)
    px, py, pz = point
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    )


def pose_at(stamp_ns, odom_poses, max_gap_ns):
    """Interpolate /aft_mapped_to_init at a LiDAR stamp for world export."""
    if not odom_poses:
        return None
    stamps = [item[0] for item in odom_poses]
    index = bisect.bisect_left(stamps, stamp_ns)
    if index == 0:
        return None if stamps[0] - stamp_ns > max_gap_ns else odom_poses[0][1]
    if index == len(odom_poses):
        return None if stamp_ns - stamps[-1] > max_gap_ns else odom_poses[-1][1]
    before_stamp, before = odom_poses[index - 1]
    after_stamp, after = odom_poses[index]
    if after_stamp - before_stamp > max_gap_ns:
        return None
    ratio = float(stamp_ns - before_stamp) / float(after_stamp - before_stamp)
    before_pose = before.pose.pose
    after_pose = after.pose.pose
    position = (
        before_pose.position.x + ratio * (after_pose.position.x - before_pose.position.x),
        before_pose.position.y + ratio * (after_pose.position.y - before_pose.position.y),
        before_pose.position.z + ratio * (after_pose.position.z - before_pose.position.z),
    )
    orientation = slerp(
        (before_pose.orientation.x, before_pose.orientation.y,
         before_pose.orientation.z, before_pose.orientation.w),
        (after_pose.orientation.x, after_pose.orientation.y,
         after_pose.orientation.z, after_pose.orientation.w),
        ratio,
    )
    return position, orientation


def write_world_message_pcd(path, message, pose, lidar_to_imu_translation):
    """Export a LiDAR frame into camera_init using odom and LiDAR->IMU extrinsics."""
    position, orientation = pose
    points = []
    for x, y, z in point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True):
        imu_point = (
            x + lidar_to_imu_translation[0],
            y + lidar_to_imu_translation[1],
            z + lidar_to_imu_translation[2],
        )
        rx, ry, rz = rotate(orientation, imu_point)
        points.append((rx + position[0], ry + position[1], rz + position[2]))
    write_pcd(path, points)


def write_inspection_frames(output_dir, selected_stamps, input_messages, filtered_messages,
                            moving_messages, ground_messages, candidate_messages,
                            boxes_messages, moving_boxes_messages, ground_diagnostics,
                            odom_poses, odom_max_gap_ns, lidar_to_imu_translation):
    """Write frame-by-frame CloudCompare inputs and a timestamped box table.

    Aggregate samples are useful for a quick overview but mix many LiDAR poses.
    These exports make the exact records removed in a single sensor frame
    inspectable: raw_points - filtered_points must equal moving_removed_points,
    while protected_ground_points must remain in filtered_points.
    """
    inspection_dir = os.path.join(output_dir, "inspection_frames")
    os.makedirs(inspection_dir, exist_ok=True)
    csv_path = os.path.join(inspection_dir, "inspection_boxes.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow([
            "stamp_ns", "stamp_sec", "box_set", "label", "score",
            "center_x", "center_y", "center_z", "size_x", "size_y", "size_z",
        ])
        for stamp_ns in selected_stamps:
            frame_dir = os.path.join(inspection_dir, "frame_{0}".format(stamp_ns))
            os.makedirs(frame_dir, exist_ok=True)
            exports = [
                ("raw_points.pcd", input_messages.get(stamp_ns)),
                ("filtered_points.pcd", filtered_messages.get(stamp_ns)),
                ("moving_removed_points.pcd", moving_messages.get(stamp_ns)),
                ("protected_ground_points.pcd", ground_messages.get(stamp_ns)),
                ("static_candidate_points.pcd", candidate_messages.get(stamp_ns)),
            ]
            for filename, message in exports:
                if message is not None:
                    write_message_pcd(os.path.join(frame_dir, filename), message)

            pose = pose_at(stamp_ns, odom_poses, odom_max_gap_ns)
            if pose is not None:
                with open(os.path.join(frame_dir, "camera_init_pose.txt"), "w", encoding="utf-8") as output:
                    position, orientation = pose
                    output.write("stamp_ns={0}\nposition={1:.9f},{2:.9f},{3:.9f}\n"
                                 "orientation_xyzw={4:.9f},{5:.9f},{6:.9f},{7:.9f}\n".format(
                                     stamp_ns, position[0], position[1], position[2],
                                     orientation[0], orientation[1], orientation[2], orientation[3]))
                for filename, message in exports:
                    if message is not None:
                        write_world_message_pcd(
                            os.path.join(frame_dir, "camera_init_" + filename),
                            message, pose, lidar_to_imu_translation)

            for set_name, message in (("detected", boxes_messages.get(stamp_ns)),
                                      ("confirmed_moving", moving_boxes_messages.get(stamp_ns))):
                if message is None:
                    continue
                for box in message.boxes:
                    writer.writerow([
                        stamp_ns, "{0:.9f}".format(stamp_ns / 1e9), set_name,
                        box.label, "{0:.6f}".format(box.value),
                        "{0:.6f}".format(box.pose.position.x),
                        "{0:.6f}".format(box.pose.position.y),
                        "{0:.6f}".format(box.pose.position.z),
                        "{0:.6f}".format(box.dimensions.x),
                        "{0:.6f}".format(box.dimensions.y),
                        "{0:.6f}".format(box.dimensions.z),
                    ])
    diagnostics_path = os.path.join(inspection_dir, "ground_diagnostics.csv")
    columns = [
        "stamp_ns", "label", "state", "accepted", "observations", "duration_sec",
        "speed_mps", "ground_status", "ground_inliers", "ground_nx", "ground_ny",
        "ground_nz", "ground_d", "ground_z_at_box_center", "box_bottom_z",
        "ground_height_above_box_bottom",
    ]
    selected_set = set(selected_stamps)
    with open(diagnostics_path, "w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        for record in ground_diagnostics:
            if record["stamp_ns"] in selected_set:
                writer.writerow(record)
    return inspection_dir


def count_usable_odom(input_stamps, odom_stamps, nearest_limit, bracket_limit):
    """Count LiDAR frames with safe dynamic-removal odometry support.

    A near pose is accepted directly.  Otherwise, a LiDAR timestamp must be
    bracketed by two odometry poses whose individual gaps are bounded.  This
    mirrors the conservative policy in centerpp_node without requiring any
    map-frame transformation during offline validation.
    """
    if not odom_stamps:
        return 0
    ordered = sorted(odom_stamps)
    usable = 0
    for value in input_stamps:
        index = bisect.bisect_left(ordered, value)
        left_ok = index > 0 and value - ordered[index - 1] <= bracket_limit
        right_ok = index < len(ordered) and ordered[index] - value <= bracket_limit
        nearest_ok = (
            (index > 0 and value - ordered[index - 1] <= nearest_limit)
            or (index < len(ordered) and ordered[index] - value <= nearest_limit)
        )
        if nearest_ok or (left_ok and right_ok):
            usable += 1
    return usable


def parse_ground_diagnostic(payload):
    """Parse a key=value ground-decision record."""
    record = {}
    for item in payload.split(";"):
        key, separator, value = item.partition("=")
        if separator and key:
            record[key] = value
    try:
        record["stamp_ns"] = int(record["stamp_ns"])
    except (KeyError, ValueError):
        return None
    return record


def parse_frame_diagnostic(payload):
    """Parse a per-input key=value runtime decision record."""
    record = {}
    for item in payload.split(";"):
        key, separator, value = item.partition("=")
        if separator and key:
            record[key] = value
    try:
        record["seq"] = int(record["seq"])
        record["stamp_ns"] = int(record["stamp_ns"])
    except (KeyError, ValueError):
        return None
    return record


def diagnostic_numbers(records, field):
    values = []
    for record in records:
        try:
            values.append(float(record[field]))
        except (KeyError, ValueError):
            pass
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="ROS1 bag recorded with dynamic removal enabled")
    parser.add_argument("--output-dir", default="dynamic_removal_report")
    parser.add_argument("--expect-no-removal", action="store_true",
                        help="Require filtered_points to equal input byte-for-byte; use for a static-scene bag")
    parser.add_argument("--expect-moving", action="store_true",
                        help="Require confirmed moving boxes, removed non-ground points and protected ground points")
    parser.add_argument("--min-odom-coverage", type=float, default=0.0,
                        help="Minimum fraction of input LiDAR frames with usable odometry; 0 disables this check")
    parser.add_argument("--odom-nearest-sec", type=float, default=0.10,
                        help="Maximum nearest odometry time error, matching the dynamic-removal configuration")
    parser.add_argument("--odom-bracket-sec", type=float, default=0.60,
                        help="Maximum one-side bracket time error, matching the dynamic-removal configuration")
    parser.add_argument("--max-inspection-frames", type=int, default=20,
                        help="Export up to this many confirmed-moving frames for exact CloudCompare inspection; 0 disables export")
    parser.add_argument("--world-odom-max-gap-sec", type=float, default=0.60,
                        help="Maximum odometry bracket gap when exporting camera_init inspection clouds")
    parser.add_argument("--lidar-to-imu-translation", type=float, nargs=3,
                        default=[-0.011, -0.02329, 0.04412], metavar=("TX", "TY", "TZ"),
                        help="LiDAR->IMU translation used for camera_init export; RobotDog calibrated default")
    parser.add_argument("--max-points", type=int, default=500000,
                        help="Maximum XYZ points exported per diagnostic topic")
    parser.add_argument("--require-frame-diagnostics", action="store_true",
                        help="Require one ordered frame_diagnostics record per input cloud")
    parser.add_argument("--expect-frame-reason", action="append", default=[],
                        help="Require at least one frame diagnostic with this reason; may repeat")
    parser.add_argument("--forbid-frame-reason", action="append", default=[],
                        help="Fail if any frame diagnostic has this reason; may repeat")
    parser.add_argument("--require-zero-dropq", action="store_true",
                        help="Require every frame diagnostic to report dropQ=0")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    counts = defaultdict(int)
    input_messages = {}
    raw_messages = {}
    static_messages = {}
    filtered_messages = {}
    moving_messages = {}
    ground_messages = {}
    candidate_messages = {}
    boxes_messages = {}
    moving_boxes_messages = {}
    ground_diagnostics = []
    frame_diagnostics = []
    odom_stamps = []
    odom_poses = []
    box_frames = box_count = moving_box_frames = moving_box_count = 0
    moving_points = []
    protected_points = []
    candidate_points = []
    input_order = []
    raw_order = []
    static_order = []
    filtered_order = []
    topics = [INPUT, IMU, RAW, STATIC, FILTERED, MOVING, GROUND, CANDIDATE,
              ODOM, BOXES, MOVING_BOXES, GROUND_DIAGNOSTICS, FRAME_DIAGNOSTICS]

    with rosbag.Bag(args.bag, "r") as bag:
        for topic, message, _ in bag.read_messages(topics=topics):
            counts[topic] += 1
            if topic == GROUND_DIAGNOSTICS:
                record = parse_ground_diagnostic(message.data)
                if record is not None:
                    ground_diagnostics.append(record)
                continue
            if topic == FRAME_DIAGNOSTICS:
                record = parse_frame_diagnostic(message.data)
                if record is not None:
                    frame_diagnostics.append(record)
                continue
            key = stamp(message)
            if topic == INPUT:
                input_messages[key] = message
                input_order.append(key)
            elif topic == RAW:
                raw_messages[key] = message
                raw_order.append(key)
            elif topic == STATIC:
                static_messages[key] = message
                static_order.append(key)
            elif topic == FILTERED:
                filtered_messages[key] = message
                filtered_order.append(key)
            elif topic == MOVING:
                append_xyz(message, moving_points, args.max_points)
                moving_messages[key] = message
            elif topic == GROUND:
                append_xyz(message, protected_points, args.max_points)
                ground_messages[key] = message
            elif topic == CANDIDATE:
                append_xyz(message, candidate_points, args.max_points)
                candidate_messages[key] = message
            elif topic == ODOM:
                odom_stamps.append(stamp(message))
                odom_poses.append((stamp(message), message))
            elif topic == BOXES:
                boxes_messages[key] = message
                number = len(message.boxes)
                box_count += number
                if number:
                    box_frames += 1
            elif topic == MOVING_BOXES:
                moving_boxes_messages[key] = message
                number = len(message.boxes)
                moving_box_count += number
                if number:
                    moving_box_frames += 1

    input_keys = set(input_messages)
    raw_keys = set(raw_messages)
    static_keys = set(static_messages)
    filtered_keys = set(filtered_messages)
    passthrough_mismatch = [key for key in sorted(raw_keys & static_keys)
                            if digest(raw_messages[key]) != digest(static_messages[key])]
    raw_input_mismatch = [key for key in sorted(raw_keys & input_keys)
                          if digest(raw_messages[key]) != digest(input_messages[key])]
    filtered_layout_failures = [key for key in sorted(filtered_keys & input_keys)
                                if not compatible_subset(input_messages[key], filtered_messages[key])]
    filtered_record_failures = [key for key in sorted(filtered_keys & input_keys)
                                if not binary_record_subset(input_messages[key], filtered_messages[key])]
    filtered_equal_input = [key for key in sorted(filtered_keys & input_keys)
                            if digest(filtered_messages[key]) == digest(input_messages[key])]
    filtered_removed_frames = [key for key in sorted(filtered_keys & input_keys)
                               if digest(filtered_messages[key]) != digest(input_messages[key])]
    input_stamps = sorted(input_keys)
    usable_odom_frames = count_usable_odom(
        input_stamps, odom_stamps,
        int(args.odom_nearest_sec * 1e9), int(args.odom_bracket_sec * 1e9))
    odom_coverage = float(usable_odom_frames) / len(input_stamps) if input_stamps else 0.0
    ground_status_counts = Counter(record.get("ground_status", "malformed")
                                   for record in ground_diagnostics)
    accepted_ground_candidates = sum(record.get("accepted") == "1"
                                     for record in ground_diagnostics)
    frame_reason_counts = Counter(record.get("reason", "malformed")
                                  for record in frame_diagnostics)
    expected_seq = list(range(len(input_order)))
    diagnostic_seq = [record["seq"] for record in frame_diagnostics]
    diagnostic_stamps = [record["stamp_ns"] for record in frame_diagnostics]
    diagnostic_order_ok = diagnostic_seq == expected_seq and diagnostic_stamps == input_order
    diagnostic_dropq_nonzero = [record for record in frame_diagnostics
                                if record.get("dropQ") != "0"]
    diagnostic_passthrough_mismatch = [
        record for record in frame_diagnostics
        if record.get("outcome") == "passthrough" and (
            record["stamp_ns"] not in input_messages
            or record["stamp_ns"] not in filtered_messages
            or digest(input_messages[record["stamp_ns"]])
            != digest(filtered_messages[record["stamp_ns"]])
        )
    ]
    inference_samples_ms = diagnostic_numbers(frame_diagnostics, "inference_ms")
    end_to_end_samples_ms = diagnostic_numbers(frame_diagnostics, "end_to_end_ms")
    input_duplicate_stamps = len(input_order) - len(set(input_order))
    raw_duplicate_stamps = len(raw_order) - len(set(raw_order))
    static_duplicate_stamps = len(static_order) - len(set(static_order))
    filtered_duplicate_stamps = len(filtered_order) - len(set(filtered_order))
    raw_order_matches_input = raw_order == input_order
    static_order_matches_input = static_order == input_order
    filtered_order_matches_input = filtered_order == input_order
    input_order_strict = all(later > earlier for earlier, later in zip(input_order, input_order[1:]))

    report = {
        "topic_counts": dict(counts),
        "input_cloud_frames": len(input_order),
        "raw_cloud_frames": len(raw_order),
        "static_cloud_frames": len(static_order),
        "filtered_cloud_frames": len(filtered_order),
        "unprocessed_input_frames": len(input_keys - raw_keys),
        "missing_static_frames": len(raw_keys - static_keys),
        "missing_filtered_frames": len(raw_keys - filtered_keys),
        "raw_input_mismatch_frames": len(raw_input_mismatch),
        "passthrough_mismatch_frames": len(passthrough_mismatch),
        "filtered_layout_failure_frames": len(filtered_layout_failures),
        "filtered_binary_record_failure_frames": len(filtered_record_failures),
        "filtered_equal_input_frames": len(filtered_equal_input),
        "filtered_changed_frames": len(filtered_removed_frames),
        "odom_messages": len(odom_stamps),
        "input_frames_with_usable_odom": usable_odom_frames,
        "odom_coverage": odom_coverage,
        "detected_box_nonempty_frames": box_frames,
        "detected_boxes": box_count,
        "moving_box_nonempty_frames": moving_box_frames,
        "moving_boxes": moving_box_count,
        "ground_diagnostic_records": len(ground_diagnostics),
        "ground_diagnostic_status_counts": dict(ground_status_counts),
        "ground_candidates_accepted_for_removal": accepted_ground_candidates,
        "imu_messages": counts[IMU],
        "input_duplicate_stamps": input_duplicate_stamps,
        "raw_duplicate_stamps": raw_duplicate_stamps,
        "static_duplicate_stamps": static_duplicate_stamps,
        "filtered_duplicate_stamps": filtered_duplicate_stamps,
        "raw_order_matches_input": raw_order_matches_input,
        "static_order_matches_input": static_order_matches_input,
        "filtered_order_matches_input": filtered_order_matches_input,
        "input_stamp_order_strict": input_order_strict,
        "frame_diagnostic_records": len(frame_diagnostics),
        "frame_diagnostic_reason_counts": dict(frame_reason_counts),
        "frame_diagnostic_order_ok": diagnostic_order_ok,
        "frame_diagnostic_dropq_nonzero": len(diagnostic_dropq_nonzero),
        "frame_diagnostic_passthrough_mismatch": len(diagnostic_passthrough_mismatch),
        "frame_diagnostic_inference_ms_mean": (
            sum(inference_samples_ms) / len(inference_samples_ms) if inference_samples_ms else None),
        "frame_diagnostic_inference_ms_max": max(inference_samples_ms) if inference_samples_ms else None,
        "frame_diagnostic_end_to_end_ms_mean": (
            sum(end_to_end_samples_ms) / len(end_to_end_samples_ms) if end_to_end_samples_ms else None),
        "frame_diagnostic_end_to_end_ms_max": max(end_to_end_samples_ms) if end_to_end_samples_ms else None,
        "moving_points_exported": len(moving_points),
        "protected_ground_points_exported": len(protected_points),
        "static_candidate_points_exported": len(candidate_points),
        "expect_no_removal": args.expect_no_removal,
        "expect_moving": args.expect_moving,
        "min_odom_coverage": args.min_odom_coverage,
        "require_frame_diagnostics": args.require_frame_diagnostics,
        "expect_frame_reasons": args.expect_frame_reason,
        "forbid_frame_reasons": args.forbid_frame_reason,
        "require_zero_dropq": args.require_zero_dropq,
    }
    report["pass"] = (
        bool(input_keys)
        and not (input_keys - raw_keys)
        and not (raw_keys - static_keys)
        and not (raw_keys - filtered_keys)
        and not raw_input_mismatch
        and not passthrough_mismatch
        and not filtered_layout_failures
        and not filtered_record_failures
        and not diagnostic_passthrough_mismatch
        and raw_order_matches_input
        and static_order_matches_input
        and filtered_order_matches_input
        and input_order_strict
        and (not args.expect_no_removal or not filtered_removed_frames)
        and (args.min_odom_coverage <= 0.0 or odom_coverage >= args.min_odom_coverage)
        and (not args.expect_moving or (
            moving_box_count > 0
            and len(filtered_removed_frames) > 0
            and len(moving_points) > 0
            and len(protected_points) > 0
        ))
        and (not args.require_frame_diagnostics or (
            len(frame_diagnostics) == len(input_order)
            and diagnostic_order_ok
        ))
        and (not args.require_zero_dropq or not diagnostic_dropq_nonzero)
        and all(frame_reason_counts[reason] > 0 for reason in args.expect_frame_reason)
        and all(frame_reason_counts[reason] == 0 for reason in args.forbid_frame_reason)
    )

    selected_stamps = sorted(stamp_ns for stamp_ns, message in moving_boxes_messages.items()
                             if message.boxes)
    if args.max_inspection_frames > 0:
        selected_stamps = selected_stamps[:args.max_inspection_frames]
        inspection_dir = write_inspection_frames(
            args.output_dir, selected_stamps, input_messages, filtered_messages,
            moving_messages, ground_messages, candidate_messages,
            boxes_messages, moving_boxes_messages, ground_diagnostics,
            sorted(odom_poses, key=lambda item: item[0]), int(args.world_odom_max_gap_sec * 1e9),
            args.lidar_to_imu_translation)
        report["inspection_frames_dir"] = inspection_dir
    report["inspection_frame_stamps_ns"] = selected_stamps

    write_pcd(os.path.join(args.output_dir, "moving_object_points_sample.pcd"), moving_points)
    write_pcd(os.path.join(args.output_dir, "protected_ground_points_sample.pcd"), protected_points)
    write_pcd(os.path.join(args.output_dir, "static_candidate_points_sample.pcd"), candidate_points)
    report_path = os.path.join(args.output_dir, "dynamic_removal_report.json")
    with open(report_path, "w", encoding="utf-8") as output:
        json.dump(report, output, ensure_ascii=False, indent=2)
        output.write("\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print("\nAcceptance report: {0}".format(report_path))
    if args.max_inspection_frames > 0:
        print("Inspection frames: {0}".format(report.get("inspection_frames_dir")))
    raise SystemExit(0 if report["pass"] else 2)


if __name__ == "__main__":
    main()
