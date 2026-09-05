#!/usr/bin/env python3
"""阶段 0/1：冻结场景证据输入并生成 map<-odom 校正。

稠密 RGB 缓存中的点已处在 FAST-LIVO2 前端的 ``camera_init``（odom）世界
坐标系。本程序绝不改写这些 PCD，而是为每帧记录一个按时间插值的校正：

    p_map = T_map_odom(vio_timestamp) * p_odom

保持源点云不可变，既避免再生成约 GB 级缓存，也使后续道路/场景阶段可复现。
程序只使用 Python 标准库。
"""

import argparse
import bisect
import csv
import hashlib
import json
import math
import shutil
import sys
from collections import namedtuple
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


EPS = 1e-9


class StageError(RuntimeError):
    """确定性的输入或输出契约错误。"""


Transform = namedtuple("Transform", "translation quaternion_xyzw")
TimedPose = namedtuple("TimedPose", "frame_id timestamp transform")
Observation = namedtuple("Observation", "frame_id timestamp point_count odom_imu source_pcd local_pcd image_topic lidar_topic")
PosePair = namedtuple("PosePair", "frame_id timestamp correction raw_odom_imu optimized_map_imu")


def fail(message: str) -> None:
    raise StageError(message)


def require_file(path: Path, label: str) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        fail(f"Missing or empty {label}: {path}")


def finite_number(text: str, field: str) -> float:
    try:
        value = float(text)
    except ValueError as error:
        raise StageError(f"Invalid {field}: {text!r}") from error
    if not math.isfinite(value):
        fail(f"Non-finite {field}: {text!r}")
    return value


def integer(text: str, field: str) -> int:
    try:
        value = int(text)
    except ValueError as error:
        raise StageError(f"Invalid {field}: {text!r}") from error
    if value < 0:
        fail(f"Negative {field}: {text!r}")
    return value


def normalize_quaternion(quaternion: Sequence[float]) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < EPS:
        fail("Zero quaternion")
    return tuple(value / norm for value in quaternion)


def quaternion_conjugate(quaternion: Sequence[float]) -> Tuple[float, float, float, float]:
    return (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def rotate(quaternion: Sequence[float], vector: Sequence[float]) -> Tuple[float, float, float]:
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compose(left: Transform, right: Transform) -> Transform:
    rotated = rotate(left.quaternion_xyzw, right.translation)
    return Transform(
        tuple(left.translation[index] + rotated[index] for index in range(3)),
        quaternion_multiply(left.quaternion_xyzw, right.quaternion_xyzw),
    )


def inverse(transform: Transform) -> Transform:
    inverse_q = quaternion_conjugate(transform.quaternion_xyzw)
    negated = tuple(-value for value in transform.translation)
    return Transform(rotate(inverse_q, negated), inverse_q)


def slerp(first: Sequence[float], second: Sequence[float], alpha: float) -> Tuple[float, float, float, float]:
    q0 = normalize_quaternion(first)
    q1 = normalize_quaternion(second)
    dot = sum(q0[index] * q1[index] for index in range(4))
    if dot < 0.0:
        q1 = tuple(-value for value in q1)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(tuple(q0[index] + alpha * (q1[index] - q0[index]) for index in range(4)))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    weight0 = math.sin((1.0 - alpha) * theta) / sin_theta
    weight1 = math.sin(alpha * theta) / sin_theta
    return normalize_quaternion(tuple(weight0 * q0[index] + weight1 * q1[index] for index in range(4)))


def interpolate(first: Transform, second: Transform, alpha: float) -> Transform:
    alpha = min(1.0, max(0.0, alpha))
    return Transform(
        tuple(first.translation[index] + alpha * (second.translation[index] - first.translation[index]) for index in range(3)),
        slerp(first.quaternion_xyzw, second.quaternion_xyzw, alpha),
    )


def translation_error(first: Transform, second: Transform) -> float:
    return math.sqrt(sum((first.translation[index] - second.translation[index]) ** 2 for index in range(3)))


def rotation_error_rad(first: Transform, second: Transform) -> float:
    dot = abs(sum(first.quaternion_xyzw[index] * second.quaternion_xyzw[index] for index in range(4)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def correction_magnitude(transform: Transform) -> Tuple[float, float]:
    translation = math.sqrt(sum(value * value for value in transform.translation))
    rotation = 2.0 * math.acos(min(1.0, max(-1.0, abs(transform.quaternion_xyzw[3]))))
    return translation, rotation


def parse_pose_file(path: Path) -> Dict[int, TimedPose]:
    poses: Dict[int, TimedPose] = {}
    with path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            values = line.split()
            if not values or values[0].startswith("#"):
                continue
            if len(values) != 9:
                fail(f"Invalid pose line {path}:{line_number}; expected 9 fields")
            frame_id = integer(values[0], "keyframe id")
            if frame_id in poses:
                fail(f"Duplicate keyframe id {frame_id} in {path}")
            timestamp = finite_number(values[1], "keyframe timestamp")
            transform = Transform(
                tuple(finite_number(values[index], "keyframe translation") for index in range(2, 5)),
                normalize_quaternion(tuple(finite_number(values[index], "keyframe quaternion") for index in range(5, 9))),
            )
            poses[frame_id] = TimedPose(frame_id, timestamp, transform)
    if not poses:
        fail(f"No keyframe poses in {path}")
    return poses


def parse_pcd_point_count(path: Path) -> int:
    points = None
    with path.open("rb") as source:
        for _ in range(64):
            line = source.readline()
            if not line:
                break
            try:
                tokens = line.decode("ascii").strip().split()
            except UnicodeDecodeError as error:
                raise StageError(f"Invalid PCD header encoding: {path}") from error
            if not tokens:
                continue
            if tokens[0] == "POINTS" and len(tokens) == 2:
                points = integer(tokens[1], "PCD POINTS")
            if tokens[0] == "DATA":
                if len(tokens) != 2 or tokens[1] != "binary":
                    fail(f"Only binary PCD is supported: {path}")
                break
        else:
            fail(f"PCD header too long: {path}")
    if points is None:
        fail(f"PCD POINTS header missing: {path}")
    return points


def parse_cache_manifest(path: Path, run_dir: Path) -> Dict[int, Tuple[float, int, Path]]:
    result: Dict[int, Tuple[float, int, Path]] = {}
    with path.open("r", encoding="utf-8", newline="") as source:
        reader = csv.DictReader(source)
        expected = {"frame_id", "timestamp", "point_count", "pcd_path"}
        if set(reader.fieldnames or []) != expected:
            fail(f"Unexpected cache manifest header in {path}: {reader.fieldnames}")
        for line_number, row in enumerate(reader, 2):
            frame_id = integer(row["frame_id"], "cache frame id")
            if frame_id in result:
                fail(f"Duplicate cache frame id {frame_id} at {path}:{line_number}")
            filename = Path(row["pcd_path"]).name
            expected_name = f"frame_{frame_id:06d}.pcd"
            if filename != expected_name:
                fail(f"Cache path does not match frame id {frame_id}: {row['pcd_path']}")
            local_path = run_dir / "dense_rgb_cache" / "frames" / filename
            result[frame_id] = (finite_number(row["timestamp"], "cache timestamp"), integer(row["point_count"], "cache point count"), local_path)
    if not result:
        fail(f"No cached frames in {path}")
    return result


def parse_observations(path: Path, run_dir: Path, cache: Dict[int, Tuple[float, int, Path]]) -> List[Observation]:
    expected = [
        "frame_id", "vio_timestamp", "point_count", "odom_imu_tx", "odom_imu_ty", "odom_imu_tz",
        "odom_imu_qx", "odom_imu_qy", "odom_imu_qz", "odom_imu_qw", "rgb_cache_pcd",
        "source_image_topic", "source_lidar_topic",
    ]
    observations: List[Observation] = []
    seen = set()
    with path.open("r", encoding="utf-8", newline="") as source:
        reader = csv.DictReader(source)
        if reader.fieldnames != expected:
            fail(f"Unexpected observation manifest header in {path}: {reader.fieldnames}")
        for line_number, row in enumerate(reader, 2):
            frame_id = integer(row["frame_id"], "observation frame id")
            if frame_id in seen:
                fail(f"Duplicate observation frame id {frame_id} at {path}:{line_number}")
            seen.add(frame_id)
            if frame_id not in cache:
                fail(f"Observation frame {frame_id} is absent from cache manifest")
            timestamp = finite_number(row["vio_timestamp"], "VIO timestamp")
            point_count = integer(row["point_count"], "observation point count")
            cache_timestamp, cache_count, cache_path = cache[frame_id]
            if abs(timestamp - cache_timestamp) > 1e-6 or point_count != cache_count:
                fail(f"Observation/cache mismatch for frame {frame_id}")
            filename = Path(row["rgb_cache_pcd"]).name
            if filename != cache_path.name:
                fail(f"Observation path does not match cache manifest for frame {frame_id}")
            odom_imu = Transform(
                tuple(finite_number(row[field], field) for field in ("odom_imu_tx", "odom_imu_ty", "odom_imu_tz")),
                normalize_quaternion(tuple(finite_number(row[field], field) for field in ("odom_imu_qx", "odom_imu_qy", "odom_imu_qz", "odom_imu_qw"))),
            )
            observations.append(Observation(
                frame_id, timestamp, point_count, odom_imu, row["rgb_cache_pcd"], cache_path,
                row["source_image_topic"], row["source_lidar_topic"],
            ))
    if len(seen) != len(cache):
        fail(f"Observation/cache cardinality mismatch: {len(seen)} observations, {len(cache)} cache entries")
    observations.sort(key=lambda item: item.timestamp)
    for previous, current in zip(observations, observations[1:]):
        if current.timestamp <= previous.timestamp:
            fail(f"Observation timestamps are not strictly increasing at frame {current.frame_id}")
    return observations


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, payload: object) -> None:
    with path.open("w", encoding="utf-8") as destination:
        json.dump(payload, destination, indent=2, sort_keys=True)
        destination.write("\n")


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as destination:
        writer = csv.DictWriter(destination, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def build_pose_pairs(raw: Dict[int, TimedPose], optimized: Dict[int, TimedPose]) -> List[PosePair]:
    if set(raw) != set(optimized):
        missing_optimized = sorted(set(raw) - set(optimized))
        missing_raw = sorted(set(optimized) - set(raw))
        fail(f"Raw/optimized keyframe ids differ; missing optimized={missing_optimized[:5]}, missing raw={missing_raw[:5]}")
    pairs: List[PosePair] = []
    for frame_id in raw:
        raw_pose, optimized_pose = raw[frame_id], optimized[frame_id]
        if abs(raw_pose.timestamp - optimized_pose.timestamp) > 1e-6:
            fail(f"Raw/optimized timestamp mismatch for keyframe {frame_id}")
        pairs.append(PosePair(
            frame_id, raw_pose.timestamp,
            compose(optimized_pose.transform, inverse(raw_pose.transform)),
            raw_pose.transform, optimized_pose.transform,
        ))
    pairs.sort(key=lambda item: item.timestamp)
    for previous, current in zip(pairs, pairs[1:]):
        if current.timestamp <= previous.timestamp:
            fail(f"Keyframe timestamps are not strictly increasing at id {current.frame_id}")
    return pairs


def correction_at(timestamp: float, pairs: Sequence[PosePair], timestamps: Sequence[float]) -> Tuple[Transform, str, PosePair, PosePair, float, float]:
    position = bisect.bisect_right(timestamps, timestamp)
    if position == 0:
        pair = pairs[0]
        return pair.correction, "clamped_before_first_keyframe", pair, pair, 0.0, pair.timestamp - timestamp
    if position == len(pairs):
        pair = pairs[-1]
        return pair.correction, "clamped_after_last_keyframe", pair, pair, 0.0, timestamp - pair.timestamp
    lower, upper = pairs[position - 1], pairs[position]
    alpha = (timestamp - lower.timestamp) / (upper.timestamp - lower.timestamp)
    nearest_delta = min(timestamp - lower.timestamp, upper.timestamp - timestamp)
    return interpolate(lower.correction, upper.correction, alpha), "interpolated", lower, upper, alpha, nearest_delta


def transform_dict(prefix: str, transform: Transform) -> Dict[str, float]:
    return {
        f"{prefix}_tx": transform.translation[0], f"{prefix}_ty": transform.translation[1], f"{prefix}_tz": transform.translation[2],
        f"{prefix}_qx": transform.quaternion_xyzw[0], f"{prefix}_qy": transform.quaternion_xyzw[1],
        f"{prefix}_qz": transform.quaternion_xyzw[2], f"{prefix}_qw": transform.quaternion_xyzw[3],
    }


def run(arguments: argparse.Namespace) -> None:
    run_dir = Path(arguments.run_dir).expanduser().resolve()
    output_dir = Path(arguments.output).expanduser().resolve()
    if not run_dir.is_dir():
        fail(f"Run directory does not exist: {run_dir}")
    if output_dir.exists():
        fail(f"Output already exists; choose a new directory: {output_dir}")
    if output_dir == run_dir or run_dir not in output_dir.parents:
        fail("Output must be a new child directory of --run-dir to preserve a relocatable package")

    input_paths = {
        "scene_metadata": run_dir / "scene_evidence" / "metadata.yaml",
        "observations": run_dir / "scene_evidence" / "frame_observations.csv",
        "cache_manifest": run_dir / "dense_rgb_cache" / "manifest.csv",
        "raw_keyframes": run_dir / "keyframes" / "keyframe_poses_imu.txt",
        "optimized_keyframes": run_dir / "loop_backend" / "optimized_keyframe_poses_imu.txt",
        "final_dense_map": run_dir / "pcd" / "all_global_optimized_rgb_dense_full.pcd",
    }
    for label, path in input_paths.items():
        require_file(path, label)

    cache = parse_cache_manifest(input_paths["cache_manifest"], run_dir)
    observations = parse_observations(input_paths["observations"], run_dir, cache)
    raw = parse_pose_file(input_paths["raw_keyframes"])
    optimized = parse_pose_file(input_paths["optimized_keyframes"])
    pairs = build_pose_pairs(raw, optimized)

    frame_bytes = 0
    for observation in observations:
        require_file(observation.local_pcd, f"cache PCD frame {observation.frame_id}")
        if parse_pcd_point_count(observation.local_pcd) != observation.point_count:
            fail(f"PCD header point count mismatch for frame {observation.frame_id}")
        frame_bytes += observation.local_pcd.stat().st_size
    total_points = sum(observation.point_count for observation in observations)
    final_points = parse_pcd_point_count(input_paths["final_dense_map"])
    if total_points != final_points:
        fail(f"Cache point total ({total_points}) does not match final dense map POINTS ({final_points})")

    output_dir.mkdir(parents=True)
    inventory_dir = output_dir / "input_inventory"
    correction_dir = output_dir / "pose_correction"
    validation_dir = output_dir / "validation"
    inventory_dir.mkdir()
    correction_dir.mkdir()
    validation_dir.mkdir()

    fingerprints = {
        label: {"path_relative_to_run": str(path.relative_to(run_dir)), "bytes": path.stat().st_size, "sha256": sha256(path)}
        for label, path in input_paths.items()
    }
    write_json(inventory_dir / "input_manifest.json", {
        "schema": "fast_livo_scene_pipeline_input_inventory/v1",
        "run_dir_name": run_dir.name,
        "source_policy": "Inputs are read-only. Cached PCD paths are normalized to paths relative to run_dir.",
        "fingerprints": fingerprints,
        "frame_cache": {"frame_count": len(observations), "total_points": total_points, "total_bytes": frame_bytes},
        "final_dense_map_points": final_points,
        "keyframe_count": len(pairs),
    })

    normalized_fields = [
        "frame_id", "vio_timestamp", "point_count", "odom_imu_tx", "odom_imu_ty", "odom_imu_tz",
        "odom_imu_qx", "odom_imu_qy", "odom_imu_qz", "odom_imu_qw", "rgb_cache_pcd",
        "source_image_topic", "source_lidar_topic",
    ]
    write_csv(inventory_dir / "normalized_frame_observations.csv", normalized_fields, (
        {
            "frame_id": observation.frame_id, "vio_timestamp": f"{observation.timestamp:.9f}", "point_count": observation.point_count,
            **transform_dict("odom_imu", observation.odom_imu),
            "rgb_cache_pcd": str(observation.local_pcd.relative_to(run_dir)),
            "source_image_topic": observation.image_topic, "source_lidar_topic": observation.lidar_topic,
        }
        for observation in observations
    ))

    timestamps = [pair.timestamp for pair in pairs]
    corrections: List[Dict[str, object]] = []
    support_counts = {"interpolated": 0, "clamped_before_first_keyframe": 0, "clamped_after_last_keyframe": 0}
    correction_translation: List[float] = []
    correction_rotation: List[float] = []
    for observation in observations:
        correction, support, lower, upper, alpha, nearest_delta = correction_at(observation.timestamp, pairs, timestamps)
        map_imu = compose(correction, observation.odom_imu)
        support_counts[support] += 1
        magnitude_translation, magnitude_rotation = correction_magnitude(correction)
        correction_translation.append(magnitude_translation)
        correction_rotation.append(magnitude_rotation)
        corrections.append({
            "frame_id": observation.frame_id, "vio_timestamp": f"{observation.timestamp:.9f}", "point_count": observation.point_count,
            "cache_pcd": str(observation.local_pcd.relative_to(run_dir)),
            **transform_dict("odom_imu", observation.odom_imu),
            **transform_dict("map_odom", correction),
            **transform_dict("map_imu", map_imu),
            "support": support, "lower_keyframe_id": lower.frame_id, "upper_keyframe_id": upper.frame_id,
            "interpolation_alpha": f"{alpha:.12f}", "nearest_keyframe_dt_s": f"{nearest_delta:.9f}",
        })
    correction_fields = list(corrections[0].keys())
    write_csv(correction_dir / "frame_map_corrections.csv", correction_fields, corrections)

    residuals: List[Dict[str, object]] = []
    keyframe_corrections: List[Dict[str, object]] = []
    max_translation_residual = 0.0
    max_rotation_residual = 0.0
    for pair in pairs:
        reconstructed = compose(pair.correction, pair.raw_odom_imu)
        translation_residual = translation_error(reconstructed, pair.optimized_map_imu)
        rotation_residual = rotation_error_rad(reconstructed, pair.optimized_map_imu)
        max_translation_residual = max(max_translation_residual, translation_residual)
        max_rotation_residual = max(max_rotation_residual, rotation_residual)
        keyframe_corrections.append({
            "keyframe_id": pair.frame_id, "timestamp": f"{pair.timestamp:.9f}",
            **transform_dict("map_odom", pair.correction),
        })
        residuals.append({"keyframe_id": pair.frame_id, "translation_residual_m": f"{translation_residual:.12g}", "rotation_residual_rad": f"{rotation_residual:.12g}"})
    write_csv(correction_dir / "keyframe_map_odom_corrections.csv", list(keyframe_corrections[0].keys()), keyframe_corrections)
    write_csv(validation_dir / "keyframe_correction_residuals.csv", list(residuals[0].keys()), residuals)

    write_json(validation_dir / "stage01_pose_correction_report.json", {
        "schema": "fast_livo_scene_pipeline_stage01_pose_correction/v1",
        "transform_definition": "p_map = T_map_odom(vio_timestamp) * p_odom; cached PCD points remain immutable in odom/camera_init.",
        "interpolation": "translation linear interpolation and shortest-path quaternion SLERP between consecutive raw/optimized keyframe correction pairs; endpoint clamping outside support.",
        "frame_count": len(observations),
        "keyframe_count": len(pairs),
        "observation_time_start": observations[0].timestamp,
        "observation_time_end": observations[-1].timestamp,
        "keyframe_time_start": pairs[0].timestamp,
        "keyframe_time_end": pairs[-1].timestamp,
        "support_counts": support_counts,
        "cache_total_points": total_points,
        "final_dense_map_points": final_points,
        "cache_total_bytes": frame_bytes,
        "max_keyframe_reconstruction_translation_residual_m": max_translation_residual,
        "max_keyframe_reconstruction_rotation_residual_rad": max_rotation_residual,
        "correction_translation_m": {"min": min(correction_translation), "max": max(correction_translation)},
        "correction_rotation_rad": {"min": min(correction_rotation), "max": max(correction_rotation)},
        "next_stage_input": "input_inventory/normalized_frame_observations.csv + pose_correction/frame_map_corrections.csv + immutable dense_rgb_cache/frames/*.pcd",
    })
    shutil.copy2(input_paths["scene_metadata"], inventory_dir / "source_scene_metadata.yaml")
    write_json(output_dir / "stage01_complete.json", {
        "status": "complete",
        "stage": "0_input_inventory_and_1_pose_correction",
        "run_dir": str(run_dir),
        "read_only_input": True,
        "corrected_pcd_policy": "lazy_transform_only",
    })
    print(f"[scene-stage01] complete: frames={len(observations)}, keyframes={len(pairs)}, output={output_dir}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Freeze scene-evidence inputs and write per-frame map<-odom corrections.")
    parser.add_argument("--run-dir", required=True, help="Existing scene_evidence run directory.")
    parser.add_argument("--output", help="New output directory; default: RUN_DIR/scene_pipeline_v4/stage01_pose_correction")
    arguments = parser.parse_args()
    if not arguments.output:
        arguments.output = str(Path(arguments.run_dir) / "scene_pipeline_v4" / "stage01_pose_correction")
    return arguments


def main() -> int:
    try:
        run(parse_arguments())
        return 0
    except StageError as error:
        print(f"[scene-stage01] {error}", file=sys.stderr)
        return 2
    except OSError as error:
        print(f"[scene-stage01] filesystem error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
