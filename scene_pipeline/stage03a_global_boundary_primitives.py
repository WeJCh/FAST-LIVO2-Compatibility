#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 3A：从硬高差图提取地图尺度的道路边界带，并按轨迹双边配对。

该阶段故意不把点按局部连通性染成碎段。先从局部线性点的切向生成霍夫投票，再在
全局尺度上拟合最长共线支持，并仅在同一拟合线内部跨越短缺口。输出仍是“边界
几何基元”，不是路沿、围墙或道路语义。

在全局直线基元之后，本脚本还会：
1. 合并同一条物理边界上、方向和法向位置相容的碎段，形成“边界带”；
2. 在校正后的轨迹横断面上寻找稳定的左、右第一候选边界带；
3. 只将被同一轨迹区段双边配对支持的边界带写入最终 PCD。

它不能从单侧观测推断未采集道路，也不把未配对的围墙/绿化边界误称为道路边界。
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

from common import boundary_geometry as local_edges
from common import geometry as base


ANGLE_BINS = 180       # 1 度法向量桶。
RHO_M = 0.20           # 与阶段 2 网格一致的距离桶。
LINE_RESIDUAL_M = 0.18
# 本数据的道路路沿存在较长但同向的观测空缺；该阶段优先保证道路边界连续性。
GAP_SPLIT_M = 3.00
MIN_LENGTH_M = 2.00
MIN_INLIERS = 8

# 边界带合并只用于修复同一条长路沿/绿化边的观测碎裂。以下三项共同限制，
# 不能仅因两条线平行就合并（道路两侧、围墙和路沿常常也平行）。
BAND_DIRECTION_COS = math.cos(math.radians(5.0))
BAND_NORMAL_OFFSET_M = 0.22
BAND_ENDPOINT_GAP_M = 5.00

# 轨迹双边配对参数。较长的路口开口允许基元先合并，但只有在一批连续横断面上
# 同时看到左右边界时才会成为最终输出，避免“单条围墙线”被误作道路边界。
TRAJECTORY_SAMPLE_SPACING_M = 0.50
PAIR_MAX_LATERAL_M = 10.0
PAIR_MIN_WIDTH_M = 1.20
PAIR_MAX_WIDTH_M = 12.0
PAIR_MIN_SUPPORT_SAMPLES = 6
PAIR_DIRECTION_COS = math.cos(math.radians(35.0))


def normalized_angle(angle):
    while angle < 0.0:
        angle += math.pi
    while angle >= math.pi:
        angle -= math.pi
    return angle


def fit_line(points, point_ids):
    """TLS 二维直线拟合，返回单位切向、法向和法向截距。"""
    mean_x = sum(points[item][0] for item in point_ids) / len(point_ids)
    mean_y = sum(points[item][1] for item in point_ids) / len(point_ids)
    xx = sum((points[item][0] - mean_x) ** 2 for item in point_ids)
    yy = sum((points[item][1] - mean_y) ** 2 for item in point_ids)
    xy = sum((points[item][0] - mean_x) * (points[item][1] - mean_y) for item in point_ids)
    largest = (xx + yy + math.sqrt(max(0.0, (xx - yy) ** 2 + 4.0 * xy * xy))) * 0.5
    tangent_x, tangent_y = xy, largest - xx
    norm = math.hypot(tangent_x, tangent_y)
    if norm < 1e-9:
        tangent_x, tangent_y = 1.0, 0.0
    else:
        tangent_x /= norm
        tangent_y /= norm
    # 保证 CSV 中同向直线的切向符号稳定，便于后续合并。
    if tangent_x < 0.0 or (abs(tangent_x) < 1e-9 and tangent_y < 0.0):
        tangent_x, tangent_y = -tangent_x, -tangent_y
    normal_x, normal_y = -tangent_y, tangent_x
    rho = normal_x * mean_x + normal_y * mean_y
    return tangent_x, tangent_y, normal_x, normal_y, rho, mean_x, mean_y


def projection(point, tangent_x, tangent_y):
    return point[0] * tangent_x + point[1] * tangent_y


def line_distance(point, normal_x, normal_y, rho):
    return abs(normal_x * point[0] + normal_y * point[1] - rho)


def hough_peak(points, tangents, active):
    """对每个点只在自身切向附近投票，避免全角度暴力霍夫造成噪声峰。"""
    accumulator = defaultdict(int)
    for point_id in active:
        tangent_x, tangent_y = tangents[point_id]
        normal_angle = normalized_angle(math.atan2(tangent_y, tangent_x) + math.pi * 0.5)
        center_bin = int(round(normal_angle / math.pi * ANGLE_BINS)) % ANGLE_BINS
        point = points[point_id]
        for offset in range(-3, 4):
            theta_bin = (center_bin + offset) % ANGLE_BINS
            theta = theta_bin * math.pi / ANGLE_BINS
            rho_bin = int(round((point[0] * math.cos(theta) + point[1] * math.sin(theta)) / RHO_M))
            accumulator[(theta_bin, rho_bin)] += 1
    if not accumulator:
        return None
    (theta_bin, rho_bin), count = max(accumulator.items(), key=lambda item: item[1])
    theta = theta_bin * math.pi / ANGLE_BINS
    return math.cos(theta), math.sin(theta), rho_bin * RHO_M, count


def candidate_inliers(points, tangents, active, normal_x, normal_y, rho):
    tangent_x, tangent_y = normal_y, -normal_x
    result = []
    for point_id in active:
        local_tangent = tangents[point_id]
        if abs(local_tangent[0] * tangent_x + local_tangent[1] * tangent_y) < 0.75:
            continue
        if line_distance(points[point_id], normal_x, normal_y, rho) <= 0.30:
            result.append(point_id)
    return result


def robust_fit(points, tangents, active, initial):
    current = initial
    for _iteration in range(3):
        if len(current) < MIN_INLIERS:
            return None
        tangent_x, tangent_y, normal_x, normal_y, rho, _x, _y = fit_line(points, current)
        refined = []
        for point_id in active:
            local_tangent = tangents[point_id]
            if abs(local_tangent[0] * tangent_x + local_tangent[1] * tangent_y) < 0.70:
                continue
            if line_distance(points[point_id], normal_x, normal_y, rho) <= LINE_RESIDUAL_M:
                refined.append(point_id)
        if set(refined) == set(current):
            break
        current = refined
    if len(current) < MIN_INLIERS:
        return None
    return fit_line(points, current), current


def split_by_gap(points, point_ids, tangent_x, tangent_y):
    ordered = sorted(point_ids, key=lambda item: projection(points[item], tangent_x, tangent_y))
    groups = []
    current = []
    previous = None
    for point_id in ordered:
        current_projection = projection(points[point_id], tangent_x, tangent_y)
        if previous is not None and current_projection - previous > GAP_SPLIT_M:
            if current:
                groups.append(current)
            current = []
        current.append(point_id)
        previous = current_projection
    if current:
        groups.append(current)
    return groups


def primitive_color(primitive_id):
    palette = ((245, 139, 45), (52, 190, 215), (220, 100, 220),
               (110, 205, 90), (245, 220, 70), (240, 85, 85))
    return base.packed_rgb(*palette[primitive_id % len(palette)])


def write_line_pcd(path, primitive_points):
    base.write_pcd(path, primitive_points)


class DisjointSet(object):
    """极小并查集：只合并满足全部几何约束的全局直线碎段。"""
    def __init__(self, size):
        self.parent = list(range(size))

    def find(self, item):
        while self.parent[item] != item:
            self.parent[item] = self.parent[self.parent[item]]
            item = self.parent[item]
        return item

    def union(self, first, second):
        first = self.find(first)
        second = self.find(second)
        if first != second:
            self.parent[second] = first


def boundary_fragment_join_score(first, second):
    """返回首尾相接碎段的合并代价；不相容时返回 None。

    这里有意拒绝大面积投影重叠的平行线。它们更可能是路沿两侧、台阶边、
    人行道外缘或贴近围墙，而不是同一条边界的观测缺口。
    """
    direction_dot = abs(first['tx'] * second['tx'] + first['ty'] * second['ty'])
    if direction_dot < BAND_DIRECTION_COS:
        return None
    # 用第一段法向量比较第二条支撑线的位置；法向量可能反向，取绝对值。
    normal_offset = abs(first['nx'] * second['mean_x'] + first['ny'] * second['mean_y'] - first['rho'])
    if normal_offset > BAND_NORMAL_OFFSET_M:
        return None
    # 投影到第一段的方向。区间重叠或端点缺口不超过阈值才可合并。
    first_min, first_max = first['start'], first['end']
    second_values = []
    anchor_x, anchor_y = second['nx'] * second['rho'], second['ny'] * second['rho']
    for coordinate in (second['start'], second['end']):
        second_values.append(projection((anchor_x + second['tx'] * coordinate,
                                         anchor_y + second['ty'] * coordinate),
                                        first['tx'], first['ty']))
    second_min, second_max = min(second_values), max(second_values)
    # 只接受 first 的末端到 second 的起点。方向由 fit_line 统一到固定半平面，
    # 因而该“前后”关系在同向直线间稳定；反向关系将在遍历 second, first 时检查。
    gap = second_min - first_max
    if gap < -0.25 or gap > BAND_ENDPOINT_GAP_M:
        return None
    return gap + normal_offset * 2.0 + (1.0 - direction_dot) * 5.0


def merge_boundary_bands(points, primitives):
    """只合并互为最近的首尾碎段，避免链式吞并相邻平行实体。"""
    union_find = DisjointSet(len(primitives))
    successors = {}
    predecessors = {}
    for first_id, first in enumerate(primitives):
        for second_id in range(first_id + 1, len(primitives)):
            score = boundary_fragment_join_score(first, primitives[second_id])
            if score is not None:
                existing = successors.get(first_id)
                if existing is None or score < existing[0]:
                    successors[first_id] = (score, second_id)
                existing = predecessors.get(second_id)
                if existing is None or score < existing[0]:
                    predecessors[second_id] = (score, first_id)
            # 由于编号并不反映几何方向，也同时检查 second 接到 first 的关系。
            reverse_score = boundary_fragment_join_score(primitives[second_id], first)
            if reverse_score is not None:
                existing = successors.get(second_id)
                if existing is None or reverse_score < existing[0]:
                    successors[second_id] = (reverse_score, first_id)
                existing = predecessors.get(first_id)
                if existing is None or reverse_score < existing[0]:
                    predecessors[first_id] = (reverse_score, second_id)
    # 只有 A 的最佳后继是 B 且 B 的最佳前驱也是 A 时才合并，消除一对多连接。
    for first_id, (_score, second_id) in successors.items():
        predecessor = predecessors.get(second_id)
        if predecessor is not None and predecessor[1] == first_id:
            union_find.union(first_id, second_id)
    groups = defaultdict(list)
    for primitive_id in range(len(primitives)):
        groups[union_find.find(primitive_id)].append(primitive_id)
    bands = []
    for fragment_ids in groups.values():
        members = []
        for fragment_id in fragment_ids:
            members.extend(primitives[fragment_id]['members'])
        line = fit_line(points, members)
        tangent_x, tangent_y, normal_x, normal_y, rho, mean_x, mean_y = line
        projections = [projection(points[item], tangent_x, tangent_y) for item in members]
        start, end = min(projections), max(projections)
        bands.append({
            'id': len(bands), 'fragment_ids': sorted(fragment_ids), 'members': members,
            'tx': tangent_x, 'ty': tangent_y, 'nx': normal_x, 'ny': normal_y,
            'rho': rho, 'start': start, 'end': end, 'length': end - start,
            'mean_x': mean_x, 'mean_y': mean_y,
            'residual': sum(line_distance(points[item], normal_x, normal_y, rho)
                            for item in members) / len(members),
        })
    # 保持 band_id 与列表下标一致，便于轨迹横断面直接引用。
    return bands


def read_corrected_trajectory(stage02):
    """阶段 2 的父目录中读取阶段 1 的回环校正后轨迹。"""
    pipeline_root = os.path.dirname(os.path.abspath(stage02))
    source = os.path.join(pipeline_root, 'stage01_pose_correction', 'pose_correction',
                          'frame_map_corrections.csv')
    if not os.path.isfile(source):
        base.fail('缺少阶段 1 校正轨迹，无法做双边配对：' + source)
    return base.read_trajectory(source), source


def nearest_band_at_cross_section(sample, side, bands):
    """从一条横断面选择最近、方向相容且投影落在边界带范围内的边界。"""
    candidates = boundary_bands_at_cross_section(sample, side, bands)
    return None if not candidates else candidates[0][1]


def boundary_bands_at_cross_section(sample, side, bands):
    """返回横断面同一侧由近至远的所有方向相容边界带。"""
    candidates = []
    for band in bands:
        direction_dot = abs(sample['tx'] * band['tx'] + sample['ty'] * band['ty'])
        if direction_dot < PAIR_DIRECTION_COS:
            continue
        coordinate = projection((sample['x'], sample['y']), band['tx'], band['ty'])
        if coordinate < band['start'] - 1.0 or coordinate > band['end'] + 1.0:
            continue
        signed = band['nx'] * sample['x'] + band['ny'] * sample['y'] - band['rho']
        # 轨迹局部左法线与边界方向无关，直接用边界最近点相对横断面的左右符号。
        closest_x = sample['x'] - signed * band['nx']
        closest_y = sample['y'] - signed * band['ny']
        lateral = ((closest_x - sample['x']) * sample['nx'] +
                   (closest_y - sample['y']) * sample['ny'])
        if side * lateral <= 0.0:
            continue
        distance = abs(lateral)
        if distance > PAIR_MAX_LATERAL_M:
            continue
        candidates.append((distance, band['id']))
    return sorted(candidates)


def longest_continuous_sample_run(sample_ids):
    """轨迹采样间允许一个缺失点；避免离散偶遇被误认为稳定边界角色。"""
    if not sample_ids:
        return 0
    ordered = sorted(set(sample_ids))
    longest = current = 1
    for previous, current_id in zip(ordered, ordered[1:]):
        if current_id - previous <= 2:
            current += 1
        else:
            current = 1
        longest = max(longest, current)
    return longest


def classify_boundary_hierarchy(trajectory, bands):
    """标记轨迹两侧最近边界及其外侧第二边界，但绝不据此删除其它几何带。

    最近边界是道路核心的优先候选；第二边界只在与第一边界相隔合理距离时标作
    人行道/路肩外缘候选。两者都是“候选角色”，不是最终语义标签。
    """
    samples = base.resample_trajectory(trajectory, TRAJECTORY_SAMPLE_SPACING_M)
    primary_support = defaultdict(list)
    secondary_support = defaultdict(list)
    for sample_id, sample in enumerate(samples):
        for side in (1.0, -1.0):
            candidates = boundary_bands_at_cross_section(sample, side, bands)
            if not candidates:
                continue
            primary_support[candidates[0][1]].append(sample_id)
            if len(candidates) < 2:
                continue
            first_distance = candidates[0][0]
            second_distance, second_id = candidates[1]
            # 第二边界必须在第一边界之外且间隔至少一个路沿尺度；不把近重叠的
            # 拟合碎段再解释成“人行道”。上限防止远处立面直接变成相邻边界。
            if 0.45 <= second_distance - first_distance <= 6.0:
                secondary_support[second_id].append(sample_id)
    roles = {}
    for band in bands:
        band_id = band['id']
        primary_run = longest_continuous_sample_run(primary_support[band_id])
        secondary_run = longest_continuous_sample_run(secondary_support[band_id])
        if primary_run >= PAIR_MIN_SUPPORT_SAMPLES:
            role = 'primary_road_side_candidate'
        elif secondary_run >= PAIR_MIN_SUPPORT_SAMPLES:
            role = 'secondary_sidewalk_outer_candidate'
        else:
            role = 'unassociated_boundary_geometry'
        roles[band_id] = {
            'role': role,
            'primary_support_samples': len(set(primary_support[band_id])),
            'primary_longest_run': primary_run,
            'secondary_support_samples': len(set(secondary_support[band_id])),
            'secondary_longest_run': secondary_run,
        }
    return samples, roles


def pair_boundary_bands(trajectory, bands):
    """在连续轨迹横断面统计左右最近边界带，输出有充分共同支持的配对。"""
    samples = base.resample_trajectory(trajectory, TRAJECTORY_SAMPLE_SPACING_M)
    support = defaultdict(list)
    for sample_index, sample in enumerate(samples):
        left_id = nearest_band_at_cross_section(sample, 1.0, bands)
        right_id = nearest_band_at_cross_section(sample, -1.0, bands)
        if left_id is None or right_id is None or left_id == right_id:
            continue
        left = bands[left_id]
        right = bands[right_id]
        # 同一横断面上必须是近似平行的两条带，并且道路宽度处于合理范围。
        if abs(left['tx'] * right['tx'] + left['ty'] * right['ty']) < BAND_DIRECTION_COS:
            continue
        left_coordinate = projection((sample['x'], sample['y']), left['tx'], left['ty'])
        right_coordinate = projection((sample['x'], sample['y']), right['tx'], right['ty'])
        left_signed = left['nx'] * sample['x'] + left['ny'] * sample['y'] - left['rho']
        right_signed = right['nx'] * sample['x'] + right['ny'] * sample['y'] - right['rho']
        left_x = sample['x'] - left_signed * left['nx']
        left_y = sample['y'] - left_signed * left['ny']
        right_x = sample['x'] - right_signed * right['nx']
        right_y = sample['y'] - right_signed * right['ny']
        width = math.hypot(left_x - right_x, left_y - right_y)
        if PAIR_MIN_WIDTH_M <= width <= PAIR_MAX_WIDTH_M:
            support[(left_id, right_id)].append((sample_index, width,
                                                  left_coordinate, right_coordinate))
    pairs = []
    for (left_id, right_id), values in support.items():
        if len(values) < PAIR_MIN_SUPPORT_SAMPLES:
            continue
        values.sort()
        # 单次从远处扫到两条线不能构成道路；至少需要连续约 3 m 的共同横断面支持。
        longest = current = 1
        for previous, current_value in zip(values, values[1:]):
            if current_value[0] - previous[0] <= 2:
                current += 1
            else:
                current = 1
            longest = max(longest, current)
        if longest < PAIR_MIN_SUPPORT_SAMPLES:
            continue
        pairs.append({
            'id': len(pairs), 'left_band_id': left_id, 'right_band_id': right_id,
            'support_samples': len(values), 'longest_continuous_samples': longest,
            'mean_width': sum(item[1] for item in values) / len(values),
        })
    return samples, pairs


def append_band_fitted_points(band, color, output_points):
    """以 10 cm 间隔输出一条合并边界带的完整拟合线。"""
    anchor_x, anchor_y = band['nx'] * band['rho'], band['ny'] * band['rho']
    z = sum(item[2] for item in band['member_points']) / len(band['member_points'])
    for step in range(int(math.floor(band['length'] / 0.10)) + 1):
        coordinate = band['start'] + step * 0.10
        output_points.append((anchor_x + band['tx'] * coordinate,
                              anchor_y + band['ty'] * coordinate, z, color))


def main():
    parser = argparse.ArgumentParser(description='阶段 3A：地图尺度全局边界直线基元')
    parser.add_argument('--stage02', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    stage02 = os.path.abspath(args.stage02)
    output = os.path.abspath(args.output)
    if not os.path.isdir(stage02):
        base.fail('阶段 2 目录不存在：' + stage02)
    if os.path.exists(output) and os.path.exists(os.path.join(output, 'stage03a_complete.json')):
        base.fail('阶段 3A 完整输出已存在，拒绝覆盖：' + output)
    if os.path.exists(output):
        # 仅允许恢复本脚本异常中断时、未写完成标记的目录；完整结果始终拒绝覆盖。
        print('[scene-stage03a] 恢复未完成输出目录：' + output)
    source = os.path.join(stage02, 'evidence', 'continuous_hard_height_edge_support.pcd')
    raw = base.read_binary_pcd(source)
    points = [(item[0], item[1], item[2]) for item in raw]
    point_index = local_edges.spatial_index(points, 0.40)
    # 1.5 m 邻域使稀疏但方向稳定的长绿化/路沿边也能参与全局投票。
    tangents = local_edges.local_line_support(points, point_index, 1.50)
    active = set(tangents)
    primitives = []
    rejected_peaks = 0
    while active and len(primitives) < 400:
        peak = hough_peak(points, tangents, active)
        if peak is None:
            break
        normal_x, normal_y, rho, _count = peak
        initial = candidate_inliers(points, tangents, active, normal_x, normal_y, rho)
        fitted = robust_fit(points, tangents, active, initial)
        if fitted is None:
            # 删除该峰的一小部分支持，避免短噪声峰无限重复。
            if initial:
                active.difference_update(initial[:max(1, min(8, len(initial)))])
            else:
                active.discard(next(iter(active)))
            rejected_peaks += 1
            continue
        line, inliers = fitted
        tangent_x, tangent_y, normal_x, normal_y, rho, mean_x, mean_y = line
        accepted = 0
        for group in split_by_gap(points, inliers, tangent_x, tangent_y):
            if len(group) < MIN_INLIERS:
                continue
            start = projection(points[group[0]], tangent_x, tangent_y)
            end = projection(points[group[-1]], tangent_x, tangent_y)
            length = end - start
            if length < MIN_LENGTH_M:
                continue
            residual = sum(line_distance(points[item], normal_x, normal_y, rho) for item in group) / len(group)
            primitives.append({
                'id': len(primitives), 'members': group, 'tx': tangent_x, 'ty': tangent_y,
                'nx': normal_x, 'ny': normal_y, 'rho': rho, 'start': start, 'end': end,
                'length': length, 'residual': residual, 'mean_x': mean_x, 'mean_y': mean_y,
            })
            accepted += 1
        # 无论本次是否产出完整基元，都移除该拟合线附近的点，转而寻找下一条全局线。
        active.difference_update(inliers)
        if not accepted:
            rejected_peaks += 1
    base.create_directory(os.path.join(output, 'evidence'))
    base.create_directory(os.path.join(output, 'validation'))
    observed = []
    fitted_points = []
    inferred = []
    with open(os.path.join(output, 'evidence', 'global_boundary_primitives.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('primitive_id', 'type', 'tangent_x', 'tangent_y', 'normal_x', 'normal_y',
                         'rho_m', 'start_projection_m', 'end_projection_m', 'length_m',
                         'observed_inlier_count', 'mean_residual_m', 'short_gap_count'))
        for primitive in primitives:
            color = primitive_color(primitive['id'])
            member_projections = sorted(projection(points[item], primitive['tx'], primitive['ty'])
                                        for item in primitive['members'])
            short_gaps = []
            for first, second in zip(member_projections, member_projections[1:]):
                if 0.25 < second - first <= GAP_SPLIT_M:
                    short_gaps.append((first, second))
            writer.writerow((primitive['id'], 'global_line_segment',
                             '%.9f' % primitive['tx'], '%.9f' % primitive['ty'],
                             '%.9f' % primitive['nx'], '%.9f' % primitive['ny'],
                             '%.6f' % primitive['rho'], '%.6f' % primitive['start'],
                             '%.6f' % primitive['end'], '%.6f' % primitive['length'],
                             len(primitive['members']), '%.6f' % primitive['residual'], len(short_gaps)))
            for item in primitive['members']:
                point = points[item]
                observed.append((point[0], point[1], point[2], color))
            # 直线起点由 mean + normal*rho 构造，避免依赖世界坐标原点的投影偏置。
            anchor_x, anchor_y = primitive['nx'] * primitive['rho'], primitive['ny'] * primitive['rho']
            for step in range(int(math.floor(primitive['length'] / 0.10)) + 1):
                coordinate = primitive['start'] + step * 0.10
                fitted_points.append((anchor_x + primitive['tx'] * coordinate,
                                      anchor_y + primitive['ty'] * coordinate,
                                      sum(points[item][2] for item in primitive['members']) / len(primitive['members']),
                                      color))
            for first, second in short_gaps:
                steps = int(math.floor((second - first) / 0.10))
                for step in range(1, steps):
                    coordinate = first + step * 0.10
                    inferred.append((anchor_x + primitive['tx'] * coordinate,
                                     anchor_y + primitive['ty'] * coordinate,
                                     sum(points[item][2] for item in primitive['members']) / len(primitive['members']),
                                     base.packed_rgb(250, 230, 70)))
    write_line_pcd(os.path.join(output, 'evidence', 'global_boundary_primitive_observed_support.pcd'), observed)
    write_line_pcd(os.path.join(output, 'evidence', 'global_boundary_primitive_fitted_support.pcd'), fitted_points)
    write_line_pcd(os.path.join(output, 'evidence', 'global_boundary_primitive_short_gap_support.pcd'), inferred)
    # 先合并同一物理边界上的直线碎段；再用阶段 1 校正轨迹的左右横断面配对。
    bands = merge_boundary_bands(points, primitives)
    for band in bands:
        band['member_points'] = [points[item] for item in band['members']]
    trajectory, trajectory_source = read_corrected_trajectory(stage02)
    trajectory_samples, pairs = pair_boundary_bands(trajectory, bands)
    hierarchy_samples, hierarchy_roles = classify_boundary_hierarchy(trajectory, bands)
    paired_band_ids = set()
    for pair in pairs:
        paired_band_ids.add(pair['left_band_id'])
        paired_band_ids.add(pair['right_band_id'])
    merged_points = []
    final_points = []
    for band in bands:
        append_band_fitted_points(band, primitive_color(band['id']), merged_points)
    # 同一对边界用相同颜色，方便在 CloudCompare 中直接辨认“是否真的是一对道路边界”。
    pair_colors = ((45, 190, 225), (240, 110, 220), (110, 210, 95),
                   (250, 170, 55), (190, 130, 250), (245, 220, 70))
    # 一条长边界带可能在路口附近短暂同时匹配多条对侧边。最终几何只写一次，
    # 颜色归属给共同横断面支持最多的一对，避免点重复导致视觉上“更粗、更可信”的错觉。
    best_pair_for_band = {}
    for pair in pairs:
        for band_id in (pair['left_band_id'], pair['right_band_id']):
            previous = best_pair_for_band.get(band_id)
            if previous is None or pair['support_samples'] > previous['support_samples']:
                best_pair_for_band[band_id] = pair
    for band_id in sorted(best_pair_for_band):
        pair = best_pair_for_band[band_id]
        color = base.packed_rgb(*pair_colors[pair['id'] % len(pair_colors)])
        append_band_fitted_points(bands[band_id], color, final_points)
    hierarchy_points = []
    hierarchy_colors = {
        'primary_road_side_candidate': base.packed_rgb(255, 164, 55),
        'secondary_sidewalk_outer_candidate': base.packed_rgb(55, 205, 225),
        'unassociated_boundary_geometry': base.packed_rgb(185, 185, 185),
    }
    for band in bands:
        append_band_fitted_points(band, hierarchy_colors[hierarchy_roles[band['id']]['role']],
                                  hierarchy_points)
    write_line_pcd(os.path.join(output, 'evidence', 'merged_boundary_band_fitted_support.pcd'), merged_points)
    write_line_pcd(os.path.join(output, 'evidence', 'trajectory_paired_boundary_band_fitted_support.pcd'), final_points)
    write_line_pcd(os.path.join(output, 'evidence', 'trajectory_boundary_hierarchy_fitted_support.pcd'),
                   hierarchy_points)
    with open(os.path.join(output, 'evidence', 'merged_boundary_bands.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('band_id', 'source_primitive_ids', 'tangent_x', 'tangent_y',
                         'normal_x', 'normal_y', 'rho_m', 'start_projection_m',
                         'end_projection_m', 'length_m', 'observed_inlier_count',
                         'mean_residual_m', 'paired_by_trajectory'))
        for band in bands:
            writer.writerow((band['id'], '|'.join(str(item) for item in band['fragment_ids']),
                             '%.9f' % band['tx'], '%.9f' % band['ty'],
                             '%.9f' % band['nx'], '%.9f' % band['ny'], '%.6f' % band['rho'],
                             '%.6f' % band['start'], '%.6f' % band['end'],
                             '%.6f' % band['length'], len(band['members']),
                             '%.6f' % band['residual'], int(band['id'] in paired_band_ids)))
    with open(os.path.join(output, 'evidence', 'trajectory_boundary_band_pairs.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('pair_id', 'left_band_id', 'right_band_id', 'support_sample_count',
                         'longest_continuous_sample_count', 'mean_width_m'))
        for pair in pairs:
            writer.writerow((pair['id'], pair['left_band_id'], pair['right_band_id'],
                             pair['support_samples'], pair['longest_continuous_samples'],
                             '%.6f' % pair['mean_width']))
    with open(os.path.join(output, 'evidence', 'trajectory_boundary_hierarchy.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('band_id', 'trajectory_role', 'primary_support_sample_count',
                         'primary_longest_continuous_sample_count', 'secondary_support_sample_count',
                         'secondary_longest_continuous_sample_count'))
        for band in bands:
            role = hierarchy_roles[band['id']]
            writer.writerow((band['id'], role['role'], role['primary_support_samples'],
                             role['primary_longest_run'], role['secondary_support_samples'],
                             role['secondary_longest_run']))
    report = {
        'schema': 'fast_livo_scene_pipeline_stage03a_global_boundary_primitives/v1',
        'purpose': '地图尺度直线边界基元；不是路沿、围墙或道路语义。',
        'source_stage02': stage02,
        'raw_hard_height_points': len(points),
        'local_tangent_support_points': len(tangents),
        'global_line_primitives': len(primitives),
        'primitive_observed_points': len(observed),
        'primitive_fitted_points': len(fitted_points),
        'short_gap_inferred_points': len(inferred),
        'merged_boundary_bands': len(bands),
        'merged_boundary_band_fitted_points': len(merged_points),
        'trajectory_source': trajectory_source,
        'trajectory_cross_section_samples': len(trajectory_samples),
        'trajectory_hierarchy_samples': len(hierarchy_samples),
        'trajectory_double_side_pairs': len(pairs),
        'trajectory_paired_boundary_bands': len(paired_band_ids),
        'trajectory_paired_boundary_fitted_points': len(final_points),
        'trajectory_hierarchy_fitted_points': len(hierarchy_points),
        'trajectory_hierarchy_primary_bands': sum(
            1 for item in hierarchy_roles.values()
            if item['role'] == 'primary_road_side_candidate'),
        'trajectory_hierarchy_secondary_bands': sum(
            1 for item in hierarchy_roles.values()
            if item['role'] == 'secondary_sidewalk_outer_candidate'),
        'rejected_short_or_noisy_peaks': rejected_peaks,
        'parameters': {'line_residual_m': LINE_RESIDUAL_M, 'gap_split_m': GAP_SPLIT_M,
                       'minimum_length_m': MIN_LENGTH_M, 'minimum_inliers': MIN_INLIERS,
                       'band_direction_cos': BAND_DIRECTION_COS,
                       'band_normal_offset_m': BAND_NORMAL_OFFSET_M,
                       'band_endpoint_gap_m': BAND_ENDPOINT_GAP_M,
                       'pair_max_lateral_m': PAIR_MAX_LATERAL_M,
                       'pair_min_width_m': PAIR_MIN_WIDTH_M,
                       'pair_max_width_m': PAIR_MAX_WIDTH_M,
                       'pair_min_support_samples': PAIR_MIN_SUPPORT_SAMPLES},
        'limits': [
            '当前版本优先提取直线基元；明显曲线路沿需在后续折线/曲线基元阶段处理。',
            '彩色拟合线在同一基元和同一边界带内可跨受限缺口；长空缺必须保持断开。',
            '双边配对 PCD 只用于道路核心候选；最终层级 PCD 保留所有合并几何带，不能用未配对删除边界。',
            '层级 PCD 中的主/第二边界是轨迹关系候选；它们不是路沿、人行道或围墙的最终语义。'
        ]
    }
    with open(os.path.join(output, 'validation', 'stage03a_global_boundary_primitives_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'stage03a_complete.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '3a_global_boundary_primitives',
                   'source_stage02': stage02,
                   'policy': '只读阶段 2 R2 原始硬高差，不改写既有阶段 3 输出。'},
                  handle, ensure_ascii=False, indent=2)
        handle.write('\n')
    print('[scene-stage03a] 完成：原始硬边点={0}，局部切向点={1}，全局直线基元={2}，合并边界带={3}，轨迹双边配对={4}，输出={5}'.format(
        len(points), len(tangents), len(primitives), len(bands), len(pairs), output))


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as error:
        print('[scene-stage03a] ' + str(error), file=sys.stderr)
        sys.exit(1)
