#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 4：路沿、人行道/路肩与开放区域的可审计提取。

本阶段严格只读阶段 2、阶段 3A 和阶段 3 最终交接包。它不会重跑地面、硬高差或
道路走廊，也不会将固定宽度道路候选升级为观测道路事实。

核心思想是沿阶段 3 的横断面，把阶段 3A 的长边界带重新关联到道路两侧；随后同时
检查边界带的硬高差原始支持、道路内侧/外侧稳定地面、完整道路搜索面、连续长度和
方向关系。分类结果是证据判断而不是渲染标签：证据不足时必须输出 unknown。
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

from common import geometry as base


# 与阶段 3 的横断面方向容忍度保持一致。阶段 4 只在该范围内讨论“沿道路”的边界。
MAX_DIRECTION_DEVIATION_DEG = 35.0
DIRECTION_COS = math.cos(math.radians(MAX_DIRECTION_DEVIATION_DEG))
MAX_BOUNDARY_LATERAL_M = 10.0
BOUNDARY_ENDPOINT_TOLERANCE_M = 0.50
MIN_BOUNDARY_SAMPLES = 5
MIN_BOUNDARY_CONTINUITY_M = 2.0
CURB_MIN_CONTINUITY_M = 3.0
CURB_INNER_DISTANCE_TOLERANCE_M = 0.45
CURB_MIN_ROAD_SUPPORT = 0.60
CURB_MIN_EDGE_SUPPORT = 0.50
CURB_MIN_GROUND_SUPPORT = 0.45
CURB_MIN_HEIGHT_M = 0.035
CURB_MAX_HEIGHT_M = 0.30
STEP_MIN_HEIGHT_M = 0.25
# 低道路证据复核层不是新语义。它只暴露“几何上连续、但道路内侧候选面不足”的原始硬高差，
# 供人工判断是否需要在后续证据阶段复查道路关系；不得被渲染层当作路沿或道路边界。
LOW_ROAD_REVIEW_MIN_CONTINUITY_M = 3.0
LOCAL_EDGE_SLICE_HALF_LENGTH_M = 0.45
LOCAL_EDGE_CLUSTER_GAP_M = 0.35
LOCAL_EDGE_TRACK_LATERAL_GAP_M = 0.55
LOCAL_EDGE_MAX_SAMPLE_GAP = 3
LOCAL_EDGE_MIN_SAMPLES = 5
SIDEWALK_MIN_WIDTH_M = 0.45
SIDEWALK_MAX_WIDTH_M = 6.0
# 0.45--0.9m 的双边带可能只是同一条粗硬高差的双线、窄路肩或栅格量化产物；
# 它们保留为 possible，不能在没有额外语义证据时画成确认人行道面。
CONFIRMED_SIDEWALK_MIN_WIDTH_M = 0.90
SIDEWALK_MIN_CONTINUITY_M = 3.0
SIDEWALK_MIN_GROUND_SUPPORT = 0.60


def fail(message):
    raise RuntimeError(message)


def median(values):
    values = sorted(value for value in values if value is not None)
    if not values:
        return None
    middle = len(values) // 2
    if len(values) % 2:
        return values[middle]
    return 0.5 * (values[middle - 1] + values[middle])


def mean(values):
    values = [value for value in values if value is not None]
    return None if not values else sum(values) / float(len(values))


def fraction(values):
    return 0.0 if not values else float(sum(1 for value in values if value)) / float(len(values))


def csv_number(row, name):
    return None if not row.get(name, '') else float(row[name])


def require_file(path, description):
    if not os.path.isfile(path):
        fail('缺少{0}：{1}'.format(description, path))


def read_grid(path):
    """读取所有稳定地面格；保留自由空间和跨帧支持，不能只从 PCD 颜色反推。"""
    need = ('cell_ix', 'cell_iy', 'ground_mean_z', 'stable_ground', 'ground_frame_count',
            'free_frame_count', 'observation_confidence')
    ground = {}
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段 2 几何网格字段不兼容：' + path)
        for row in reader:
            # 与阶段 3 一致：没有至少一次自由空间支持的稳定地面，不扩展成可走区域。
            if row['stable_ground'] != '1' or int(row['free_frame_count']) < 1:
                continue
            ground[(int(row['cell_ix']), int(row['cell_iy']))] = {
                'z': float(row['ground_mean_z']),
                'ground_frames': int(row['ground_frame_count']),
                'free_frames': int(row['free_frame_count']),
                'confidence': float(row['observation_confidence']),
            }
    if not ground:
        fail('阶段 2 中没有带自由空间支持的稳定地面')
    return ground


def read_bands(path):
    need = ('band_id', 'source_primitive_ids', 'tangent_x', 'tangent_y', 'normal_x', 'normal_y',
            'rho_m', 'start_projection_m', 'end_projection_m', 'length_m',
            'observed_inlier_count', 'mean_residual_m')
    bands = []
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段 3A 边界带字段不兼容：' + path)
        for row in reader:
            bands.append({
                'id': int(row['band_id']), 'source_primitive_ids': row['source_primitive_ids'],
                'tx': float(row['tangent_x']), 'ty': float(row['tangent_y']),
                'nx': float(row['normal_x']), 'ny': float(row['normal_y']), 'rho': float(row['rho_m']),
                'start': float(row['start_projection_m']), 'end': float(row['end_projection_m']),
                'length': float(row['length_m']), 'observed_inliers': int(row['observed_inlier_count']),
                'mean_residual': float(row['mean_residual_m']),
                'source': 'stage03a_r4_merged_band',
            })
    if not bands:
        fail('阶段 3A 的合并边界带为空')
    return bands


def read_hierarchy(path):
    result = {}
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('band_id', 'trajectory_role', 'primary_support_sample_count',
                'primary_longest_continuous_sample_count', 'secondary_support_sample_count',
                'secondary_longest_continuous_sample_count')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段 3A 边界层级字段不兼容：' + path)
        for row in reader:
            result[int(row['band_id'])] = {
                'role': row['trajectory_role'],
                'primary_support': int(row['primary_support_sample_count']),
                'primary_run': int(row['primary_longest_continuous_sample_count']),
                'secondary_support': int(row['secondary_support_sample_count']),
                'secondary_run': int(row['secondary_longest_continuous_sample_count']),
            }
    return result


def read_cross_sections(path):
    result = []
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('sample_id', 'x', 'y', 'tangent_x', 'tangent_y', 'confidence', 'candidate_class',
                'use_for_stage4_road_surface_candidate', 'use_for_stage4_centerline',
                'use_for_stage4_complete_candidate_centerline', 'left_boundary_distance_m',
                'right_boundary_distance_m')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段 3 横断面字段不兼容：' + path)
        for row in reader:
            tx, ty = float(row['tangent_x']), float(row['tangent_y'])
            result.append({
                'id': int(row['sample_id']), 'x': float(row['x']), 'y': float(row['y']),
                'tx': tx, 'ty': ty, 'nx': -ty, 'ny': tx, 'confidence': row['confidence'],
                'candidate_class': row['candidate_class'],
                'use_road': row['use_for_stage4_road_surface_candidate'] == '1',
                'use_centerline': row['use_for_stage4_centerline'] == '1',
                'use_complete': row['use_for_stage4_complete_candidate_centerline'] == '1',
                'left_distance': csv_number(row, 'left_boundary_distance_m'),
                'right_distance': csv_number(row, 'right_boundary_distance_m'),
            })
    if len(result) < 3:
        fail('阶段 3 横断面数量不足')
    return result


def read_road_cells(path):
    result = {}
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('cell_ix', 'cell_iy', 'road_candidate_primary_source', 'observed',
                'road_membership_confidence', 'all_candidate_sources')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段 3 完整道路候选字段不兼容：' + path)
        for row in reader:
            result[(int(row['cell_ix']), int(row['cell_iy']))] = {
                'source': row['road_candidate_primary_source'], 'observed': row['observed'] == '1',
                'confidence': row['road_membership_confidence'], 'all_sources': row['all_candidate_sources'],
            }
    if not result:
        fail('阶段 3 完整道路候选为空')
    return result


class PointIndex(object):
    """非常小的二维散列索引，避免为每条边界线扫描全部硬高差点。"""

    def __init__(self, points, cell_m=0.8):
        self.points = points
        self.cell_m = cell_m
        self.cells = defaultdict(list)
        for point in points:
            self.cells[self.key(point[0], point[1])].append(point)

    def key(self, x, y):
        return (int(math.floor(x / self.cell_m)), int(math.floor(y / self.cell_m)))

    def nearest_z(self, x, y, radius_m):
        key = self.key(x, y)
        radius = int(math.ceil(radius_m / self.cell_m))
        best = None
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for point in self.cells.get((key[0] + dx, key[1] + dy), ()):
                    distance = math.hypot(point[0] - x, point[1] - y)
                    if distance <= radius_m and (best is None or distance < best[0]):
                        best = (distance, point[2])
        return None if best is None else best[1]

    def nearby(self, x, y, radius_m):
        """返回半径内原始点；调用者仍须按横断面方向进行精确筛选。"""
        key = self.key(x, y)
        radius = int(math.ceil(radius_m / self.cell_m))
        result = []
        radius_sq = radius_m * radius_m
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                for point in self.cells.get((key[0] + dx, key[1] + dy), ()):
                    if (point[0] - x) * (point[0] - x) + (point[1] - y) * (point[1] - y) <= radius_sq:
                        result.append(point)
        return result


def nearest_ground(ground, x, y, radius_cells=2):
    key = base.grid_key(x, y)
    best = None
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            cell = (key[0] + dx, key[1] + dy)
            item = ground.get(cell)
            if item is None:
                continue
            cx, cy = base.cell_center(cell)
            distance = math.hypot(cx - x, cy - y)
            if best is None or distance < best[0]:
                best = (distance, item, cell)
    return None if best is None else best


def road_support_state(road_cells, x, y, radius_cells=2):
    """区分“可搜索道路候选”和“主来源为双边实测”的道路支持。

    完整候选面允许阶段 4 不中断搜索，但固定宽度/桥接格不能单独让一条边界变成
    observed 路沿。因此调用者必须同时查看两个返回值。
    """
    key = base.grid_key(x, y)
    candidate, observed = False, False
    for dx in range(-radius_cells, radius_cells + 1):
        for dy in range(-radius_cells, radius_cells + 1):
            item = road_cells.get((key[0] + dx, key[1] + dy))
            if item is not None:
                candidate = True
                observed = observed or item['observed']
    return candidate, observed


def observed_road_edge_distance(road_cells, sample, side):
    """估计横断面上双边实测道路核心向外的最后支持距离。

    仅供阶段2原始硬高差的局部补拟合使用：当 R4 漏拟合时，横断面 CSV 往往没有
    可比的 R4 内边界距离，不能因此永久拒绝已观测硬高差。
    """
    last, misses = None, 0
    for index in range(1, int(MAX_BOUNDARY_LATERAL_M / base.GRID_M) + 1):
        distance = index * base.GRID_M
        x = sample['x'] + side * sample['nx'] * distance
        y = sample['y'] + side * sample['ny'] * distance
        _, observed = road_support_state(road_cells, x, y, 1)
        if observed:
            last, misses = distance, 0
        elif last is not None:
            misses += 1
            if misses >= 3:
                break
    return last


def band_point(band, projection):
    return (band['tx'] * projection + band['nx'] * band['rho'],
            band['ty'] * projection + band['ny'] * band['rho'])


def path_length_for_ids(samples_by_id, ids):
    """最长连续轨迹段长度；跨越大采样跳跃时保持断开。"""
    ordered = sorted(set(ids))
    if not ordered:
        return 0.0
    longest, current = 0.0, 0.0
    previous = ordered[0]
    for sample_id in ordered[1:]:
        if sample_id > previous + 3:
            longest = max(longest, current)
            current = 0.0
        else:
            first, second = samples_by_id[previous], samples_by_id[sample_id]
            current += math.hypot(second['x'] - first['x'], second['y'] - first['y'])
        previous = sample_id
    return max(longest, current)


def classify_band_association(band, side, links, hierarchy, samples_by_id, ground, road_cells, edge_index):
    """从一条边界带在道路一侧的所有关联，生成一条可审计语义记录。"""
    checked = []
    for link in links:
        sample = samples_by_id[link['sample_id']]
        px, py = link['point']
        # 朝道路中心的一侧是内侧，远离道路的一侧是外侧；仅比较稳定地面，不生成新地面。
        inside = nearest_ground(ground, px - side * sample['nx'] * 0.25,
                                py - side * sample['ny'] * 0.25)
        outside = nearest_ground(ground, px + side * sample['nx'] * 0.35,
                                 py + side * sample['ny'] * 0.35)
        edge_z = edge_index.nearest_z(px, py, 0.45)
        expected = sample['left_distance'] if side > 0 else sample['right_distance']
        observed_road_edge = observed_road_edge_distance(road_cells, sample, side)
        road_candidate, road_observed = road_support_state(road_cells,
                                                            px - side * sample['nx'] * 0.25,
                                                            py - side * sample['ny'] * 0.25)
        checked.append({
            'sample_id': link['sample_id'], 'lateral': link['lateral'], 'projection': link['projection'],
            'point': link['point'], 'edge_z': edge_z, 'inside': inside, 'outside': outside,
            'inner_match': expected is not None and abs(abs(link['lateral']) - expected) <= CURB_INNER_DISTANCE_TOLERANCE_M,
            'local_road_edge_match': observed_road_edge is not None and
                                     abs(abs(link['lateral']) - observed_road_edge) <= 0.60,
            'road_support': road_candidate, 'observed_road_support': road_observed,
        })

    edge_support = fraction([item['edge_z'] is not None for item in checked])
    road_support = fraction([item['road_support'] for item in checked])
    observed_road_support = fraction([item['observed_road_support'] for item in checked])
    topology_support = fraction([samples_by_id[item['sample_id']]['use_centerline'] for item in checked])
    inside_support = fraction([item['inside'] is not None for item in checked])
    outside_support = fraction([item['outside'] is not None for item in checked])
    inner_match = fraction([item['inner_match'] for item in checked])
    local_road_edge_match = fraction([item['local_road_edge_match'] for item in checked])
    height_deltas = []
    for item in checked:
        if item['inside'] is not None and item['outside'] is not None:
            height_deltas.append(item['outside'][1]['z'] - item['inside'][1]['z'])
        elif item['edge_z'] is not None and item['inside'] is not None:
            height_deltas.append(item['edge_z'] - item['inside'][1]['z'])
    local_height = median(height_deltas)
    continuity = path_length_for_ids(samples_by_id, [item['sample_id'] for item in checked])
    direction_angle = math.degrees(math.acos(min(1.0, max(-1.0,
        abs(band['tx'] * samples_by_id[checked[0]['sample_id']]['tx'] +
            band['ty'] * samples_by_id[checked[0]['sample_id']]['ty'])))))
    role = hierarchy.get(band['id'], {}).get('role', 'unknown_boundary_hierarchy')

    reasons = []
    if len(checked) < MIN_BOUNDARY_SAMPLES:
        reasons.append('横断面关联数量不足')
    if continuity < MIN_BOUNDARY_CONTINUITY_M:
        reasons.append('沿轨迹连续长度不足')
    if edge_support < CURB_MIN_EDGE_SUPPORT:
        reasons.append('连续硬高差原始支持不足')
    if road_support < CURB_MIN_ROAD_SUPPORT:
        reasons.append('道路内侧完整搜索面支持不足')
    if observed_road_support < CURB_MIN_ROAD_SUPPORT:
        reasons.append('道路内侧双边实测道路核心支持不足')
    if topology_support < CURB_MIN_ROAD_SUPPORT:
        reasons.append('道路内侧高/中证据中心线支持不足')
    if inside_support < CURB_MIN_GROUND_SUPPORT:
        reasons.append('道路内侧稳定地面支持不足')

    semantic = 'unknown_boundary'
    state = 'unknown'
    confidence = 0.0
    height_abs = None if local_height is None else abs(local_height)
    basic_ok = (len(checked) >= MIN_BOUNDARY_SAMPLES and continuity >= MIN_BOUNDARY_CONTINUITY_M and
                edge_support >= CURB_MIN_EDGE_SUPPORT and road_support >= CURB_MIN_ROAD_SUPPORT and
                inside_support >= CURB_MIN_GROUND_SUPPORT)
    boundary_match = max(inner_match, local_road_edge_match) if band.get('source') == 'stage02_hard_edge_local_fit' else inner_match
    curb_geometry_ok = (basic_ok and topology_support >= CURB_MIN_ROAD_SUPPORT and
                        continuity >= CURB_MIN_CONTINUITY_M and boundary_match >= 0.50 and
                        height_abs is not None and CURB_MIN_HEIGHT_M <= height_abs <= CURB_MAX_HEIGHT_M)
    if curb_geometry_ok:
        semantic = 'curb_candidate'
        state = 'observed' if observed_road_support >= CURB_MIN_ROAD_SUPPORT else 'inferred'
        confidence = min(1.0 if state == 'observed' else 0.74,
                         0.20 * min(1.0, continuity / 8.0) + 0.20 * edge_support +
                         0.15 * road_support + 0.10 * observed_road_support + 0.15 * topology_support +
                         0.10 * inside_support + 0.10 * inner_match)
        if state == 'inferred':
            reasons.append('道路内侧非双边实测核心：沿高/中证据中心线与连续硬高差的推断路沿延续')
    elif basic_ok and height_abs is not None and height_abs >= STEP_MIN_HEIGHT_M and outside_support >= 0.35:
        semantic = 'step_candidate'
        state = 'observed'
        confidence = min(1.0, 0.25 * min(1.0, continuity / 8.0) + 0.25 * edge_support +
                         0.25 * road_support + 0.25 * min(1.0, height_abs / 0.5))
    elif basic_ok and outside_support < 0.35:
        # 仅靠当前冻结输入无法可靠区分墙根和植被，故保守地输出组合候选。
        semantic = 'wall_or_green_boundary_candidate'
        state = 'observed' if edge_support >= 0.65 else 'inferred'
        confidence = min(0.85, 0.30 * min(1.0, continuity / 8.0) + 0.30 * edge_support +
                         0.25 * road_support + 0.15 * (1.0 - outside_support))
    else:
        if basic_ok:
            reasons.append('高度或内外地面关系不足以区分路沿、台阶或外侧对象')
            state = 'inferred'
            confidence = min(0.49, 0.2 * edge_support + 0.2 * road_support + 0.1 * inside_support)

    min_projection = min(item['projection'] for item in checked)
    max_projection = max(item['projection'] for item in checked)
    first_x, first_y = band_point(band, min_projection)
    last_x, last_y = band_point(band, max_projection)
    return {
        'band_id': band['id'], 'boundary_source': band.get('source', 'stage03a_r4_merged_band'),
        'source_primitive_ids': band['source_primitive_ids'], 'side': 'left' if side > 0 else 'right',
        'side_sign': side, 'band': band, 'trajectory_role': role, 'links': checked,
        'start_x': first_x, 'start_y': first_y, 'end_x': last_x, 'end_y': last_y,
        'start_projection': min_projection, 'end_projection': max_projection,
        'associated_length_m': math.hypot(last_x - first_x, last_y - first_y),
        'continuous_length_m': continuity,
        'relative_road_distance_m': mean([abs(item['lateral']) for item in checked]),
        'local_height_delta_m': local_height, 'direction_angle_deg': direction_angle,
        'hard_edge_support_fraction': edge_support, 'road_support_fraction': road_support,
        'observed_road_support_fraction': observed_road_support,
        'topology_support_fraction': topology_support,
        'inside_ground_support_fraction': inside_support, 'outside_ground_support_fraction': outside_support,
        'inner_boundary_match_fraction': inner_match, 'local_road_edge_match_fraction': local_road_edge_match,
        'semantic_class': semantic,
        'classification_confidence': confidence, 'evidence_state': state,
        'rejection_reason': '|'.join(reasons),
    }


def associate_boundaries(bands, samples, hierarchy, ground, road_cells, edge_index):
    samples_by_id = dict((sample['id'], sample) for sample in samples)
    grouped = defaultdict(list)
    for band in bands:
        for sample in samples:
            if not sample['use_complete']:
                continue
            if abs(sample['tx'] * band['tx'] + sample['ty'] * band['ty']) < DIRECTION_COS:
                continue
            projection = sample['x'] * band['tx'] + sample['y'] * band['ty']
            if projection < band['start'] - BOUNDARY_ENDPOINT_TOLERANCE_M or projection > band['end'] + BOUNDARY_ENDPOINT_TOLERANCE_M:
                continue
            px, py = band_point(band, projection)
            lateral = (px - sample['x']) * sample['nx'] + (py - sample['y']) * sample['ny']
            if abs(lateral) > MAX_BOUNDARY_LATERAL_M or abs(lateral) < 0.05:
                continue
            side = 1 if lateral > 0.0 else -1
            grouped[(band['id'], side)].append({
                'sample_id': sample['id'], 'lateral': lateral, 'projection': projection, 'point': (px, py),
            })
    records = []
    for band in bands:
        for side in (-1, 1):
            links = grouped.get((band['id'], side), ())
            if links:
                records.append(classify_band_association(band, side, links, hierarchy, samples_by_id,
                                                         ground, road_cells, edge_index))
    return records, samples_by_id


def local_slice_clusters(sample, edge_index):
    """在单个横断面内，把原始硬高差点按左右侧和横向距离聚为多个局部候选。

    这里不从 PCD 颜色读取语义。颜色字段只是阶段 2 可视化编码，局部候选完全依据
    原始 XYZ 点相对于横断面的纵向/横向位置构成。
    """
    by_side = {1: [], -1: []}
    for point in edge_index.nearby(sample['x'], sample['y'], MAX_BOUNDARY_LATERAL_M + 0.5):
        dx, dy = point[0] - sample['x'], point[1] - sample['y']
        longitudinal = dx * sample['tx'] + dy * sample['ty']
        lateral = dx * sample['nx'] + dy * sample['ny']
        if abs(longitudinal) > LOCAL_EDGE_SLICE_HALF_LENGTH_M or abs(lateral) < 0.10:
            continue
        if abs(lateral) > MAX_BOUNDARY_LATERAL_M:
            continue
        by_side[1 if lateral > 0.0 else -1].append((lateral, point))
    result = []
    for side, entries in by_side.items():
        entries.sort(key=lambda item: abs(item[0]))
        groups, current, previous = [], [], None
        for lateral, point in entries:
            if current and abs(abs(lateral) - abs(previous)) > LOCAL_EDGE_CLUSTER_GAP_M:
                groups.append(current)
                current = []
            current.append((lateral, point))
            previous = lateral
        if current:
            groups.append(current)
        for group in groups:
            # 一个横断面内的点数少并不自动无效；跨横断面连续性会在追踪步骤中检验。
            lateral = median([item[0] for item in group])
            x = median([item[1][0] for item in group])
            y = median([item[1][1] for item in group])
            z = median([item[1][2] for item in group])
            result.append({'sample_id': sample['id'], 'side': side, 'lateral': lateral,
                           'point': (x, y), 'z': z, 'raw_count': len(group)})
    return result


def local_edge_tracks(samples, edge_index):
    """跨完整中心线搜索范围追踪同侧、横向位置平滑的原始硬高差候选。

    高/中证据仍决定能否把结果升级为 observed/inferred 路沿，但不能再作为“是否提取
    原始局部边界”的前置门槛；否则阶段3低证据缺口会让阶段4无法看到恰好位于缺口的
    清晰硬高差。
    """
    tracks = []
    active = []
    for sample in samples:
        if not sample['use_complete']:
            continue
        candidates = local_slice_clusters(sample, edge_index)
        used = set()
        # 近到远处理，防止一条内侧边界被更远的墙根候选抢占。
        for candidate in sorted(candidates, key=lambda item: (item['side'], abs(item['lateral']))):
            best = None
            for index, track in enumerate(active):
                last = track[-1]
                if index in used or last['side'] != candidate['side']:
                    continue
                if candidate['sample_id'] > last['sample_id'] + LOCAL_EDGE_MAX_SAMPLE_GAP:
                    continue
                difference = abs(candidate['lateral'] - last['lateral'])
                if difference <= LOCAL_EDGE_TRACK_LATERAL_GAP_M and (best is None or difference < best[0]):
                    best = (difference, index)
            if best is None:
                track = [candidate]
                active.append(track)
                tracks.append(track)
            else:
                active[best[1]].append(candidate)
                used.add(best[1])
        # 已长时间未更新的轨迹不再参与后续匹配；保留在 tracks 中等待筛选。
        active = [track for track in active if sample['id'] <= track[-1]['sample_id'] + LOCAL_EDGE_MAX_SAMPLE_GAP]
    return tracks


def split_track(track):
    """从轨迹中移除 R4 已覆盖点后，按采样缺口切分为独立局部段。"""
    result, current, previous = [], [], None
    for item in track:
        if previous is not None and item['sample_id'] > previous + LOCAL_EDGE_MAX_SAMPLE_GAP:
            if current:
                result.append(current)
            current = []
        current.append(item)
        previous = item['sample_id']
    if current:
        result.append(current)
    return result


def local_band_from_track(track, local_id):
    if len(track) < LOCAL_EDGE_MIN_SAMPLES:
        return None
    first, last = track[0], track[-1]
    dx, dy = last['point'][0] - first['point'][0], last['point'][1] - first['point'][1]
    norm = math.hypot(dx, dy)
    if norm < MIN_BOUNDARY_CONTINUITY_M:
        return None
    tx, ty = dx / norm, dy / norm
    nx, ny = -ty, tx
    rho = nx * first['point'][0] + ny * first['point'][1]
    projections = [item['point'][0] * tx + item['point'][1] * ty for item in track]
    band = {
        'id': -100000 - local_id, 'source_primitive_ids': 'stage02_raw_hard_edge',
        'tx': tx, 'ty': ty, 'nx': nx, 'ny': ny, 'rho': rho,
        'start': min(projections), 'end': max(projections), 'length': norm,
        'observed_inliers': sum(item['raw_count'] for item in track), 'mean_residual': 0.0,
        'source': 'stage02_hard_edge_local_fit',
    }
    links = []
    for item, projection in zip(track, projections):
        links.append({'sample_id': item['sample_id'], 'lateral': item['lateral'],
                      'projection': projection, 'point': item['point']})
    return band, (1 if median([item['lateral'] for item in track]) > 0.0 else -1), links


def extract_local_hard_edge_records(samples, samples_by_id, r4_records, hierarchy, ground, road_cells, edge_index):
    """从阶段 2 原始硬高差补提取 R4 未覆盖的局部边界，并复用同一语义分类规则。"""
    covered = defaultdict(list)
    for record in r4_records:
        # 只让已经确认的 R4 路沿覆盖局部补拟合。R4 几何存在但语义为 unknown 时，
        # 正是本补丁需要重新以原始硬高差和道路关系审查的对象，不能静默排除。
        if record['semantic_class'] != 'curb_candidate':
            continue
        for link in record['links']:
            covered[(record['side_sign'], link['sample_id'])].append(link['lateral'])
    local_records = []
    local_id = 0
    for track in local_edge_tracks(samples, edge_index):
        uncovered = []
        for item in track:
            # 已有 R4 几何即使未被分类为路沿，也不由本补丁重复生成第二条边界记录。
            existing = covered.get((item['side'], item['sample_id']), ())
            if any(abs(item['lateral'] - lateral) <= CURB_INNER_DISTANCE_TOLERANCE_M for lateral in existing):
                continue
            uncovered.append(item)
        for fragment in split_track(uncovered):
            built = local_band_from_track(fragment, local_id)
            if built is None:
                continue
            local_id += 1
            band, side, links = built
            local_records.append(classify_band_association(band, side, links, hierarchy, samples_by_id,
                                                           ground, road_cells, edge_index))
    return local_records


def points_for_record(record, color):
    """PCD 仅用于人工检视；机器语义始终以 CSV 字段为准。"""
    links = record['links']
    z_values = [item['edge_z'] for item in links if item['edge_z'] is not None]
    if not z_values:
        z_values = [item['inside'][1]['z'] for item in links if item['inside'] is not None]
    z = median(z_values)
    if z is None:
        return []
    result = []
    step = 0.20
    start, end = record['start_projection'], record['end_projection']
    count = max(1, int(math.ceil((end - start) / step)))
    for index in range(count + 1):
        projection = start + (end - start) * float(index) / float(count)
        x, y = band_point(record['band'], projection)
        result.append((x, y, z + 0.025, color))
    return result


def qualifies_low_road_evidence_review(record):
    """判断局部硬高差是否进入低道路证据复核层。

    该层故意不降低路沿判定阈值：它要求连续硬高差、内侧稳定地面和可观测高度跳变均已满足，
    仅把“道路完整搜索面支持不足”的段单独可视化。road_support 为 0 也保留，因为这正是
    需要排查阶段3道路覆盖是否过窄、或该边界是否根本不属于道路的情形。
    """
    height = record['local_height_delta_m']
    return (record['boundary_source'] == 'stage02_hard_edge_local_fit' and
            record['continuous_length_m'] >= LOW_ROAD_REVIEW_MIN_CONTINUITY_M and
            record['hard_edge_support_fraction'] >= CURB_MIN_EDGE_SUPPORT and
            record['inside_ground_support_fraction'] >= CURB_MIN_GROUND_SUPPORT and
            height is not None and abs(height) >= CURB_MIN_HEIGHT_M and
            record['road_support_fraction'] < CURB_MIN_ROAD_SUPPORT)


def ribbon_cells(samples_by_id, inner_record, outer_record, ground):
    """收集两条边界之间已存在的稳定地面栅格，绝不填补未观测栅格。"""
    inner_by_id = dict((item['sample_id'], item) for item in inner_record['links'])
    outer_by_id = dict((item['sample_id'], item) for item in outer_record['links'])
    common_ids = sorted(set(inner_by_id).intersection(outer_by_id))
    cells = set()
    support = []
    for sample_id in common_ids:
        sample = samples_by_id[sample_id]
        low = inner_by_id[sample_id]['lateral']
        high = outer_by_id[sample_id]['lateral']
        if low > high:
            low, high = high, low
        expected = []
        for alpha in (0.25, 0.50, 0.75):
            lateral = low * (1.0 - alpha) + high * alpha
            expected.append(nearest_ground(ground, sample['x'] + sample['nx'] * lateral,
                                           sample['y'] + sample['ny'] * lateral, 2) is not None)
        support.append(all(expected))
        radius = int(math.ceil((max(abs(low), abs(high)) + 0.4) / base.GRID_M))
        center = base.grid_key(sample['x'], sample['y'])
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                cell = (center[0] + dx, center[1] + dy)
                if cell not in ground:
                    continue
                x, y = base.cell_center(cell)
                longitudinal = (x - sample['x']) * sample['tx'] + (y - sample['y']) * sample['ty']
                lateral = (x - sample['x']) * sample['nx'] + (y - sample['y']) * sample['ny']
                if abs(longitudinal) <= 0.35 and low <= lateral <= high:
                    cells.add(cell)
    return common_ids, fraction(support), cells


def build_sidewalk_records(records, samples_by_id, ground):
    """按横断面连续配对内外边界，而不是把两条完整长带的平均距离直接配对。

    R4 仍会在长缺口处分段；若外边界由相邻 R4 带接力，旧版“整带对整带”会把
    共同样本清空，造成视觉上本应连续的已观测地面出现无意义空洞。
    """
    by_side = defaultdict(list)
    for record in records:
        by_side[record['side']].append(record)
    surfaces = []
    confirmed_cells = set()
    paired_curb_keys = set()
    surface_id = 0
    for side, candidates in sorted(by_side.items()):
        curbs = [item for item in candidates if item['semantic_class'] == 'curb_candidate']
        for inner in curbs:
            inner_by_id = dict((item['sample_id'], item) for item in inner['links'])
            outer_links = dict((item['band_id'], dict((link['sample_id'], link) for link in item['links']))
                               for item in candidates if item is not inner)
            selections = {}
            for sample_id, inner_link in inner_by_id.items():
                options = []
                for outer in candidates:
                    if outer is inner or outer['hard_edge_support_fraction'] < 0.35:
                        continue
                    outer_link = outer_links.get(outer['band_id'], {}).get(sample_id)
                    if outer_link is None:
                        continue
                    width = abs(outer_link['lateral']) - abs(inner_link['lateral'])
                    if width < SIDEWALK_MIN_WIDTH_M or width > SIDEWALK_MAX_WIDTH_M:
                        continue
                    parallel_cos = abs(inner['band']['tx'] * outer['band']['tx'] +
                                       inner['band']['ty'] * outer['band']['ty'])
                    if parallel_cos < 0.94:
                        continue
                    # 优先同一横断面上的较宽、但不无限向外跳跃的候选；宽度只用于选择
                    # 外边界，真正确认仍由稳定地面、连续性和最小确认宽度共同决定。
                    width_preference = 1.0 - min(abs(width - 1.80), 1.80) / 1.80
                    score = (int(width >= CONFIRMED_SIDEWALK_MIN_WIDTH_M), width_preference,
                             outer['hard_edge_support_fraction'], parallel_cos)
                    options.append((score, outer, outer_link, width, parallel_cos))
                if options:
                    score, outer, outer_link, width, parallel_cos = max(options, key=lambda item: item[0])
                    selections[sample_id] = {'inner_link': inner_link, 'outer': outer,
                                             'outer_link': outer_link, 'width': width,
                                             'parallel_cos': parallel_cos}
            if not selections:
                surface_id += 1
                surfaces.append({
                    'surface_id': surface_id, 'side': side, 'inner_band_id': inner['band_id'],
                    'outer_band_id': '', 'width_m': '', 'parallel_cos': '', 'continuous_length_m': 0.0,
                    'stable_ground_support_fraction': 0.0, 'surface_cell_count': 0,
                    'classification': 'possible_sidewalk_or_shoulder', 'confidence': 0.20,
                    'evidence_state': 'unknown', 'reason': '缺少同侧、平行且宽度合理的外边界带', 'cells': set(),
                })
                continue
            # 外边界带可以在物理连续处切换；仅在采样断开或横向跳变时断开表面记录。
            runs, current, previous_id, previous_lateral = [], [], None, None
            for sample_id in sorted(selections):
                outer_lateral = selections[sample_id]['outer_link']['lateral']
                if current and (sample_id > previous_id + 3 or abs(outer_lateral - previous_lateral) > 0.75):
                    runs.append(current)
                    current = []
                current.append(sample_id)
                previous_id, previous_lateral = sample_id, outer_lateral
            if current:
                runs.append(current)
            for run_ids in runs:
                surface_id += 1
                inner_fragment = dict(inner)
                inner_fragment['links'] = [selections[item]['inner_link'] for item in run_ids]
                outer_fragment = {'links': [selections[item]['outer_link'] for item in run_ids]}
                common, ground_support, cells = ribbon_cells(samples_by_id, inner_fragment, outer_fragment, ground)
                continuity = path_length_for_ids(samples_by_id, common)
                widths = [selections[item]['width'] for item in run_ids]
                parallel_cos = median([selections[item]['parallel_cos'] for item in run_ids])
                outer_records = [selections[item]['outer'] for item in run_ids]
                outer_ids = sorted(set(item['band_id'] for item in outer_records))
                outer_hard = median([item['hard_edge_support_fraction'] for item in outer_records])
                width = median(widths)
                confirmed = (width >= CONFIRMED_SIDEWALK_MIN_WIDTH_M and
                             continuity >= SIDEWALK_MIN_CONTINUITY_M and
                             ground_support >= SIDEWALK_MIN_GROUND_SUPPORT and len(cells) >= 12)
                if confirmed:
                    classification, state, reason = 'confirmed_sidewalk_or_shoulder', 'observed', ''
                    confidence = min(1.0, 0.25 * ground_support + 0.25 * min(1.0, continuity / 8.0) +
                                     0.20 * parallel_cos + 0.15 * inner['classification_confidence'] +
                                     0.15 * outer_hard)
                    confirmed_cells.update(cells)
                    paired_curb_keys.add((inner['band_id'], inner['side']))
                else:
                    classification, state = 'possible_sidewalk_or_shoulder', 'inferred'
                    confidence = min(0.49, 0.25 * ground_support + 0.20 * min(1.0, continuity / 8.0) +
                                     0.15 * parallel_cos)
                    reasons = []
                    if continuity < SIDEWALK_MIN_CONTINUITY_M:
                        reasons.append('内外边界共同连续长度不足')
                    if width < CONFIRMED_SIDEWALK_MIN_WIDTH_M:
                        reasons.append('内外边界间距小于确认人行道/路肩最小宽度，可能是双线硬高差或窄路肩')
                    if ground_support < SIDEWALK_MIN_GROUND_SUPPORT:
                        reasons.append('两边界间稳定地面支持不足')
                    if len(cells) < 12:
                        reasons.append('两边界间可观测地面格不足')
                    reason = '|'.join(reasons)
                surfaces.append({
                    'surface_id': surface_id, 'side': side, 'inner_band_id': inner['band_id'],
                    'outer_band_id': '|'.join(str(item) for item in outer_ids), 'width_m': width,
                    'parallel_cos': parallel_cos, 'continuous_length_m': continuity,
                    'stable_ground_support_fraction': ground_support, 'surface_cell_count': len(cells),
                    'classification': classification, 'confidence': confidence, 'evidence_state': state,
                    'reason': reason, 'cells': cells,
                })
    return surfaces, confirmed_cells, paired_curb_keys


def write_boundary_records(path, records):
    header = ('boundary_band_id', 'boundary_source', 'source_primitive_ids', 'side', 'trajectory_role',
              'start_x', 'start_y', 'end_x', 'end_y', 'associated_length_m', 'continuous_length_m',
              'relative_road_distance_m', 'local_height_delta_m', 'direction_angle_deg',
              'hard_edge_support_fraction', 'road_support_fraction', 'observed_road_support_fraction',
              'topology_support_fraction', 'inside_ground_support_fraction',
              'outside_ground_support_fraction', 'inner_boundary_match_fraction',
              'local_road_edge_match_fraction', 'semantic_class',
              'classification_confidence', 'evidence_state', 'rejection_reason')
    with open(path, 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for item in sorted(records, key=lambda record: (record['side'], record['band_id'])):
            writer.writerow((item['band_id'], item['boundary_source'], item['source_primitive_ids'], item['side'], item['trajectory_role'],
                             '%.6f' % item['start_x'], '%.6f' % item['start_y'], '%.6f' % item['end_x'],
                             '%.6f' % item['end_y'], '%.6f' % item['associated_length_m'],
                             '%.6f' % item['continuous_length_m'], '%.6f' % item['relative_road_distance_m'],
                             '' if item['local_height_delta_m'] is None else '%.6f' % item['local_height_delta_m'],
                             '%.6f' % item['direction_angle_deg'], '%.6f' % item['hard_edge_support_fraction'],
                             '%.6f' % item['road_support_fraction'], '%.6f' % item['observed_road_support_fraction'],
                             '%.6f' % item['topology_support_fraction'],
                             '%.6f' % item['inside_ground_support_fraction'],
                             '%.6f' % item['outside_ground_support_fraction'],
                             '%.6f' % item['inner_boundary_match_fraction'],
                             '%.6f' % item['local_road_edge_match_fraction'], item['semantic_class'],
                             '%.6f' % item['classification_confidence'], item['evidence_state'],
                             item['rejection_reason']))


def write_sidewalk_records(path, surfaces):
    header = ('surface_id', 'side', 'inner_boundary_band_id', 'outer_boundary_band_id', 'width_m',
              'parallel_cos', 'continuous_length_m', 'stable_ground_support_fraction', 'surface_cell_count',
              'classification', 'classification_confidence', 'evidence_state', 'rejection_reason')
    with open(path, 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for item in surfaces:
            writer.writerow((item['surface_id'], item['side'], item['inner_band_id'], item['outer_band_id'],
                             '' if item['width_m'] == '' else '%.6f' % item['width_m'],
                             '' if item['parallel_cos'] == '' else '%.6f' % item['parallel_cos'],
                             '%.6f' % item['continuous_length_m'],
                             '%.6f' % item['stable_ground_support_fraction'], item['surface_cell_count'],
                             item['classification'], '%.6f' % item['confidence'], item['evidence_state'],
                             item['reason']))


def write_rejected_records(path, records, surfaces):
    with open(path, 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('record_type', 'record_id', 'side', 'classification', 'evidence_state', 'reason'))
        for item in records:
            if item['semantic_class'] == 'unknown_boundary':
                writer.writerow(('boundary', item['band_id'], item['side'], item['semantic_class'],
                                 item['evidence_state'], item['rejection_reason']))
        for item in surfaces:
            if item['classification'] != 'confirmed_sidewalk_or_shoulder':
                writer.writerow(('sidewalk_surface', item['surface_id'], item['side'], item['classification'],
                                 item['evidence_state'], item['reason']))


def surface_points(cells, ground, color):
    points = []
    for cell in sorted(cells):
        x, y = base.cell_center(cell)
        points.append((x, y, ground[cell]['z'] + 0.02, color))
    return points


def write_pcd_or_mark_empty(path, points, empty_layers, reason):
    """部分 PCD 读取器拒绝 POINTS 0；空语义层以 JSON 明确表达，绝不伪造占位点。"""
    if points:
        base.write_pcd(path, points)
        return
    empty_layers.append({'pcd_filename': os.path.basename(path), 'reason': reason})


def pcd_points(path):
    require_file(path, 'PCD 证据层')
    return base.read_binary_pcd(path)


def write_report(path, stage02, stage03a, stage03, records, surfaces, confirmed_cells, open_points,
                 empty_layers, suppressed_layers, low_road_review_records):
    class_counts = defaultdict(int)
    state_counts = defaultdict(int)
    source_counts = defaultdict(int)
    for item in records:
        class_counts[item['semantic_class']] += 1
        state_counts[item['evidence_state']] += 1
        source_counts[item['boundary_source']] += 1
    surface_counts = defaultdict(int)
    for item in surfaces:
        surface_counts[item['classification']] += 1
    report = {
        'schema': 'fast_livo_scene_pipeline_stage04_curb_sidewalk_extraction/v1',
        'purpose': '只读阶段2/3A/3证据，保守分类道路两侧边界与人行道/路肩；未知区域不补齐。',
        'source_stage02': os.path.abspath(stage02), 'source_stage03a': os.path.abspath(stage03a),
        'source_stage03_final_handoff': os.path.abspath(stage03),
        'input_policy': [
            '固定宽度道路候选仅用于连续搜索，未用于实测道路宽度或观测路沿判断。',
            '持续硬高差、长边界带和PCD颜色不是语义标签；分类由几何、道路关系和观测支持共同决定。',
            '墙根与绿化在当前冻结输入下无法可靠二分时保留 wall_or_green_boundary_candidate。',
            '人行道确认需要道路相邻、同侧内外平行边界、连续稳定地面和观测支持。',
        ],
        'parameters': {
            'max_direction_deviation_deg': MAX_DIRECTION_DEVIATION_DEG,
            'curb_min_continuity_m': CURB_MIN_CONTINUITY_M,
            'curb_height_abs_m': [CURB_MIN_HEIGHT_M, CURB_MAX_HEIGHT_M],
            'sidewalk_width_m': [SIDEWALK_MIN_WIDTH_M, SIDEWALK_MAX_WIDTH_M],
            'sidewalk_min_stable_ground_support': SIDEWALK_MIN_GROUND_SUPPORT,
        },
        'boundary_record_count': len(records), 'boundary_class_counts': dict(sorted(class_counts.items())),
        'boundary_evidence_state_counts': dict(sorted(state_counts.items())),
        'boundary_source_counts': dict(sorted(source_counts.items())),
        'sidewalk_surface_counts': dict(sorted(surface_counts.items())),
        'confirmed_sidewalk_cells': len(confirmed_cells),
        'possible_sidewalk_surface_records': surface_counts.get('possible_sidewalk_or_shoulder', 0),
        'junction_or_open_area_source_points': len(open_points),
        'low_road_evidence_hard_edge_review_records': len(low_road_review_records),
        'empty_pcd_layers': empty_layers,
        'suppressed_visual_layers': suppressed_layers,
        'limits': [
            'confirmed_sidewalk_or_shoulder 表示当前几何与观测条件下的确认候选，不是无外部标注的法律/无障碍设施属性。',
            '本阶段不输出最终道路面、最终道路拓扑、完整建筑或渲染几何。',
            'PCD 颜色仅供目视审查；CSV 是机器可读语义、来源和拒绝原因的唯一依据。',
            'low_road_evidence_hard_edge_review 是人工复核层，不是路沿、道路边界或渲染输入。',
        ],
    }
    with open(path, 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    return report


def main():
    parser = argparse.ArgumentParser(description='阶段4：路沿与人行道/路肩的可审计提取')
    parser.add_argument('--stage02', required=True, help='阶段2 R2 输出目录')
    parser.add_argument('--stage03a', required=True, help='阶段3A R4 输出目录')
    parser.add_argument('--stage03', required=True, help='阶段3最终交接输出目录')
    parser.add_argument('--output', required=True, help='新的阶段4输出目录；必须不存在')
    arguments = parser.parse_args()
    stage02, stage03a, stage03 = map(os.path.abspath, (arguments.stage02, arguments.stage03a, arguments.stage03))
    output = os.path.abspath(arguments.output)
    if os.path.exists(output):
        fail('阶段4输出目录已存在，拒绝覆盖：' + output)

    grid_path = os.path.join(stage02, 'evidence', 'geometric_observation_grid.csv')
    hard_path = os.path.join(stage02, 'evidence', 'continuous_hard_height_edge_support.pcd')
    bands_path = os.path.join(stage03a, 'evidence', 'merged_boundary_bands.csv')
    hierarchy_path = os.path.join(stage03a, 'evidence', 'trajectory_boundary_hierarchy.csv')
    cross_path = os.path.join(stage03, 'evidence', 'road_cross_section_confidence.csv')
    road_path = os.path.join(stage03, 'evidence', 'road_surface_candidate_complete_cells.csv')
    open_paths = [os.path.join(stage03, 'evidence', name) for name in (
        'one_sided_open_candidate_support.pcd', 'open_area_continuation_candidate_support.pcd',
        'junction_turn_node_candidate_support.pcd')]
    for path, description in ((grid_path, '阶段2几何网格'), (hard_path, '阶段2硬高差'),
                              (bands_path, '阶段3A边界带'), (hierarchy_path, '阶段3A层级'),
                              (cross_path, '阶段3横断面'), (road_path, '阶段3道路候选')):
        require_file(path, description)

    ground = read_grid(grid_path)
    bands = read_bands(bands_path)
    hierarchy = read_hierarchy(hierarchy_path)
    samples = read_cross_sections(cross_path)
    road_cells = read_road_cells(road_path)
    hard_points = pcd_points(hard_path)
    edge_index = PointIndex(hard_points)
    r4_records, samples_by_id = associate_boundaries(bands, samples, hierarchy, ground, road_cells, edge_index)
    local_records = extract_local_hard_edge_records(samples, samples_by_id, r4_records, hierarchy,
                                                    ground, road_cells, edge_index)
    records = r4_records + local_records
    low_road_review_records = [record for record in local_records
                               if qualifies_low_road_evidence_review(record)]
    surfaces, confirmed_cells, unused_paired_curbs = build_sidewalk_records(records, samples_by_id, ground)

    os.makedirs(os.path.join(output, 'evidence'))
    os.makedirs(os.path.join(output, 'validation'))
    evidence = os.path.join(output, 'evidence')
    write_boundary_records(os.path.join(evidence, 'boundary_semantic_records.csv'), records)
    write_boundary_records(os.path.join(evidence, 'combined_boundary_semantic_records.csv'), records)
    write_boundary_records(os.path.join(evidence, 'local_hard_edge_boundary_records.csv'), local_records)
    write_boundary_records(os.path.join(evidence, 'low_road_evidence_hard_edge_review_records.csv'),
                           low_road_review_records)
    write_sidewalk_records(os.path.join(evidence, 'sidewalk_surface_records.csv'), surfaces)
    write_rejected_records(os.path.join(evidence, 'rejected_boundary_records.csv'), records, surfaces)

    colors = {
        'curb_candidate': base.packed_rgb(245, 155, 40),
        'step_candidate': base.packed_rgb(180, 80, 230),
        'wall_or_green_boundary_candidate': base.packed_rgb(65, 180, 90),
        'unknown_boundary': base.packed_rgb(150, 150, 150),
    }
    left_points, right_points = [], []
    by_class = defaultdict(list)
    for record in records:
        points = points_for_record(record, colors[record['semantic_class']])
        (left_points if record['side'] == 'left' else right_points).extend(points)
        by_class[record['semantic_class']].extend(points)
    local_points = []
    for record in local_records:
        # 蓝色审查层只显示道路相邻、内侧有稳定地面且存在实际高度跳变的局部段，避免把
        # 远离道路的无关硬高差全部画入人工检查图层。语义仍以 CSV 为准。
        height = record['local_height_delta_m']
        if (record['road_support_fraction'] >= CURB_MIN_ROAD_SUPPORT and
                record['inside_ground_support_fraction'] >= CURB_MIN_GROUND_SUPPORT and
                height is not None and abs(height) >= CURB_MIN_HEIGHT_M):
            local_points.extend(points_for_record(record, base.packed_rgb(80, 155, 245)))
    low_road_review_points = []
    for record in low_road_review_records:
        low_road_review_points.extend(points_for_record(record, base.packed_rgb(210, 90, 220)))
    empty_layers = []
    write_pcd_or_mark_empty(os.path.join(evidence, 'left_boundary_candidate_support.pcd'), left_points,
                            empty_layers, '没有可关联到完整中心线搜索范围的左侧边界带')
    write_pcd_or_mark_empty(os.path.join(evidence, 'right_boundary_candidate_support.pcd'), right_points,
                            empty_layers, '没有可关联到完整中心线搜索范围的右侧边界带')
    write_pcd_or_mark_empty(os.path.join(evidence, 'curb_candidate_support.pcd'), by_class['curb_candidate'],
                            empty_layers, '没有满足路沿候选证据规则的边界带')
    write_pcd_or_mark_empty(os.path.join(evidence, 'step_candidate_support.pcd'), by_class['step_candidate'],
                            empty_layers, '没有满足台阶候选证据规则的边界带')
    write_pcd_or_mark_empty(os.path.join(evidence, 'wall_or_green_boundary_candidate_support.pcd'),
                            by_class['wall_or_green_boundary_candidate'], empty_layers,
                            '没有满足墙根/绿化组合候选证据规则的边界带')
    write_pcd_or_mark_empty(os.path.join(evidence, 'local_hard_edge_boundary_support.pcd'), local_points,
                            empty_layers, '没有R4未覆盖且满足局部追踪规则的原始硬高差边界段')
    write_pcd_or_mark_empty(os.path.join(evidence, 'low_road_evidence_hard_edge_review_support.pcd'),
                            low_road_review_points, empty_layers,
                            '没有同时满足连续硬高差、内侧稳定地面和实际高度跳变的低道路证据局部段')
    confirmed_points = surface_points(confirmed_cells, ground, base.packed_rgb(80, 210, 225))
    write_pcd_or_mark_empty(os.path.join(evidence, 'sidewalk_confirmed_support.pcd'), confirmed_points,
                            empty_layers, '没有满足确认人行道/路肩规则的稳定地面')

    # 可能人行道/路肩不是可确认的几何面。旧版将阶段3的大范围候选 PCD 原样可视化，
    # 会把已被阶段4降级的窄双线再次画成黄色“面”。R3 改为只在 CSV 中保留可能/拒绝
    # 记录，不生成可能面 PCD，避免渲染层或人工检视误将其当作事实图层。
    suppressed_layers = [{
        'pcd_filename': 'sidewalk_possible_support.pcd',
        'reason': '产品策略禁用可能人行道/路肩的可视化PCD；请仅通过 sidewalk_surface_records.csv 审计。',
    }]
    with open(os.path.join(evidence, 'suppressed_visual_layers.json'), 'w') as handle:
        json.dump({'schema': 'fast_livo_scene_pipeline_stage04_suppressed_visual_layers/v1',
                   'policy': '可能候选不得作为面图层显示或被渲染消费；保留CSV以便追溯。',
                   'suppressed_layers': suppressed_layers}, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(evidence, 'low_road_evidence_hard_edge_review_contract.json'), 'w') as handle:
        json.dump({
            'schema': 'fast_livo_scene_pipeline_stage04_low_road_evidence_hard_edge_review/v1',
            'pcd_filename': 'low_road_evidence_hard_edge_review_support.pcd',
            'csv_filename': 'low_road_evidence_hard_edge_review_records.csv',
            'purpose': '人工复核阶段2连续硬高差与阶段3道路搜索面关系；不生成新的道路或路沿事实。',
            'criteria': {
                'boundary_source': 'stage02_hard_edge_local_fit',
                'min_continuous_length_m': LOW_ROAD_REVIEW_MIN_CONTINUITY_M,
                'min_hard_edge_support': CURB_MIN_EDGE_SUPPORT,
                'min_inside_ground_support': CURB_MIN_GROUND_SUPPORT,
                'min_abs_height_delta_m': CURB_MIN_HEIGHT_M,
                'max_road_support_exclusive': CURB_MIN_ROAD_SUPPORT,
            },
            'consumption_policy': [
                '仅供人工审计和后续道路证据诊断。',
                '不得作为 curb_candidate、road_surface、sidewalk 或任何渲染层输入。',
                '道路支持为零可能表示道路候选过窄，也可能表示该硬高差是墙根、台阶、绿化边或开放区边缘。',
            ],
            'record_count': len(low_road_review_records),
        }, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    open_points = []
    for path in open_paths:
        for point in pcd_points(path):
            open_points.append((point[0], point[1], point[2] + 0.025, base.packed_rgb(210, 120, 210)))
    write_pcd_or_mark_empty(os.path.join(evidence, 'junction_or_open_area_candidate_support.pcd'), open_points,
                            empty_layers, '没有阶段3遗留的路口或开放区域候选支持点')
    with open(os.path.join(evidence, 'empty_layers.json'), 'w') as handle:
        json.dump({'schema': 'fast_livo_scene_pipeline_stage04_empty_layers/v1',
                   'policy': '无候选层不写 POINTS 0 PCD，以免第三方读取器报错；不得用占位点伪造证据。',
                   'empty_layers': empty_layers}, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')

    write_report(os.path.join(output, 'validation', 'stage04_curb_sidewalk_report.json'),
                 stage02, stage03a, stage03, records, surfaces, confirmed_cells, open_points,
                 empty_layers, suppressed_layers, low_road_review_records)
    with open(os.path.join(output, 'stage04_complete.json'), 'w') as handle:
        json.dump({
            'status': 'complete', 'stage': '04_curb_sidewalk_extraction',
            'source_stage02': stage02, 'source_stage03a': stage03a, 'source_stage03_final_handoff': stage03,
            'policy': '只读冻结证据；未知区域不补成路沿或确认人行道。',
        }, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段4失败：{0}'.format(error), file=sys.stderr)
        sys.exit(1)
