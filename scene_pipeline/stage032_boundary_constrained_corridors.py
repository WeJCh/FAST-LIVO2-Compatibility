#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 3.2：以已确认的长边界带约束道路核心和人行道候选。

本阶段不再读取离散硬高差点，也不进行二维泛洪。它只使用阶段 3A R4 的有限长度
边界带、边界带的轨迹层级关系，以及阶段 2 的稳定地面：

* 道路核心：同一轨迹横断面上，左右最近的长边界带之间；
* 人行道/路肩候选：同一侧内、外两条平行长边界带之间；
* 其它稳定地面：保持未知，不能因为边界缺失而向外扩张。

输出仍是几何候选，不会把内外带之间的区域直接命名为“人行道”。例如绿化带、
建筑退界也可能具有同样的几何形态，须在后续阶段结合材质、语义或人工规则确认。
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


MIN_BAND_LENGTH_M = 4.0
MAX_LATERAL_M = 10.0
MAX_ORIENTATION_DEVIATION_DEG = 35.0
ORIENTATION_COS = math.cos(math.radians(MAX_ORIENTATION_DEVIATION_DEG))
ROAD_MIN_WIDTH_M = 1.20
ROAD_MAX_WIDTH_M = 9.0
SIDEWALK_MIN_WIDTH_M = 0.45
SIDEWALK_MAX_WIDTH_M = 6.0
TRAJECTORY_SAMPLE_SPACING_M = 0.40
# 两端双边界的横向距离连续时，可沿已采集轨迹跨越中等长度遮挡/路口缺口。
# 这不是自由扩张：仍要求两端均有有效双边锚点，且左右边界变化各不超过 2.5 m。
REGULAR_BRIDGE_MAX_PATH_M = 20.0
INTERSECTION_BRIDGE_MAX_PATH_M = 20.0
BRIDGE_MAX_SIDE_CHANGE_M = 2.5
TURN_BRIDGE_MIN_DEG = 40.0
TURN_BRIDGE_MAX_DEG = 135.0
TURN_BRIDGE_MAX_PATH_M = 12.0
LONG_CORRIDOR_MAX_PATH_M = 40.0
LONG_CORRIDOR_MAX_TURN_DEG = 12.0
LONG_CORRIDOR_MAX_SIDE_CHANGE_M = 1.0
LONG_CORRIDOR_STRONG_SIDE_FRACTION = 0.60
LONG_CORRIDOR_WEAK_SIDE_FRACTION = 0.25
ONE_SIDE_MIN_CONTINUOUS_SAMPLES = 6
ONE_SIDE_REFERENCE_MAX_SAMPLES = 40
ONE_SIDE_REFERENCE_DIRECTION_COS = math.cos(math.radians(30.0))
ONE_SIDE_MIN_INFERRED_HALF_WIDTH_M = 0.60
OPEN_CONTINUATION_MAX_PATH_M = 15.0
OPEN_CONTINUATION_MAX_WIDTH_CHANGE_M = 1.50
# 由本数据已确认“窄道路簇”（双边宽度 ≤5.5m）的稳健中值约 3.47m 取整得到。
# 它是产品先验，不是从无边界区域自动观测到的道路真实宽度。
DEFAULT_CORRIDOR_WIDTH_M = 3.50


def fail(message):
    raise RuntimeError(message)


def read_bands(path):
    """读取阶段 3A 的有限线段，不能从彩色 PCD 反推其身份。"""
    bands = []
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('band_id', 'tangent_x', 'tangent_y', 'normal_x', 'normal_y', 'rho_m',
                'start_projection_m', 'end_projection_m', 'length_m')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('边界带 CSV 字段不兼容：' + path)
        for row in reader:
            band = {
                'id': int(row['band_id']),
                'tx': float(row['tangent_x']), 'ty': float(row['tangent_y']),
                'nx': float(row['normal_x']), 'ny': float(row['normal_y']),
                'rho': float(row['rho_m']),
                'start': float(row['start_projection_m']),
                'end': float(row['end_projection_m']),
                'length': float(row['length_m']),
            }
            if band['length'] >= MIN_BAND_LENGTH_M:
                bands.append(band)
    if not bands:
        fail('没有长度至少 {0:.1f} m 的长边界带'.format(MIN_BAND_LENGTH_M))
    return bands


def read_hierarchy(path):
    result = {}
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('band_id', 'trajectory_role', 'primary_longest_continuous_sample_count',
                'secondary_longest_continuous_sample_count')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('边界层级 CSV 字段不兼容：' + path)
        for row in reader:
            result[int(row['band_id'])] = {
                'role': row['trajectory_role'],
                'primary_run': int(row['primary_longest_continuous_sample_count']),
                'secondary_run': int(row['secondary_longest_continuous_sample_count']),
            }
    return result


def band_candidates(sample, side, bands):
    """取得横断面一侧的有限边界带，从近至远排序。"""
    candidates = []
    for band in bands:
        if abs(sample['tx'] * band['tx'] + sample['ty'] * band['ty']) < ORIENTATION_COS:
            continue
        coordinate = sample['x'] * band['tx'] + sample['y'] * band['ty']
        # 只在线段自身范围内关联。容忍 0.5m 是采样和直线拟合端点误差，不跨长缺口。
        if coordinate < band['start'] - 0.5 or coordinate > band['end'] + 0.5:
            continue
        signed_normal = band['nx'] * sample['x'] + band['ny'] * sample['y'] - band['rho']
        closest_x = sample['x'] - signed_normal * band['nx']
        closest_y = sample['y'] - signed_normal * band['ny']
        lateral = ((closest_x - sample['x']) * sample['nx'] +
                   (closest_y - sample['y']) * sample['ny'])
        if side * lateral <= 0.0:
            continue
        distance = abs(lateral)
        if distance <= MAX_LATERAL_M:
            candidates.append((distance, band))
    return sorted(candidates, key=lambda item: (item[0], item[1]['id']))


def select_outer_band(inner_distance, candidates):
    """从同侧第二及更外的边界中选最靠近内带、宽度合理的一条。"""
    for distance, band in candidates[1:]:
        width = distance - inner_distance
        if SIDEWALK_MIN_WIDTH_M <= width <= SIDEWALK_MAX_WIDTH_M:
            return distance, band
    return None, None


def add_lateral_ribbon(sample, lateral_low, lateral_high, ground, coverage):
    """仅给已有稳定地面栅格加候选标签，绝不生成不存在的地面。"""
    longitudinal_half_m = 0.35
    lateral_limit = max(abs(lateral_low), abs(lateral_high))
    radius = int(math.ceil((longitudinal_half_m + lateral_limit) / base.GRID_M)) + 1
    center = base.grid_key(sample['x'], sample['y'])
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            cell = (center[0] + dx, center[1] + dy)
            if cell not in ground:
                continue
            x, y = base.cell_center(cell)
            relative_x = x - sample['x']
            relative_y = y - sample['y']
            longitudinal = relative_x * sample['tx'] + relative_y * sample['ty']
            lateral = relative_x * sample['nx'] + relative_y * sample['ny']
            if abs(longitudinal) <= longitudinal_half_m and lateral_low <= lateral <= lateral_high:
                coverage[cell] = coverage.get(cell, 0) + 1


def closest_ground_z(ground, x, y):
    return base.closest_ground_z(ground, x, y)


def write_cell_csv(path, cells, ground, meaning):
    with open(path, 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('cell_ix', 'cell_iy', 'x', 'y', 'ground_z', 'cross_section_support_count',
                         'ground_frame_count', 'free_frame_count', 'observation_confidence', 'meaning'))
        for cell in sorted(cells):
            record = ground[cell]
            x, y = base.cell_center(cell)
            writer.writerow((cell[0], cell[1], '%.6f' % x, '%.6f' % y,
                             '%.6f' % record['z'], cells[cell], record['ground_frames'],
                             record['free_frames'], '%.6f' % record['confidence'], meaning))


def path_length(samples, first_id, second_id):
    return sum(math.hypot(samples[index]['x'] - samples[index - 1]['x'],
                          samples[index]['y'] - samples[index - 1]['y'])
               for index in range(first_id + 1, second_id + 1))


def intersection_samples(samples):
    """找出轨迹在地图上与非相邻、明显不同方向轨迹相遇的位置。"""
    index = defaultdict(list)
    for sample_id, sample in enumerate(samples):
        index[(int(math.floor(sample['x'] / 2.0)), int(math.floor(sample['y'] / 2.0)))].append(sample_id)
    result = set()
    for sample_id, sample in enumerate(samples):
        key_x, key_y = int(math.floor(sample['x'] / 2.0)), int(math.floor(sample['y'] / 2.0))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for other_id in index.get((key_x + dx, key_y + dy), ()):
                    if abs(other_id - sample_id) <= 15:
                        continue
                    other = samples[other_id]
                    if math.hypot(sample['x'] - other['x'], sample['y'] - other['y']) > 3.0:
                        continue
                    if abs(sample['tx'] * other['tx'] + sample['ty'] * other['ty']) < 0.82:
                        result.add(sample_id)
                        break
                if sample_id in result:
                    break
            if sample_id in result:
                break
    return result


def bridge_internal_road_gaps(samples, cross_sections, bands, ground):
    """以三种受限证据模式桥接内部道路缺口，绝不桥接起止端。"""
    bridge_cells = {}
    bridge_cell_kinds = {}
    bridge_centerline = []
    records = []
    intersections = intersection_samples(samples)
    valid_ids = [item['sample_id'] for item in cross_sections if item['road_valid']]
    for first_id, second_id in zip(valid_ids, valid_ids[1:]):
        if second_id <= first_id + 1:
            continue
        first = cross_sections[first_id]
        second = cross_sections[second_id]
        first_left, first_right = first['left_inner'][0], first['right_inner'][0]
        second_left, second_right = second['left_inner'][0], second['right_inner'][0]
        distance = path_length(samples, first_id, second_id)
        is_intersection = any(index in intersections for index in range(first_id, second_id + 1))
        direction_dot = (samples[first_id]['tx'] * samples[second_id]['tx'] +
                         samples[first_id]['ty'] * samples[second_id]['ty'])
        turn_deg = math.degrees(math.acos(max(-1.0, min(1.0, direction_dot))))
        left_change = abs(first_left - second_left)
        right_change = abs(first_right - second_right)
        gap_ids = range(first_id + 1, second_id)
        left_seen = sum(1 for sample_id in gap_ids if band_candidates(samples[sample_id], 1.0, bands))
        right_seen = sum(1 for sample_id in gap_ids if band_candidates(samples[sample_id], -1.0, bands))
        gap_count = second_id - first_id - 1
        left_fraction = float(left_seen) / gap_count
        right_fraction = float(right_seen) / gap_count
        bridge_kind = None
        # 直路：双边距离连续的普通内部缺口。
        maximum = INTERSECTION_BRIDGE_MAX_PATH_M if is_intersection else REGULAR_BRIDGE_MAX_PATH_M
        if left_change <= BRIDGE_MAX_SIDE_CHANGE_M and right_change <= BRIDGE_MAX_SIDE_CHANGE_M and distance <= maximum:
            bridge_kind = 'regular_continuity'
        # 转弯：横向左右距离不再可逐侧比较，但可沿实际转向轨迹将大路/小路的
        # 两端半宽渐变连接。它表示“已通过的转向支路”，不是完整交叉口面。
        elif (TURN_BRIDGE_MIN_DEG <= turn_deg <= TURN_BRIDGE_MAX_DEG and
              distance <= TURN_BRIDGE_MAX_PATH_M and
              max(left_fraction, right_fraction) >= 0.50):
            bridge_kind = 'turn_continuity'
            maximum = TURN_BRIDGE_MAX_PATH_M
        # 长直走廊：允许超过常规 20m，但必须近似直行、端点宽度稳定，且段内两侧
        # 都有可见长带（至少一侧强、另一侧弱），因此不是无证据外推。
        elif (turn_deg <= LONG_CORRIDOR_MAX_TURN_DEG and distance <= LONG_CORRIDOR_MAX_PATH_M and
              left_change <= LONG_CORRIDOR_MAX_SIDE_CHANGE_M and
              right_change <= LONG_CORRIDOR_MAX_SIDE_CHANGE_M and
              max(left_fraction, right_fraction) >= LONG_CORRIDOR_STRONG_SIDE_FRACTION and
              min(left_fraction, right_fraction) >= LONG_CORRIDOR_WEAK_SIDE_FRACTION):
            bridge_kind = 'long_corridor_evidence'
            maximum = LONG_CORRIDOR_MAX_PATH_M
        if bridge_kind is None:
            continue
        for sample_id in range(first_id + 1, second_id):
            alpha = float(sample_id - first_id) / float(second_id - first_id)
            left = first_left * (1.0 - alpha) + second_left * alpha
            right = first_right * (1.0 - alpha) + second_right * alpha
            sample = samples[sample_id]
            previous_cells = set(bridge_cells)
            add_lateral_ribbon(sample, -right, left, ground, bridge_cells)
            for cell in set(bridge_cells).difference(previous_cells):
                bridge_cell_kinds[cell] = bridge_kind
            shift = (left - right) * 0.5
            center_x = sample['x'] + sample['nx'] * shift
            center_y = sample['y'] + sample['ny'] * shift
            bridge_centerline.append((center_x, center_y, closest_ground_z(ground, center_x, center_y)))
        records.append({
            'first_sample_id': first_id, 'second_sample_id': second_id,
            'gap_sample_count': second_id - first_id - 1, 'path_length_m': distance,
            'bridge_kind': bridge_kind, 'intersection_bridge': is_intersection,
            'maximum_path_m': maximum, 'turn_deg': turn_deg,
            'left_change_m': left_change, 'right_change_m': right_change,
            'left_visible_fraction': left_fraction, 'right_visible_fraction': right_fraction,
        })
    return bridge_cells, bridge_cell_kinds, bridge_centerline, records


def one_sided_open_candidates(cross_sections, ground):
    """由连续单侧长边与邻近已确认道路宽度生成低置信度开放面候选。

    该输出刻意不写入 road_core_candidate_support.pcd：它可能是路口、建筑入口广场、
    路肩或真实道路的无边界一侧，只能留待后续语义/产品规则决定是否采用。
    """
    groups = []
    start = None
    side = None
    for index, item in enumerate(cross_sections + [None]):
        current_side = None
        if item is not None and not item['road_valid']:
            has_left = item['left_inner'][1] is not None
            has_right = item['right_inner'][1] is not None
            if has_left != has_right:
                current_side = 'left' if has_left else 'right'
        if current_side == side and current_side is not None:
            continue
        if side is not None and index - start >= ONE_SIDE_MIN_CONTINUOUS_SAMPLES:
            groups.append((start, index - 1, side))
        start = index if current_side is not None else None
        side = current_side
    valid_ids = [item['sample_id'] for item in cross_sections if item['road_valid']]
    cells = {}
    records = []
    for first_id, last_id, side in groups:
        accepted = 0
        for sample_id in range(first_id, last_id + 1):
            item = cross_sections[sample_id]
            sample = item['sample']
            reference = None
            for valid_id in valid_ids:
                if abs(valid_id - sample_id) > ONE_SIDE_REFERENCE_MAX_SAMPLES:
                    continue
                candidate = cross_sections[valid_id]
                other = candidate['sample']
                direction = sample['tx'] * other['tx'] + sample['ty'] * other['ty']
                if direction < ONE_SIDE_REFERENCE_DIRECTION_COS:
                    continue
                key = (abs(valid_id - sample_id), valid_id)
                if reference is None or key < reference[0]:
                    reference = (key, candidate)
            if reference is None:
                continue
            width = reference[1]['road_width']
            if width is None:
                continue
            known = item['left_inner'][0] if side == 'left' else item['right_inner'][0]
            inferred = width - known
            if inferred < ONE_SIDE_MIN_INFERRED_HALF_WIDTH_M:
                continue
            if side == 'left':
                add_lateral_ribbon(sample, -inferred, known, ground, cells)
            else:
                add_lateral_ribbon(sample, -known, inferred, ground, cells)
            accepted += 1
        if accepted:
            records.append({'first_sample_id': first_id, 'last_sample_id': last_id, 'side': side,
                            'continuous_samples': last_id - first_id + 1,
                            'accepted_samples': accepted})
    return cells, records


def one_side_observation(item):
    """返回唯一可见内边的方向和距离；两侧都有或都无时返回空。"""
    has_left = item['left_inner'][1] is not None
    has_right = item['right_inner'][1] is not None
    if has_left == has_right:
        return None
    return ('left', item['left_inner'][0]) if has_left else ('right', item['right_inner'][0])


def nearest_confirmed_width(sample_id, cross_sections):
    """从同向、邻近的双边道路横断面取得宽度参考，避免跨路口借用宽度。"""
    sample = cross_sections[sample_id]['sample']
    best = None
    for item in cross_sections:
        if not item['road_valid'] or abs(item['sample_id'] - sample_id) > ONE_SIDE_REFERENCE_MAX_SAMPLES:
            continue
        other = item['sample']
        if sample['tx'] * other['tx'] + sample['ty'] * other['ty'] < ONE_SIDE_REFERENCE_DIRECTION_COS:
            continue
        key = (abs(item['sample_id'] - sample_id), item['sample_id'])
        if best is None or key < best[0]:
            best = (key, item['road_width'])
    return None if best is None else best[1]


def open_continuation_candidates(cross_sections, ground):
    """连接“单侧边—无边—单侧边”的有限内部空档，输出低置信度开放面。"""
    states = []
    for item in cross_sections:
        if one_side_observation(item) is not None:
            states.append('one')
        elif item['left_inner'][1] is None and item['right_inner'][1] is None:
            states.append('none')
        else:
            states.append('other')
    cells = {}
    records = []
    start = 0
    while start < len(cross_sections):
        if states[start] != 'none':
            start += 1
            continue
        end = start
        while end + 1 < len(cross_sections) and states[end + 1] == 'none':
            end += 1
        if start == 0 or end == len(cross_sections) - 1 or states[start - 1] != 'one' or states[end + 1] != 'one':
            start = end + 1
            continue
        first_id, second_id = start - 1, end + 1
        samples = [item['sample'] for item in cross_sections]
        distance = path_length(samples, first_id, second_id)
        first_width = nearest_confirmed_width(first_id, cross_sections)
        second_width = nearest_confirmed_width(second_id, cross_sections)
        if (distance > OPEN_CONTINUATION_MAX_PATH_M or first_width is None or second_width is None or
                abs(first_width - second_width) > OPEN_CONTINUATION_MAX_WIDTH_CHANGE_M):
            start = end + 1
            continue
        first_side, first_known = one_side_observation(cross_sections[first_id])
        second_side, second_known = one_side_observation(cross_sections[second_id])
        for sample_id in range(start, end + 1):
            alpha = float(sample_id - first_id) / float(second_id - first_id)
            width = first_width * (1.0 - alpha) + second_width * alpha
            sample = cross_sections[sample_id]['sample']
            if first_side == second_side:
                known = first_known * (1.0 - alpha) + second_known * alpha
                inferred = width - known
                if inferred < ONE_SIDE_MIN_INFERRED_HALF_WIDTH_M:
                    continue
                if first_side == 'left':
                    add_lateral_ribbon(sample, -inferred, known, ground, cells)
                else:
                    add_lateral_ribbon(sample, -known, inferred, ground, cells)
            else:
                # 两端看到的是不同侧边时，不能假设它们属于同一物理线；退化为沿轨迹居中的
                # 对称开阔面候选，仍保持低置信度，留待交叉口区域阶段确认。
                add_lateral_ribbon(sample, -width * 0.5, width * 0.5, ground, cells)
        records.append({'first_sample_id': first_id, 'last_sample_id': second_id,
                        'missing_samples': end - start + 1, 'path_length_m': distance,
                        'first_side': first_side, 'second_side': second_side,
                        'first_width_m': first_width, 'second_width_m': second_width})
        start = end + 1
    return cells, records


def default_width_corridor_candidates(cross_sections, bridge_records, ground):
    """沿已采集轨迹为无充分边界证据段生成默认宽度通行走廊假设。

    该层是用户明确允许的产品先验。它只覆盖轨迹附近已有稳定地面，不输出道路语义，
    且不覆盖现有的道路核心、单侧边或开放延续证据层。
    """
    bridged_ids = set()
    for record in bridge_records:
        bridged_ids.update(range(record['first_sample_id'] + 1, record['second_sample_id']))
    cells = {}
    records = defaultdict(int)
    for item in cross_sections:
        sample_id = item['sample_id']
        if item['road_valid'] or sample_id in bridged_ids:
            continue
        left_distance = item['left_inner'][0] if item['left_inner'][1] is not None else None
        right_distance = item['right_inner'][0] if item['right_inner'][1] is not None else None
        sample = item['sample']
        # 固定宽度层只沿轨迹中心线延续，不受每帧单侧墙边/绿化边的横向跳变牵引。
        # 边界存在状态仅作为后续分类的来源标签，不能改变该层的几何位置或带宽。
        add_lateral_ribbon(sample, -DEFAULT_CORRIDOR_WIDTH_M * 0.5,
                           DEFAULT_CORRIDOR_WIDTH_M * 0.5, ground, cells)
        if left_distance is not None and right_distance is None:
            records['one_side_default_width'] += 1
        elif right_distance is not None and left_distance is None:
            records['one_side_default_width'] += 1
        elif left_distance is not None and right_distance is not None:
            records['two_side_abnormal_default_width'] += 1
        else:
            records['no_boundary_default_width'] += 1
    return cells, records


def main():
    parser = argparse.ArgumentParser(description='阶段 3.2：长边界带约束道路/人行道候选')
    parser.add_argument('--stage02', required=True)
    parser.add_argument('--stage03a', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    stage02 = os.path.abspath(args.stage02)
    stage03a = os.path.abspath(args.stage03a)
    output = os.path.abspath(args.output)
    if not os.path.isdir(stage02):
        fail('阶段 2 目录不存在：' + stage02)
    if not os.path.isdir(stage03a):
        fail('阶段 3A R4 目录不存在：' + stage03a)
    if os.path.exists(output) and os.path.exists(os.path.join(output, 'stage032_complete.json')):
        fail('阶段 3.2 完整输出已存在，拒绝覆盖：' + output)
    if os.path.exists(output):
        print('[scene-stage032] 恢复未完成输出目录：' + output)

    stage02_evidence = os.path.join(stage02, 'evidence')
    stage03a_evidence = os.path.join(stage03a, 'evidence')
    ground = base.read_ground(os.path.join(stage02_evidence, 'geometric_observation_grid.csv'))
    trajectory = base.read_trajectory(os.path.join(stage02_evidence, 'map_trajectory_samples.csv'))
    bands = read_bands(os.path.join(stage03a_evidence, 'merged_boundary_bands.csv'))
    hierarchy = read_hierarchy(os.path.join(stage03a_evidence, 'trajectory_boundary_hierarchy.csv'))
    for band in bands:
        # 角色只作可审计的附加证据，不能作为过滤条件；否则灰色但真实的边界会再次消失。
        band['hierarchy_role'] = hierarchy.get(band['id'], {}).get('role', 'unrecorded_boundary_geometry')
    samples = base.resample_trajectory(trajectory, TRAJECTORY_SAMPLE_SPACING_M)

    road_cells = {}
    sidewalk_cells = {}
    centerline = []
    cross_sections = []
    selected_boundaries = []
    road_valid_count = 0
    sidewalk_valid_count = 0
    for sample_id, sample in enumerate(samples):
        left = band_candidates(sample, 1.0, bands)
        right = band_candidates(sample, -1.0, bands)
        left_inner = left[0] if left else (None, None)
        right_inner = right[0] if right else (None, None)
        road_valid = False
        if left_inner[1] is not None and right_inner[1] is not None:
            road_width = left_inner[0] + right_inner[0]
            road_valid = ROAD_MIN_WIDTH_M <= road_width <= ROAD_MAX_WIDTH_M
        else:
            road_width = None
        left_outer_distance, left_outer = (None, None)
        right_outer_distance, right_outer = (None, None)
        if road_valid:
            road_valid_count += 1
            add_lateral_ribbon(sample, -right_inner[0], left_inner[0], ground, road_cells)
            shift = (left_inner[0] - right_inner[0]) * 0.5
            center_x = sample['x'] + sample['nx'] * shift
            center_y = sample['y'] + sample['ny'] * shift
            centerline.append((center_x, center_y, closest_ground_z(ground, center_x, center_y)))
            left_outer_distance, left_outer = select_outer_band(left_inner[0], left)
            right_outer_distance, right_outer = select_outer_band(right_inner[0], right)
            if left_outer is not None:
                add_lateral_ribbon(sample, left_inner[0], left_outer_distance, ground, sidewalk_cells)
                sidewalk_valid_count += 1
            if right_outer is not None:
                add_lateral_ribbon(sample, -right_outer_distance, -right_inner[0], ground, sidewalk_cells)
                sidewalk_valid_count += 1
        for side_name, sign, item, kind in (
                ('left_inner', 1.0, left_inner, 'inner'),
                ('right_inner', -1.0, right_inner, 'inner'),
                ('left_outer', 1.0, (left_outer_distance, left_outer), 'outer'),
                ('right_outer', -1.0, (right_outer_distance, right_outer), 'outer')):
            distance, band = item
            if band is None:
                continue
            x = sample['x'] + sign * sample['nx'] * distance
            y = sample['y'] + sign * sample['ny'] * distance
            z = closest_ground_z(ground, x, y)
            if z is not None:
                selected_boundaries.append((x, y, z + 0.05,
                                            base.packed_rgb(255, 165, 45) if kind == 'inner'
                                            else base.packed_rgb(50, 205, 225)))
        def band_id(item):
            return '' if item[1] is None else item[1]['id']
        def band_distance(item):
            return '' if item[0] is None else '%.6f' % item[0]
        def band_role(item):
            return '' if item[1] is None else item[1]['hierarchy_role']
        cross_sections.append({
            'sample_id': sample_id, 'sample': sample, 'road_valid': road_valid,
            'road_width': road_width, 'left_inner': left_inner, 'right_inner': right_inner,
            'left_outer': (left_outer_distance, left_outer), 'right_outer': (right_outer_distance, right_outer),
            'band_id': band_id, 'band_distance': band_distance, 'band_role': band_role,
        })

    bridge_cells, bridge_cell_kinds, bridge_centerline, bridge_records = bridge_internal_road_gaps(
        samples, cross_sections, bands, ground)
    one_sided_cells, one_sided_records = one_sided_open_candidates(cross_sections, ground)
    continuation_cells, continuation_records = open_continuation_candidates(cross_sections, ground)
    default_corridor_cells, default_corridor_records = default_width_corridor_candidates(
        cross_sections, bridge_records, ground)
    all_road_cells = dict(bridge_cells)
    all_road_cells.update(road_cells)

    base.create_directory(os.path.join(output, 'evidence'))
    base.create_directory(os.path.join(output, 'validation'))
    write_cell_csv(os.path.join(output, 'evidence', 'road_core_candidate_cells.csv'), all_road_cells, ground,
                   '双边界直接约束或受限内部桥接的稳定地面道路核心候选，非道路语义结论')
    write_cell_csv(os.path.join(output, 'evidence', 'road_core_direct_candidate_cells.csv'), road_cells, ground,
                   '左右最近长边界带直接约束的稳定地面道路核心候选')
    write_cell_csv(os.path.join(output, 'evidence', 'road_core_bridged_candidate_cells.csv'), bridge_cells, ground,
                   '两端双边界锚点之间、受限内部缺口桥接的道路核心候选')
    write_cell_csv(os.path.join(output, 'evidence', 'one_sided_open_junction_candidate_cells.csv'), one_sided_cells,
                   ground, '连续单侧长边与邻近道路宽度参考生成的路口/开放通行面低置信度候选')
    write_cell_csv(os.path.join(output, 'evidence', 'open_area_continuation_candidate_cells.csv'), continuation_cells,
                   ground, '单侧边界证据岛之间无边界空档的沿轨迹开放通行面延续低置信度候选')
    write_cell_csv(os.path.join(output, 'evidence', 'default_width_corridor_candidate_cells.csv'), default_corridor_cells,
                   ground, '默认宽度产品先验生成的最低置信度通行走廊候选，非自动提取道路')
    write_cell_csv(os.path.join(output, 'evidence', 'sidewalk_between_boundary_candidate_cells.csv'), sidewalk_cells,
                   ground, '同侧内外长边界带之间的稳定地面人行道/路肩候选，非语义结论')
    with open(os.path.join(output, 'evidence', 'trajectory_boundary_cross_sections.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('sample_id', 'x', 'y', 'tangent_x', 'tangent_y', 'road_core_valid', 'road_width_m',
                         'left_inner_band_id', 'left_inner_distance_m', 'left_inner_hierarchy_role',
                         'right_inner_band_id', 'right_inner_distance_m', 'right_inner_hierarchy_role',
                         'left_outer_band_id', 'left_outer_distance_m', 'left_outer_hierarchy_role',
                         'right_outer_band_id', 'right_outer_distance_m', 'right_outer_hierarchy_role'))
        for item in cross_sections:
            sample = item['sample']
            writer.writerow((item['sample_id'], '%.6f' % sample['x'], '%.6f' % sample['y'],
                             '%.9f' % sample['tx'], '%.9f' % sample['ty'], int(item['road_valid']),
                             '' if item['road_width'] is None else '%.6f' % item['road_width'],
                             item['band_id'](item['left_inner']), item['band_distance'](item['left_inner']), item['band_role'](item['left_inner']),
                             item['band_id'](item['right_inner']), item['band_distance'](item['right_inner']), item['band_role'](item['right_inner']),
                             item['band_id'](item['left_outer']), item['band_distance'](item['left_outer']), item['band_role'](item['left_outer']),
                             item['band_id'](item['right_outer']), item['band_distance'](item['right_outer']), item['band_role'](item['right_outer'])))
    with open(os.path.join(output, 'evidence', 'road_core_bridge_records.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('first_valid_sample_id', 'second_valid_sample_id', 'bridged_missing_sample_count',
                         'path_length_m', 'bridge_kind', 'intersection_bridge', 'maximum_allowed_path_m',
                         'turn_angle_deg', 'left_boundary_change_m', 'right_boundary_change_m',
                         'left_visible_long_boundary_fraction', 'right_visible_long_boundary_fraction'))
        for item in bridge_records:
            writer.writerow((item['first_sample_id'], item['second_sample_id'], item['gap_sample_count'],
                             '%.6f' % item['path_length_m'], item['bridge_kind'], int(item['intersection_bridge']),
                             '%.6f' % item['maximum_path_m'], '%.6f' % item['turn_deg'],
                             '%.6f' % item['left_change_m'], '%.6f' % item['right_change_m'],
                             '%.6f' % item['left_visible_fraction'], '%.6f' % item['right_visible_fraction']))
    with open(os.path.join(output, 'evidence', 'one_sided_open_junction_records.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('first_sample_id', 'last_sample_id', 'observed_boundary_side',
                         'continuous_one_side_sample_count', 'accepted_reference_width_sample_count'))
        for item in one_sided_records:
            writer.writerow((item['first_sample_id'], item['last_sample_id'], item['side'],
                             item['continuous_samples'], item['accepted_samples']))
    with open(os.path.join(output, 'evidence', 'open_area_continuation_records.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('first_one_side_sample_id', 'last_one_side_sample_id', 'missing_no_boundary_sample_count',
                         'path_length_m', 'first_observed_side', 'last_observed_side',
                         'first_reference_width_m', 'last_reference_width_m'))
        for item in continuation_records:
            writer.writerow((item['first_sample_id'], item['last_sample_id'], item['missing_samples'],
                             '%.6f' % item['path_length_m'], item['first_side'], item['second_side'],
                             '%.6f' % item['first_width_m'], '%.6f' % item['second_width_m']))
    with open(os.path.join(output, 'evidence', 'default_width_corridor_records.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('source_evidence_kind', 'trajectory_cross_section_count', 'fixed_centerline_width_m', 'meaning'))
        meanings = {
            'one_side_default_width': '来源为单侧边界的固定中心线默认宽度通行走廊',
            'two_side_abnormal_default_width': '来源为双侧宽度异常的固定中心线默认宽度通行走廊',
            'no_boundary_default_width': '来源为无边界的固定中心线默认宽度通行走廊',
        }
        for kind in sorted(default_corridor_records):
            writer.writerow((kind, default_corridor_records[kind], '%.6f' % DEFAULT_CORRIDOR_WIDTH_M,
                             meanings[kind]))
    road_points = []
    for cell in all_road_cells:
        x, y = base.cell_center(cell)
        colors = {'regular_continuity': base.packed_rgb(155, 215, 65),
                  'turn_continuity': base.packed_rgb(220, 85, 225),
                  'long_corridor_evidence': base.packed_rgb(85, 215, 170)}
        color = base.packed_rgb(45, 175, 100) if cell in road_cells else colors[bridge_cell_kinds[cell]]
        road_points.append((x, y, ground[cell]['z'], color))
    bridge_points = []
    for cell in bridge_cells:
        if cell in road_cells:
            continue
        x, y = base.cell_center(cell)
        colors = {'regular_continuity': base.packed_rgb(155, 215, 65),
                  'turn_continuity': base.packed_rgb(220, 85, 225),
                  'long_corridor_evidence': base.packed_rgb(85, 215, 170)}
        bridge_points.append((x, y, ground[cell]['z'], colors[bridge_cell_kinds[cell]]))
    one_sided_points = []
    for cell in one_sided_cells:
        x, y = base.cell_center(cell)
        one_sided_points.append((x, y, ground[cell]['z'] + 0.01, base.packed_rgb(105, 105, 235)))
    continuation_points = []
    for cell in continuation_cells:
        x, y = base.cell_center(cell)
        continuation_points.append((x, y, ground[cell]['z'] + 0.02, base.packed_rgb(160, 105, 245)))
    default_corridor_points = []
    for cell in default_corridor_cells:
        x, y = base.cell_center(cell)
        default_corridor_points.append((x, y, ground[cell]['z'] + 0.03, base.packed_rgb(190, 190, 255)))
    sidewalk_points = []
    for cell in sidewalk_cells:
        x, y = base.cell_center(cell)
        sidewalk_points.append((x, y, ground[cell]['z'] + 0.015, base.packed_rgb(45, 205, 225)))
    centerline_points = []
    for x, y, z in centerline:
        if z is not None:
            centerline_points.append((x, y, z + 0.04, base.packed_rgb(55, 125, 230)))
    bridge_centerline_points = []
    for x, y, z in bridge_centerline:
        if z is not None:
            bridge_centerline_points.append((x, y, z + 0.045, base.packed_rgb(155, 215, 65)))
    junction_points = []
    junction_rows = []
    for node_id, item in enumerate(record for record in bridge_records if record['bridge_kind'] == 'turn_continuity'):
        middle_id = (item['first_sample_id'] + item['second_sample_id']) // 2
        sample = samples[middle_id]
        z = closest_ground_z(ground, sample['x'], sample['y'])
        if z is None:
            continue
        junction_points.append((sample['x'], sample['y'], z + 0.10, base.packed_rgb(235, 55, 160)))
        junction_rows.append((node_id, middle_id, sample['x'], sample['y'], z,
                              item['turn_deg'], item['path_length_m']))
    base.write_pcd(os.path.join(output, 'evidence', 'road_core_candidate_support.pcd'), road_points)
    base.write_pcd(os.path.join(output, 'evidence', 'road_core_bridged_candidate_support.pcd'), bridge_points)
    base.write_pcd(os.path.join(output, 'evidence', 'one_sided_open_junction_candidate_support.pcd'), one_sided_points)
    base.write_pcd(os.path.join(output, 'evidence', 'open_area_continuation_candidate_support.pcd'), continuation_points)
    base.write_pcd(os.path.join(output, 'evidence', 'default_width_corridor_candidate_support.pcd'), default_corridor_points)
    base.write_pcd(os.path.join(output, 'evidence', 'sidewalk_between_boundary_candidate_support.pcd'), sidewalk_points)
    base.write_pcd(os.path.join(output, 'evidence', 'road_centerline_candidate_support.pcd'), centerline_points)
    base.write_pcd(os.path.join(output, 'evidence', 'road_centerline_bridged_support.pcd'), bridge_centerline_points)
    base.write_pcd(os.path.join(output, 'evidence', 'junction_turn_node_candidate_support.pcd'), junction_points)
    base.write_pcd(os.path.join(output, 'evidence', 'selected_boundary_cross_section_support.pcd'), selected_boundaries)
    with open(os.path.join(output, 'evidence', 'junction_turn_node_candidates.csv'), 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('node_id', 'trajectory_sample_id', 'x', 'y', 'ground_z', 'turn_angle_deg', 'turn_path_length_m', 'meaning'))
        for item in junction_rows:
            writer.writerow((item[0], item[1], '%.6f' % item[2], '%.6f' % item[3], '%.6f' % item[4],
                             '%.6f' % item[5], '%.6f' % item[6],
                             '已通过转弯的三岔/十字路口候选节点，非完整路口面'))
    report = {
        'schema': 'fast_livo_scene_pipeline_stage032_boundary_constrained_corridors/v1',
        'purpose': '只在长边界带约束下生成道路核心与同侧内外带之间的候选，不做二维泛洪。',
        'source_stage02': stage02,
        'source_stage03a': stage03a,
        'stable_ground_cells': len(ground),
        'long_boundary_bands': len(bands),
        'trajectory_cross_sections': len(samples),
        'road_core_valid_cross_sections': road_valid_count,
        'sidewalk_valid_side_cross_sections': sidewalk_valid_count,
        'road_core_direct_candidate_cells': len(road_cells),
        'road_core_bridged_candidate_cells': len(bridge_cells),
        'road_core_candidate_cells': len(all_road_cells),
        'road_core_bridge_segments': len(bridge_records),
        'one_sided_open_junction_candidate_cells': len(one_sided_cells),
        'one_sided_open_junction_candidate_runs': len(one_sided_records),
        'open_area_continuation_candidate_cells': len(continuation_cells),
        'open_area_continuation_candidate_runs': len(continuation_records),
        'default_width_corridor_candidate_cells': len(default_corridor_cells),
        'default_width_corridor_cross_sections_by_evidence': dict(default_corridor_records),
        'junction_turn_node_candidates': len(junction_points),
        'sidewalk_candidate_cells': len(sidewalk_cells),
        'parameters': {
            'minimum_band_length_m': MIN_BAND_LENGTH_M, 'max_lateral_m': MAX_LATERAL_M,
            'orientation_deviation_deg': MAX_ORIENTATION_DEVIATION_DEG,
            'road_width_m': [ROAD_MIN_WIDTH_M, ROAD_MAX_WIDTH_M],
            'sidewalk_width_m': [SIDEWALK_MIN_WIDTH_M, SIDEWALK_MAX_WIDTH_M],
            'regular_bridge_max_path_m': REGULAR_BRIDGE_MAX_PATH_M,
            'turn_bridge_max_path_m': TURN_BRIDGE_MAX_PATH_M,
            'long_corridor_bridge_max_path_m': LONG_CORRIDOR_MAX_PATH_M,
        },
        'limits': [
            '灰色未关联边界仍保留在阶段 3A，阶段 3.2 不会凭空把其周围地面归入道路。',
            '内外带之间可能是人行道、路肩、绿化带或建筑退界，当前仅是几何候选。',
            '两侧边界任一侧缺失、线段长度不足或宽度不合理时，道路核心保持未知，除非该缺口被两端双边锚点受限桥接。',
            '常规桥接只允许两端双边距离连续的内部轨迹缺口，最多 20 m；起止端、宽度突变和超长未知段不桥接。',
            '转弯桥接仅限 40–135°、不超过 12 m 且缺口内至少一侧可见长边界；沿实际转向轨迹渐变大小路半宽。',
            '长走廊桥接仅限近似直行、不超过 40 m、端点宽度稳定且缺口内两侧仍有长边界支持。',
            '单侧开放面候选不属于道路核心；它只能用于后续路口/广场语义判定，不能直接渲染为道路。',
            '开放面延续候选只连接被单侧边界证据岛包夹、最长 15m 且宽度参考连续的无边界内部空档。',
            '默认宽度走廊是用户授权的产品先验，必须与道路核心和其它证据层分开，后续才能判为道路、路口或入口广场。',
            '本阶段不修改 SLAM、点云地图或阶段 0/1/2/3A 输入。'
        ]
    }
    with open(os.path.join(output, 'validation', 'stage032_boundary_constrained_corridors_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'stage032_complete.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '3.2_boundary_constrained_corridors',
                   'source_stage02': stage02, 'source_stage03a': stage03a,
                   'policy': '只读阶段 2 与阶段 3A R4 证据，不改写建图或既有输出。'},
                  handle, ensure_ascii=False, indent=2)
        handle.write('\n')
    print('[scene-stage032] 完成：长边界带={0}，有效道路横断面={1}/{2}，道路候选格={3}，单侧开放候选格={4}，开放延续候选格={5}，默认宽度候选格={6}，人行道候选格={7}，输出={8}'.format(
        len(bands), road_valid_count, len(samples), len(all_road_cells), len(one_sided_cells), len(continuation_cells), len(default_corridor_cells), len(sidewalk_cells), output))


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as error:
        print('[scene-stage032] ' + str(error), file=sys.stderr)
        sys.exit(1)
