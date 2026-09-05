#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 4.2：将低道路证据硬高差整理为“推断道路边界约束”。

本工具只读阶段 2、阶段 3A、阶段 3 最终交接和阶段 4 R7 复核层。它不创建道路面，
也不把紫色复核线升级为路沿。输出的 inferred_road_edge_constraint 只可在阶段 6 中作为
低置信度、需保留来源的矢量化约束；阶段 7 不得把它绘制成 observed 道路。
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
import stage04_curb_sidewalk_extraction as stage04


# 这些是“是否适合进入推断约束”的保守规则，而不是道路/路沿分类阈值。
MAX_CONSTRAINT_DIRECTION_ANGLE_DEG = 15.0
MAX_CONSTRAINT_DISTANCE_TO_TRAJECTORY_M = 5.0
PARALLEL_PAIR_MAX_ANGLE_DEG = 12.0
PARALLEL_PAIR_MIN_OVERLAP_FRACTION = 0.50
PARALLEL_PAIR_AMBIGUOUS_SEPARATION_M = 0.45
# 仅方向平行且投影重叠会把地图上相距很远、方向偶然相同的两条线误配成一组。
# 只有实际横向间距也足够近，才可以讨论“同一路段的内侧/外侧边”。
PARALLEL_PAIR_MAX_SEPARATION_M = 5.0
JUNCTION_CONTEXT_RADIUS_M = 6.0


def fail(message):
    raise RuntimeError(message)


def vector(record):
    dx = record['end_x'] - record['start_x']
    dy = record['end_y'] - record['start_y']
    length = math.hypot(dx, dy)
    if length == 0.0:
        return None
    return dx / length, dy / length, length


def direction_angle(first, second):
    first_vector, second_vector = vector(first), vector(second)
    if first_vector is None or second_vector is None:
        return 180.0
    dot = abs(first_vector[0] * second_vector[0] + first_vector[1] * second_vector[1])
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def parallel_overlap_and_separation(inner, outer):
    """计算同侧近平行段的投影重叠与横向分离，用于排除更外侧场景边。"""
    item = vector(inner)
    if item is None:
        return 0.0, None
    tx, ty, inner_length = item
    base_x, base_y = inner['start_x'], inner['start_y']
    inner_range = (0.0, inner_length)
    outer_projection = []
    for x, y in ((outer['start_x'], outer['start_y']), (outer['end_x'], outer['end_y'])):
        outer_projection.append((x - base_x) * tx + (y - base_y) * ty)
    outer_range = (min(outer_projection), max(outer_projection))
    overlap = max(0.0, min(inner_range[1], outer_range[1]) - max(inner_range[0], outer_range[0]))
    outer_length = vector(outer)[2]
    fraction = overlap / min(inner_length, outer_length) if min(inner_length, outer_length) else 0.0
    middle_x = 0.5 * (outer['start_x'] + outer['end_x'])
    middle_y = 0.5 * (outer['start_y'] + outer['end_y'])
    separation = abs((middle_x - base_x) * (-ty) + (middle_y - base_y) * tx)
    return fraction, separation


def point_segment_distance(point, record):
    ax, ay, bx, by = record['start_x'], record['start_y'], record['end_x'], record['end_y']
    dx, dy = bx - ax, by - ay
    denominator = dx * dx + dy * dy
    if denominator == 0.0:
        return math.hypot(point[0] - ax, point[1] - ay)
    ratio = max(0.0, min(1.0, ((point[0] - ax) * dx + (point[1] - ay) * dy) / denominator))
    return math.hypot(point[0] - (ax + ratio * dx), point[1] - (ay + ratio * dy))


def nearest_junction_distance(record, junction_points):
    if not junction_points:
        return None
    return min(point_segment_distance(point, record) for point in junction_points)


def review_key(record):
    return record['boundary_source'], str(record['band_id']), record['side']


def read_review_keys(path):
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('boundary_band_id', 'boundary_source', 'side')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段4低道路证据复核CSV字段不兼容：' + path)
        return set((row['boundary_source'], row['boundary_band_id'], row['side']) for row in reader)


def recompute_review_records(stage02, stage03a, stage03):
    """复现 R7 的局部段与筛选，以获得逐横断面来源和拟合几何。"""
    grid_path = os.path.join(stage02, 'evidence', 'geometric_observation_grid.csv')
    hard_path = os.path.join(stage02, 'evidence', 'continuous_hard_height_edge_support.pcd')
    bands_path = os.path.join(stage03a, 'evidence', 'merged_boundary_bands.csv')
    hierarchy_path = os.path.join(stage03a, 'evidence', 'trajectory_boundary_hierarchy.csv')
    cross_path = os.path.join(stage03, 'evidence', 'road_cross_section_confidence.csv')
    road_path = os.path.join(stage03, 'evidence', 'road_surface_candidate_complete_cells.csv')
    for path, title in ((grid_path, '阶段2几何网格'), (hard_path, '阶段2硬高差'),
                        (bands_path, '阶段3A边界带'), (hierarchy_path, '阶段3A层级'),
                        (cross_path, '阶段3横断面'), (road_path, '阶段3道路候选')):
        stage04.require_file(path, title)
    ground = stage04.read_grid(grid_path)
    bands = stage04.read_bands(bands_path)
    hierarchy = stage04.read_hierarchy(hierarchy_path)
    samples = stage04.read_cross_sections(cross_path)
    samples_by_id = dict((sample['id'], sample) for sample in samples)
    road_cells = stage04.read_road_cells(road_path)
    edge_index = stage04.PointIndex(stage04.pcd_points(hard_path))
    r4_records, _ = stage04.associate_boundaries(bands, samples, hierarchy, ground, road_cells, edge_index)
    local_records = stage04.extract_local_hard_edge_records(samples, samples_by_id, r4_records,
                                                            hierarchy, ground, road_cells, edge_index)
    return [record for record in local_records if stage04.qualifies_low_road_evidence_review(record)]


def write_records(path, records):
    header = ('boundary_band_id', 'boundary_source', 'side', 'start_x', 'start_y', 'end_x', 'end_y',
              'continuous_length_m', 'relative_road_distance_m', 'direction_angle_deg',
              'local_height_delta_m', 'hard_edge_support_fraction', 'road_support_fraction',
              'observed_road_support_fraction', 'topology_support_fraction', 'inside_ground_support_fraction',
              'candidate_status', 'evidence_state', 'constraint_confidence',
              'nearer_parallel_band_id', 'parallel_overlap_fraction', 'parallel_separation_m',
              'nearest_junction_distance_m', 'reason')
    with open(path, 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for item in sorted(records, key=lambda item: (item['candidate_status'], item['record']['side'],
                                                       item['record']['band_id'])):
            record = item['record']
            writer.writerow((record['band_id'], record['boundary_source'], record['side'],
                             '%.6f' % record['start_x'], '%.6f' % record['start_y'],
                             '%.6f' % record['end_x'], '%.6f' % record['end_y'],
                             '%.6f' % record['continuous_length_m'], '%.6f' % record['relative_road_distance_m'],
                             '%.6f' % record['direction_angle_deg'], '%.6f' % record['local_height_delta_m'],
                             '%.6f' % record['hard_edge_support_fraction'], '%.6f' % record['road_support_fraction'],
                             '%.6f' % record['observed_road_support_fraction'],
                             '%.6f' % record['topology_support_fraction'],
                             '%.6f' % record['inside_ground_support_fraction'], item['candidate_status'],
                             item['evidence_state'], '%.6f' % item['confidence'],
                             '' if item['nearer_band_id'] is None else item['nearer_band_id'],
                             '' if item['overlap_fraction'] is None else '%.6f' % item['overlap_fraction'],
                             '' if item['separation_m'] is None else '%.6f' % item['separation_m'],
                             '' if item['junction_distance_m'] is None else '%.6f' % item['junction_distance_m'],
                             '|'.join(item['reasons'])))


def main():
    parser = argparse.ArgumentParser(description='阶段4.2：低道路证据硬高差的推断道路边界约束')
    parser.add_argument('--stage02', required=True, help='冻结阶段2目录')
    parser.add_argument('--stage03a', required=True, help='冻结阶段3A目录')
    parser.add_argument('--stage03', required=True, help='冻结阶段3最终交接目录')
    parser.add_argument('--stage04', required=True, help='阶段4 R7低道路证据复核输出目录')
    parser.add_argument('--output', required=True, help='新的阶段4.2输出目录；必须不存在')
    arguments = parser.parse_args()
    stage02, stage03a, stage03, stage04_output = map(
        os.path.abspath, (arguments.stage02, arguments.stage03a, arguments.stage03, arguments.stage04))
    output = os.path.abspath(arguments.output)
    if os.path.exists(output):
        fail('阶段4.2输出目录已存在，拒绝覆盖：' + output)
    review_path = os.path.join(stage04_output, 'evidence', 'low_road_evidence_hard_edge_review_records.csv')
    junction_path = os.path.join(stage03, 'evidence', 'junction_turn_node_candidate_support.pcd')
    stage04.require_file(review_path, '阶段4 R7低道路证据复核CSV')
    stage04.require_file(junction_path, '阶段3路口候选PCD')

    review_records = recompute_review_records(stage02, stage03a, stage03)
    csv_keys = read_review_keys(review_path)
    computed_keys = set(review_key(record) for record in review_records)
    if csv_keys != computed_keys:
        fail('阶段4 R7复核记录与当前重算结果不一致；拒绝将不同版本混入阶段4.2')
    junction_points = stage04.pcd_points(junction_path)

    # 同侧且更靠近轨迹的平行段可以说明另一条线更像绿化/围墙等外侧场景边；
    # 若两线间距过小，则可能是同一粗硬高差的双线，双方都保持未知而非强行择一。
    relations = defaultdict(list)
    ambiguous = set()
    for outer in review_records:
        for inner in review_records:
            if outer is inner or outer['side'] != inner['side']:
                continue
            if inner['relative_road_distance_m'] >= outer['relative_road_distance_m']:
                continue
            if direction_angle(inner, outer) > PARALLEL_PAIR_MAX_ANGLE_DEG:
                continue
            overlap, separation = parallel_overlap_and_separation(inner, outer)
            if (overlap < PARALLEL_PAIR_MIN_OVERLAP_FRACTION or separation is None or
                    separation > PARALLEL_PAIR_MAX_SEPARATION_M):
                continue
            relations[outer['band_id']].append((inner, overlap, separation))
            if separation < PARALLEL_PAIR_AMBIGUOUS_SEPARATION_M:
                ambiguous.add(inner['band_id'])
                ambiguous.add(outer['band_id'])

    classified = []
    for record in review_records:
        reasons = []
        nearer = sorted(relations.get(record['band_id'], ()),
                        key=lambda item: (item[0]['relative_road_distance_m'], -item[1]))
        nearer_band_id, overlap, separation = (None, None, None)
        if nearer:
            selected = nearer[0]
            nearer_band_id, overlap, separation = selected[0]['band_id'], selected[1], selected[2]
        junction_distance = nearest_junction_distance(record, junction_points)
        status, state = 'inferred_road_edge_constraint', 'inferred'
        if record['band_id'] in ambiguous:
            status, state = 'parallel_ambiguous_context_review', 'unknown'
            reasons.append('与同侧更近的近平行线间距小于%.2fm，可能是同一粗硬高差双线' %
                           PARALLEL_PAIR_AMBIGUOUS_SEPARATION_M)
        elif nearer_band_id is not None:
            status, state = 'outer_context_edge', 'unknown'
            reasons.append('同侧存在更靠近轨迹且投影重叠的近平行段；本段保留为外侧场景边复核')
        elif record['relative_road_distance_m'] > MAX_CONSTRAINT_DISTANCE_TO_TRAJECTORY_M:
            status, state = 'far_context_edge', 'unknown'
            reasons.append('距轨迹%.2fm，超过推断道路边界约束上限%.2fm' %
                           (record['relative_road_distance_m'], MAX_CONSTRAINT_DISTANCE_TO_TRAJECTORY_M))
        elif record['direction_angle_deg'] > MAX_CONSTRAINT_DIRECTION_ANGLE_DEG:
            status, state = 'direction_context_review', 'unknown'
            reasons.append('与局部轨迹方向偏差%.2f度，超过%.2f度' %
                           (record['direction_angle_deg'], MAX_CONSTRAINT_DIRECTION_ANGLE_DEG))
        elif junction_distance is not None and junction_distance < JUNCTION_CONTEXT_RADIUS_M:
            status, state = 'junction_context_review', 'unknown'
            reasons.append('距阶段3转弯路口候选%.2fm，小于%.2fm；不得按直路延续' %
                           (junction_distance, JUNCTION_CONTEXT_RADIUS_M))
        else:
            reasons.append('连续硬高差可作为阶段6低置信度道路边界约束；不等同于观测道路边界')
        if record['topology_support_fraction'] < stage04.CURB_MIN_ROAD_SUPPORT:
            reasons.append('高/中证据中心线支持不足%.2f，阶段6必须保留低置信度' %
                           stage04.CURB_MIN_ROAD_SUPPORT)
        # 约束置信度上限 0.49，防止后续产品把它误用为观测道路证据。
        direction_score = max(0.0, 1.0 - record['direction_angle_deg'] / MAX_CONSTRAINT_DIRECTION_ANGLE_DEG)
        confidence = min(0.49, 0.12 * min(1.0, record['continuous_length_m'] / 10.0) +
                         0.16 * record['hard_edge_support_fraction'] +
                         0.16 * record['inside_ground_support_fraction'] +
                         0.16 * direction_score + 0.10 * record['topology_support_fraction'])
        if status != 'inferred_road_edge_constraint':
            confidence = 0.0
        classified.append({
            'record': record, 'candidate_status': status, 'evidence_state': state, 'confidence': confidence,
            'nearer_band_id': nearer_band_id, 'overlap_fraction': overlap, 'separation_m': separation,
            'junction_distance_m': junction_distance, 'reasons': reasons,
        })

    os.makedirs(os.path.join(output, 'evidence'))
    os.makedirs(os.path.join(output, 'validation'))
    evidence = os.path.join(output, 'evidence')
    write_records(os.path.join(evidence, 'inferred_road_edge_constraint_records.csv'), classified)
    colors = {
        'inferred_road_edge_constraint': base.packed_rgb(95, 205, 245),
        'outer_context_edge': base.packed_rgb(105, 190, 100),
        'parallel_ambiguous_context_review': base.packed_rgb(235, 185, 55),
        'far_context_edge': base.packed_rgb(160, 160, 160),
        'direction_context_review': base.packed_rgb(235, 145, 55),
        'junction_context_review': base.packed_rgb(225, 80, 160),
    }
    by_status = defaultdict(list)
    for item in classified:
        by_status[item['candidate_status']].extend(stage04.points_for_record(item['record'], colors[item['candidate_status']]))
    empty_layers = []
    stage04.write_pcd_or_mark_empty(os.path.join(evidence, 'inferred_road_edge_constraint_support.pcd'),
                                    by_status['inferred_road_edge_constraint'], empty_layers,
                                    '没有可作为阶段6推断道路边界约束的低道路证据硬高差')
    context_points = []
    for status, points in by_status.items():
        if status != 'inferred_road_edge_constraint':
            context_points.extend(points)
    stage04.write_pcd_or_mark_empty(os.path.join(evidence, 'road_edge_context_review_support.pcd'), context_points,
                                    empty_layers, '没有被排除或需要上下文复核的低道路证据硬高差')
    with open(os.path.join(evidence, 'road_edge_constraint_contract.json'), 'w') as handle:
        json.dump({
            'schema': 'fast_livo_scene_pipeline_stage042_inferred_road_edge_constraints/v1',
            'purpose': '把阶段4 R7低道路证据连续硬高差分为可供阶段6审查的推断道路边界约束和上下文复核边。',
            'source_stage04_review': stage04_output,
            'parameters': {
                'max_constraint_direction_angle_deg': MAX_CONSTRAINT_DIRECTION_ANGLE_DEG,
                'max_constraint_distance_to_trajectory_m': MAX_CONSTRAINT_DISTANCE_TO_TRAJECTORY_M,
                'parallel_pair_max_angle_deg': PARALLEL_PAIR_MAX_ANGLE_DEG,
                'parallel_pair_min_overlap_fraction': PARALLEL_PAIR_MIN_OVERLAP_FRACTION,
                'parallel_pair_ambiguous_separation_m': PARALLEL_PAIR_AMBIGUOUS_SEPARATION_M,
                'parallel_pair_max_separation_m': PARALLEL_PAIR_MAX_SEPARATION_M,
                'junction_context_radius_m': JUNCTION_CONTEXT_RADIUS_M,
                'constraint_confidence_cap': 0.49,
            },
            'consumption_policy': [
                'inferred_road_edge_constraint 只可作为阶段6低置信度道路延续的边界约束，必须保留 inferred 属性、来源与置信度。',
                '本输出不包含道路面、实测道路宽度或 curb_candidate；不得由阶段7直接渲染为 observed 道路。',
                'outer_context_edge、parallel_ambiguous_context_review、junction_context_review 等状态只可人工复核，不能用于道路补面。',
                '固定宽度道路不得因为本约束被回写、扩宽或升级为 observed。',
            ],
            'counts': dict((status, sum(1 for item in classified if item['candidate_status'] == status))
                           for status in sorted(set(item['candidate_status'] for item in classified))),
            'empty_pcd_layers': empty_layers,
        }, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    report = {
        'status': 'complete', 'stage': '04.2_inferred_road_edge_constraints',
        'source_stage02': stage02, 'source_stage03a': stage03a, 'source_stage03_final_handoff': stage03,
        'source_stage04_review': stage04_output,
        'review_record_identity_check': 'passed：R7复核CSV与当前阶段4重算记录键一致。',
        'total_review_records': len(classified),
        'status_counts': dict((status, sum(1 for item in classified if item['candidate_status'] == status))
                              for status in sorted(set(item['candidate_status'] for item in classified))),
        'limits': [
            '推断道路边界约束不是道路面或路沿。',
            '阶段6必须针对直路、路口和开放区域分别建模，不能把本层整体挤出一个道路面。',
            '阶段7只能按 inferred 低置信度策略消费阶段6审查通过的后续矢量，不能直接消费本阶段PCD。',
        ],
    }
    with open(os.path.join(output, 'validation', 'stage042_inferred_road_edge_constraints_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段4.2失败：{0}'.format(error), file=sys.stderr)
        sys.exit(1)
