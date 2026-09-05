#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 4.1：定点追溯局部硬高差边界为何未成为蓝线或路沿。

这是只读诊断工具：读取冻结的阶段 2、阶段 3 最终交接包，以及一个既有阶段 4
版本的 CSV 结果。它不会改写任何阶段产物，也不会重新把缺失路沿补成事实。

诊断分成四层：
1. 用户给出的坐标是否实际命中阶段 2 原始硬高差点；
2. 该点能否进入阶段 4 局部横断面候选与跨断面轨迹；
3. 该轨迹附近生成了哪些阶段 4 记录；
4. 哪个分类/蓝色审查层门槛拒绝了它。
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys

from common import geometry as base
import stage04_curb_sidewalk_extraction as stage04


TARGET_RECORD_RADIUS_M = 1.0
TARGET_TRACK_RADIUS_M = 0.75


def fail(message):
    raise RuntimeError(message)


def parse_point(value):
    """解析 label,x,y,z；z 只用于审计输入是否对应同一个原始点。"""
    parts = value.split(',')
    if len(parts) != 4:
        raise argparse.ArgumentTypeError('点位格式应为 label,x,y,z：' + value)
    try:
        return {'label': parts[0], 'x': float(parts[1]), 'y': float(parts[2]), 'z': float(parts[3])}
    except ValueError:
        raise argparse.ArgumentTypeError('点位坐标必须是数字：' + value)


def distance_xy(first, second):
    return math.hypot(first[0] - second[0], first[1] - second[1])


def record_distance(record, x, y):
    """点到审计记录的有限线段距离，避免延长线把远处记录误算为命中。"""
    ax, ay, bx, by = record['start_x'], record['start_y'], record['end_x'], record['end_y']
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom == 0.0:
        return math.hypot(x - ax, y - ay)
    ratio = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / denom))
    return math.hypot(x - (ax + ratio * dx), y - (ay + ratio * dy))


def track_span(track):
    if len(track) < 2:
        return 0.0
    return distance_xy(track[0]['point'], track[-1]['point'])


def write_csv(path, fieldnames, rows):
    with open(path, 'w') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def record_brief(record, distance):
    """将阶段4记录转成稳定 CSV/JSON 字段，拒绝原因保持原文。"""
    return {
        'distance_to_target_m': distance,
        'boundary_band_id': record['band_id'],
        'boundary_source': record['boundary_source'],
        'side': record['side'],
        'semantic_class': record['semantic_class'],
        'evidence_state': record['evidence_state'],
        'associated_length_m': record['associated_length_m'],
        'continuous_length_m': record['continuous_length_m'],
        'relative_road_distance_m': record['relative_road_distance_m'],
        'local_height_delta_m': record['local_height_delta_m'],
        'hard_edge_support_fraction': record['hard_edge_support_fraction'],
        'road_support_fraction': record['road_support_fraction'],
        'observed_road_support_fraction': record['observed_road_support_fraction'],
        'topology_support_fraction': record['topology_support_fraction'],
        'inside_ground_support_fraction': record['inside_ground_support_fraction'],
        'outside_ground_support_fraction': record['outside_ground_support_fraction'],
        'inner_boundary_match_fraction': record['inner_boundary_match_fraction'],
        'local_road_edge_match_fraction': record['local_road_edge_match_fraction'],
        'rejection_reason': record['rejection_reason'],
    }


def blue_layer_eligible(record):
    """与 R5 主程序写蓝色 local_hard_edge PCD 的条件完全一致。"""
    height = record['local_height_delta_m']
    return (record['road_support_fraction'] >= stage04.CURB_MIN_ROAD_SUPPORT and
            record['inside_ground_support_fraction'] >= stage04.CURB_MIN_GROUND_SUPPORT and
            height is not None and abs(height) >= stage04.CURB_MIN_HEIGHT_M)


def road_cells_nearby(road_cells, x, y, radius_cells=2):
    """复现阶段4道路支持的 5×5 栅格检索，并保留来源以便解释“为什么是零”。"""
    cell_x, cell_y = base.grid_key(x, y)
    result = []
    for offset_x in range(-radius_cells, radius_cells + 1):
        for offset_y in range(-radius_cells, radius_cells + 1):
            item = road_cells.get((cell_x + offset_x, cell_y + offset_y))
            if item is not None:
                result.append(item)
    return result


def main():
    parser = argparse.ArgumentParser(description='阶段4.1局部硬高差定点诊断（只读）')
    parser.add_argument('--stage02', required=True, help='冻结阶段2目录')
    parser.add_argument('--stage03a', required=True, help='冻结阶段3A目录')
    parser.add_argument('--stage03', required=True, help='冻结阶段3最终交接目录')
    parser.add_argument('--stage04', required=True, help='待审计的既有阶段4输出目录，例如R5')
    parser.add_argument('--output', required=True, help='新的诊断输出目录；必须不存在')
    parser.add_argument('--point', action='append', type=parse_point, required=True,
                        help='重复提供：label,x,y,z')
    arguments = parser.parse_args()

    stage02, stage03a, stage03, stage04_output = map(
        os.path.abspath, (arguments.stage02, arguments.stage03a, arguments.stage03, arguments.stage04))
    output = os.path.abspath(arguments.output)
    if os.path.exists(output):
        fail('诊断输出目录已存在，拒绝覆盖：' + output)

    grid_path = os.path.join(stage02, 'evidence', 'geometric_observation_grid.csv')
    hard_path = os.path.join(stage02, 'evidence', 'continuous_hard_height_edge_support.pcd')
    band_path = os.path.join(stage03a, 'evidence', 'merged_boundary_bands.csv')
    hierarchy_path = os.path.join(stage03a, 'evidence', 'trajectory_boundary_hierarchy.csv')
    cross_path = os.path.join(stage03, 'evidence', 'road_cross_section_confidence.csv')
    road_path = os.path.join(stage03, 'evidence', 'road_surface_candidate_complete_cells.csv')
    stage04_records_path = os.path.join(stage04_output, 'evidence', 'combined_boundary_semantic_records.csv')
    for path, title in ((grid_path, '阶段2几何网格'), (hard_path, '阶段2硬高差'),
                        (band_path, '阶段3A边界带'), (hierarchy_path, '阶段3A层级'),
                        (cross_path, '阶段3横断面'), (road_path, '阶段3道路候选'),
                        (stage04_records_path, '阶段4语义记录')):
        stage04.require_file(path, title)

    # 这里重新计算局部轨迹，是为了显示在生成 R5 CSV 前每一条候选在哪一道门槛停止。
    ground = stage04.read_grid(grid_path)
    bands = stage04.read_bands(band_path)
    hierarchy = stage04.read_hierarchy(hierarchy_path)
    samples = stage04.read_cross_sections(cross_path)
    samples_by_id = dict((sample['id'], sample) for sample in samples)
    road_cells = stage04.read_road_cells(road_path)
    hard_points = stage04.pcd_points(hard_path)
    edge_index = stage04.PointIndex(hard_points)
    r4_records, _ = stage04.associate_boundaries(bands, samples, hierarchy, ground, road_cells, edge_index)
    local_records = stage04.extract_local_hard_edge_records(samples, samples_by_id, r4_records,
                                                            hierarchy, ground, road_cells, edge_index)
    tracks = stage04.local_edge_tracks(samples, edge_index)

    # 既有阶段4 CSV 必须和按当前 R5 参数重算的记录一致；否则不能把诊断误说成
    # 某个旧版本的追溯结果。
    with open(stage04_records_path, 'r') as handle:
        existing_rows = list(csv.DictReader(handle))
    recomputed_records = r4_records + local_records
    existing_keys = set((row['boundary_source'], int(row['boundary_band_id']), row['side'])
                        for row in existing_rows)
    recomputed_keys = set((record['boundary_source'], record['band_id'], record['side'])
                          for record in recomputed_records)
    if existing_keys != recomputed_keys:
        fail('既有阶段4记录与当前R5重算结果的记录键不一致；拒绝把它作为同一版本诊断')
    existing_by_key = dict(((row['boundary_source'], int(row['boundary_band_id']), row['side']), row)
                           for row in existing_rows)
    for record in recomputed_records:
        key = (record['boundary_source'], record['band_id'], record['side'])
        row = existing_by_key[key]
        if (row['semantic_class'] != record['semantic_class'] or
                row['evidence_state'] != record['evidence_state'] or
                row['rejection_reason'] != record['rejection_reason']):
            fail('既有阶段4记录与当前R5重算结果的语义/拒绝原因不一致：%s' % (key,))
    existing_record_count = len(existing_rows)

    target_rows, track_rows, record_rows, road_probe_rows, markers, summaries = [], [], [], [], [], []
    for target in arguments.point:
        x, y, z = target['x'], target['y'], target['z']
        nearest_raw = min(hard_points, key=lambda point: math.hypot(point[0] - x, point[1] - y))
        raw_distance = distance_xy(nearest_raw, (x, y))
        slices = []
        for sample in samples:
            if not sample['use_complete']:
                continue
            dx, dy = x - sample['x'], y - sample['y']
            longitudinal = dx * sample['tx'] + dy * sample['ty']
            lateral = dx * sample['nx'] + dy * sample['ny']
            if (abs(longitudinal) <= stage04.LOCAL_EDGE_SLICE_HALF_LENGTH_M and
                    0.10 <= abs(lateral) <= stage04.MAX_BOUNDARY_LATERAL_M):
                slices.append({'sample': sample, 'longitudinal': longitudinal, 'lateral': lateral})

        nearby_tracks = []
        for track_id, track in enumerate(tracks):
            minimum = min(distance_xy(item['point'], (x, y)) for item in track)
            if minimum <= TARGET_TRACK_RADIUS_M:
                nearby_tracks.append((minimum, track_id, track))
                track_rows.append({
                    'target_label': target['label'], 'track_id': track_id,
                    'nearest_candidate_distance_m': '%.6f' % minimum,
                    'candidate_count': len(track), 'start_sample_id': track[0]['sample_id'],
                    'end_sample_id': track[-1]['sample_id'], 'endpoint_span_m': '%.6f' % track_span(track),
                    'passes_min_samples': int(len(track) >= stage04.LOCAL_EDGE_MIN_SAMPLES),
                    'passes_min_span': int(track_span(track) >= stage04.MIN_BOUNDARY_CONTINUITY_M),
                    'sample_id_gaps': '|'.join(str(track[index]['sample_id'] - track[index - 1]['sample_id'])
                                               for index in range(1, len(track))),
                })

        nearby_records = []
        for record in recomputed_records:
            distance = record_distance(record, x, y)
            if distance <= TARGET_RECORD_RADIUS_M:
                brief = record_brief(record, distance)
                brief['target_label'] = target['label']
                brief['blue_layer_eligible'] = int(record['boundary_source'] == 'stage02_hard_edge_local_fit' and
                                                   blue_layer_eligible(record))
                nearby_records.append(brief)
                record_rows.append(brief)
                if record['boundary_source'] == 'stage02_hard_edge_local_fit':
                    for link in record['links']:
                        sample = samples_by_id[link['sample_id']]
                        edge_x, edge_y = link['point']
                        # 与 classify_band_association 相同：从边界朝道路内侧偏移 0.25m，
                        # 再在半径 2 个 0.2m 栅格内寻找阶段3完整道路候选。
                        inset_x = edge_x - record['side_sign'] * sample['nx'] * 0.25
                        inset_y = edge_y - record['side_sign'] * sample['ny'] * 0.25
                        cells = road_cells_nearby(road_cells, inset_x, inset_y)
                        road_probe_rows.append({
                            'target_label': target['label'], 'boundary_band_id': record['band_id'],
                            'sample_id': sample['id'], 'edge_x': '%.6f' % edge_x,
                            'edge_y': '%.6f' % edge_y, 'edge_lateral_m': '%.6f' % link['lateral'],
                            'inset_x': '%.6f' % inset_x, 'inset_y': '%.6f' % inset_y,
                            'has_complete_road_candidate': int(bool(cells)),
                            'has_observed_road_candidate': int(any(item['observed'] for item in cells)),
                            'nearby_candidate_cell_count': len(cells),
                            'nearby_candidate_sources': '|'.join(sorted(set(item['source'] for item in cells))),
                            'cross_section_confidence': sample['confidence'],
                            'cross_section_class': sample['candidate_class'],
                            'uses_high_medium_centerline': int(sample['use_centerline']),
                        })

        nearest_sample = min(samples, key=lambda sample: math.hypot(sample['x'] - x, sample['y'] - y))
        nearest_sample_dx, nearest_sample_dy = x - nearest_sample['x'], y - nearest_sample['y']
        nearest_longitudinal = (nearest_sample_dx * nearest_sample['tx'] +
                                nearest_sample_dy * nearest_sample['ty'])
        nearest_lateral = (nearest_sample_dx * nearest_sample['nx'] +
                           nearest_sample_dy * nearest_sample['ny'])
        blue_records = [item for item in nearby_records if item['blue_layer_eligible'] == 1]
        local_nearby = [item for item in nearby_records
                        if item['boundary_source'] == 'stage02_hard_edge_local_fit']
        exact_raw = raw_distance <= 0.001
        conclusion = []
        if exact_raw:
            conclusion.append('阶段2原始硬高差点精确命中')
        else:
            conclusion.append('阶段2原始硬高差点未精确命中，最近距离 %.3fm' % raw_distance)
        if nearby_tracks:
            conclusion.append('局部跨断面追踪已覆盖该位置')
        else:
            conclusion.append('未进入局部跨断面追踪；应先检查横断面切片和跟踪阈值')
        if local_nearby and not blue_records:
            conclusion.append('附近局部记录未写入蓝色审查层：不满足道路支持/内侧地面/高度三项显示门槛')
        elif blue_records:
            conclusion.append('附近存在满足蓝色审查层显示门槛的局部记录')
        else:
            conclusion.append('附近没有形成可定位的局部语义记录')
        if local_nearby and all(item['road_support_fraction'] < stage04.CURB_MIN_ROAD_SUPPORT for item in local_nearby):
            conclusion.append('直接限制因素是道路完整搜索面支持低于 %.2f，而不是局部跟踪阈值' %
                              stage04.CURB_MIN_ROAD_SUPPORT)

        summaries.append({
            'target': target, 'nearest_raw_edge': {'x': nearest_raw[0], 'y': nearest_raw[1], 'z': nearest_raw[2],
                                                    'xy_distance_m': raw_distance,
                                                    'z_delta_m': nearest_raw[2] - z},
            'nearest_cross_section': {
                'sample_id': nearest_sample['id'],
                'xy_distance_m': math.hypot(nearest_sample_dx, nearest_sample_dy),
                'longitudinal_m': nearest_longitudinal, 'lateral_m': nearest_lateral,
                'confidence': nearest_sample['confidence'], 'candidate_class': nearest_sample['candidate_class'],
                'use_complete': nearest_sample['use_complete'], 'use_centerline': nearest_sample['use_centerline'],
                'use_road': nearest_sample['use_road'],
            },
            'eligible_local_slices': [{'sample_id': item['sample']['id'],
                                       'longitudinal_m': item['longitudinal'], 'lateral_m': item['lateral'],
                                       'confidence': item['sample']['confidence'],
                                       'candidate_class': item['sample']['candidate_class']}
                                      for item in slices],
            'nearby_track_count': len(nearby_tracks),
            'nearby_stage04_record_count': len(nearby_records),
            'nearby_blue_layer_record_count': len(blue_records),
            'conclusion': conclusion,
        })
        target_rows.append({
            'target_label': target['label'], 'target_x': '%.6f' % x, 'target_y': '%.6f' % y,
            'target_z': '%.6f' % z, 'nearest_raw_x': '%.6f' % nearest_raw[0],
            'nearest_raw_y': '%.6f' % nearest_raw[1], 'nearest_raw_z': '%.6f' % nearest_raw[2],
            'raw_xy_distance_m': '%.6f' % raw_distance, 'raw_z_delta_m': '%.6f' % (nearest_raw[2] - z),
            'eligible_local_slice_count': len(slices), 'nearby_track_count': len(nearby_tracks),
            'nearby_stage04_record_count': len(nearby_records), 'nearby_blue_layer_record_count': len(blue_records),
            'conclusion': '|'.join(conclusion),
        })
        # 三种颜色只用于 CloudCompare 复核：红=用户点，黄=最近原始硬高差点，蓝=最近横断面。
        markers.extend(((x, y, z + 0.03, base.packed_rgb(240, 70, 70)),
                        (nearest_raw[0], nearest_raw[1], nearest_raw[2] + 0.05, base.packed_rgb(245, 210, 40)),
                        (nearest_sample['x'], nearest_sample['y'], z + 0.07, base.packed_rgb(80, 155, 245))))

    os.makedirs(os.path.join(output, 'evidence'))
    os.makedirs(os.path.join(output, 'validation'))
    evidence = os.path.join(output, 'evidence')
    write_csv(os.path.join(evidence, 'target_diagnosis.csv'), list(target_rows[0].keys()), target_rows)
    track_fields = ('target_label', 'track_id', 'nearest_candidate_distance_m', 'candidate_count',
                    'start_sample_id', 'end_sample_id', 'endpoint_span_m', 'passes_min_samples',
                    'passes_min_span', 'sample_id_gaps')
    write_csv(os.path.join(evidence, 'nearby_local_tracks.csv'), track_fields, track_rows)
    record_fields = ('target_label', 'distance_to_target_m', 'boundary_band_id', 'boundary_source', 'side',
                     'semantic_class', 'evidence_state', 'associated_length_m', 'continuous_length_m',
                     'relative_road_distance_m', 'local_height_delta_m', 'hard_edge_support_fraction',
                     'road_support_fraction', 'observed_road_support_fraction', 'topology_support_fraction',
                     'inside_ground_support_fraction', 'outside_ground_support_fraction',
                     'inner_boundary_match_fraction', 'local_road_edge_match_fraction',
                     'rejection_reason', 'blue_layer_eligible')
    write_csv(os.path.join(evidence, 'nearby_stage04_records.csv'), record_fields, record_rows)
    probe_fields = ('target_label', 'boundary_band_id', 'sample_id', 'edge_x', 'edge_y', 'edge_lateral_m',
                    'inset_x', 'inset_y', 'has_complete_road_candidate', 'has_observed_road_candidate',
                    'nearby_candidate_cell_count', 'nearby_candidate_sources', 'cross_section_confidence',
                    'cross_section_class', 'uses_high_medium_centerline')
    write_csv(os.path.join(evidence, 'road_support_probe.csv'), probe_fields, road_probe_rows)
    base.write_pcd(os.path.join(evidence, 'target_markers.pcd'), markers)
    report = {
        'schema': 'fast_livo_scene_pipeline_stage041_local_hard_edge_point_diagnosis/v1',
        'policy': '只读冻结阶段2/3/3A与既有阶段4；诊断结果不得反向修改道路或路沿事实。',
        'source_stage02': stage02, 'source_stage03a': stage03a, 'source_stage03_final_handoff': stage03,
        'source_stage04': stage04_output, 'existing_stage04_record_count': existing_record_count,
        'recomputed_stage04_record_count': len(recomputed_records),
        'stage04_record_identity_check': 'passed：记录键、语义分类、证据状态和拒绝原因与R5重算一致。',
        'road_support_probe': '逐局部边界横断面复现阶段4的内侧0.25m偏移和2格邻域道路检索；CSV保留来源。',
        'parameters_reproduced_from_stage04_r5': {
            'local_edge_slice_half_length_m': stage04.LOCAL_EDGE_SLICE_HALF_LENGTH_M,
            'local_edge_cluster_gap_m': stage04.LOCAL_EDGE_CLUSTER_GAP_M,
            'local_edge_track_lateral_gap_m': stage04.LOCAL_EDGE_TRACK_LATERAL_GAP_M,
            'local_edge_max_sample_gap': stage04.LOCAL_EDGE_MAX_SAMPLE_GAP,
            'local_edge_min_samples': stage04.LOCAL_EDGE_MIN_SAMPLES,
            'local_edge_min_span_m': stage04.MIN_BOUNDARY_CONTINUITY_M,
            'blue_layer_min_road_support': stage04.CURB_MIN_ROAD_SUPPORT,
            'blue_layer_min_inside_ground_support': stage04.CURB_MIN_GROUND_SUPPORT,
            'blue_layer_min_abs_height_m': stage04.CURB_MIN_HEIGHT_M,
        },
        'targets': summaries,
        'marker_pcd_legend': '红=输入坐标；黄=最近阶段2硬高差点；蓝=最近阶段3横断面（仅复核标记，非路沿语义）。',
    }
    with open(os.path.join(output, 'validation', 'stage041_point_diagnosis_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段4.1定点诊断失败：{0}'.format(error), file=sys.stderr)
        sys.exit(1)
