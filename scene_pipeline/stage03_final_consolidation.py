#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 3 收束：输出供阶段 4 使用的道路/开放面分层证据交接包。

本脚本不再改变道路候选几何，也不重新做泛洪、边界拟合或默认宽度推断。它只把已经
验证过的阶段 2、阶段 3A 和阶段 3.2 R12 输出整理为可审计的最终契约：

* 高置信度：同一横断面双边实测；
* 中置信度：双边锚点受限连续桥接，或实测单侧长边；
* 推断连续性：无边界/宽度异常处、但仍具轨迹与稳定地面支持的固定宽度延续；
* 开放面候选：单侧边界、入口广场和其它尚未完成类别判定的区域；

完整道路候选面包含双边实测、受限桥接与固定宽度延续，以便阶段 4 不会因证据缺口
中断搜索范围。但每个栅格保留 ``observed``、``width_source`` 和多来源标记；固定宽度
延续只能作为阶段 4 的搜索工作面，绝不能伪装为实测道路宽度或实测边界。
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys

from common import geometry as base


def fail(message):
    raise RuntimeError(message)


def read_cross_sections(path):
    result = []
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('sample_id', 'x', 'y', 'tangent_x', 'tangent_y', 'road_core_valid', 'road_width_m',
                'left_inner_band_id', 'left_inner_distance_m', 'right_inner_band_id', 'right_inner_distance_m')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段 3.2 横断面 CSV 字段不兼容：' + path)
        for row in reader:
            def number(name):
                return None if not row[name] else float(row[name])
            result.append({
                'id': int(row['sample_id']), 'x': float(row['x']), 'y': float(row['y']),
                'tx': float(row['tangent_x']), 'ty': float(row['tangent_y']),
                'valid': row['road_core_valid'] == '1', 'width': number('road_width_m'),
                'left_id': None if not row['left_inner_band_id'] else int(row['left_inner_band_id']),
                'left': number('left_inner_distance_m'),
                'right_id': None if not row['right_inner_band_id'] else int(row['right_inner_band_id']),
                'right': number('right_inner_distance_m'),
            })
    if not result:
        fail('阶段 3.2 横断面 CSV 为空：' + path)
    return result


def read_bridge_records(path):
    result = []
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('first_valid_sample_id', 'second_valid_sample_id', 'bridge_kind')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('道路桥接 CSV 字段不兼容：' + path)
        for row in reader:
            result.append({'first': int(row['first_valid_sample_id']),
                           'second': int(row['second_valid_sample_id']),
                           'kind': row['bridge_kind']})
    return result


def read_binary_layer(path):
    if not os.path.isfile(path):
        fail('缺少阶段 3.2 证据 PCD：' + path)
    return base.read_binary_pcd(path)


def read_cell_layer(path, color):
    """将阶段 3.2 的审计 CSV 恢复成单一证据层 PCD。"""
    points = []
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('x', 'y', 'ground_z')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('候选栅格 CSV 字段不兼容：' + path)
        for row in reader:
            points.append((float(row['x']), float(row['y']), float(row['ground_z']), color))
    return points


def read_candidate_cells(path, source_class, observed, width_source, confidence):
    """读取栅格候选层，并保留供阶段 4 审计的来源属性。"""
    result = []
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('cell_ix', 'cell_iy', 'x', 'y', 'ground_z', 'cross_section_support_count')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('道路候选栅格 CSV 字段不兼容：' + path)
        for row in reader:
            result.append({
                'cell': (int(row['cell_ix']), int(row['cell_iy'])),
                'x': float(row['x']), 'y': float(row['y']), 'z': float(row['ground_z']),
                'support': int(row['cross_section_support_count']), 'source_class': source_class,
                'observed': observed, 'width_source': width_source, 'confidence': confidence,
            })
    return result


def write_complete_road_candidate(path_csv, path_pcd, observed_cells, bridge_cells, fallback_cells):
    """输出连续道路联合面，并记录一个栅格全部参与过的候选来源。"""
    source_rank = {
        'fixed_width_fallback': 1,
        'two_sided_continuity_bridge': 2,
        'two_sided_measured': 3,
    }
    combined = {}
    for records in (fallback_cells, bridge_cells, observed_cells):
        for item in records:
            entry = combined.setdefault(item['cell'], {'items': []})
            entry['items'].append(item)
    # 单个 PCD 点只能有一种颜色，因此显示最强来源；CSV 同时保留所有来源，避免旧版
    # “互斥分区”隐藏道路边界/桥接证据的问题。
    for entry in combined.values():
        entry['primary'] = max(entry['items'], key=lambda item: source_rank[item['source_class']])
    colors = {
        'two_sided_measured': base.packed_rgb(45, 175, 100),
        'two_sided_continuity_bridge': base.packed_rgb(155, 215, 65),
        'fixed_width_fallback': base.packed_rgb(190, 190, 255),
    }
    with open(path_csv, 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('cell_ix', 'cell_iy', 'x', 'y', 'ground_z', 'cross_section_support_count',
                         'road_candidate_primary_source', 'observed', 'width_source',
                         'road_membership_confidence', 'has_two_sided_measured',
                         'has_two_sided_continuity_bridge', 'has_fixed_width_fallback',
                         'all_candidate_sources', 'meaning'))
        meanings = {
            'two_sided_measured': '双边实测道路核心；可作为真实道路宽度与边界规则化证据',
            'two_sided_continuity_bridge': '双边锚点间的连续道路候选；宽度为插值，非实测',
            'fixed_width_fallback': '轨迹与稳定地面支持的固定宽度道路延续；宽度为产品先验，非实测',
        }
        for cell in sorted(combined):
            entry = combined[cell]
            item = entry['primary']
            source_classes = set(candidate['source_class'] for candidate in entry['items'])
            writer.writerow((cell[0], cell[1], '%.6f' % item['x'], '%.6f' % item['y'],
                             '%.6f' % item['z'], item['support'], item['source_class'],
                             int(item['observed']), item['width_source'], item['confidence'],
                             int('two_sided_measured' in source_classes),
                             int('two_sided_continuity_bridge' in source_classes),
                             int('fixed_width_fallback' in source_classes),
                             '|'.join(sorted(source_classes)), meanings[item['source_class']]))
    points = []
    for entry in combined.values():
        item = entry['primary']
        points.append((item['x'], item['y'], item['z'] + 0.01, colors[item['source_class']]))
    base.write_pcd(path_pcd, points)
    return combined


def closest_ground_z(ground, x, y):
    return base.closest_ground_z(ground, x, y)


def classify_cross_sections(cross_sections, bridge_records):
    """给每个横断面生成唯一置信度与可否用于阶段 4 的明确标记。"""
    bridge_info = {}
    for record in bridge_records:
        first, second = record['first'], record['second']
        if first < 0 or second >= len(cross_sections) or second <= first:
            continue
        left_first, right_first = cross_sections[first]['left'], cross_sections[first]['right']
        left_second, right_second = cross_sections[second]['left'], cross_sections[second]['right']
        if None in (left_first, right_first, left_second, right_second):
            continue
        for sample_id in range(first + 1, second):
            alpha = float(sample_id - first) / float(second - first)
            bridge_info[sample_id] = {
                'kind': record['kind'],
                'left': left_first * (1.0 - alpha) + left_second * alpha,
                'right': right_first * (1.0 - alpha) + right_second * alpha,
            }
    rows = []
    for item in cross_sections:
        sample_id = item['id']
        has_left = item['left_id'] is not None
        has_right = item['right_id'] is not None
        row = dict(item)
        row['bridge_kind'] = ''
        row['true_width'] = False
        row['use_road_surface_candidate'] = True
        row['use_complete_candidate_centerline'] = True
        if item['valid'] and item['width'] is not None:
            row.update({'confidence': 'high', 'class': 'two_sided_measured_road_core',
                        'width_for_stage4': item['width'], 'true_width': True,
                        'use_centerline_stage4': True})
        elif sample_id in bridge_info:
            bridge = bridge_info[sample_id]
            row.update({'confidence': 'medium', 'class': 'two_sided_continuity_bridge',
                        'width_for_stage4': bridge['left'] + bridge['right'], 'true_width': False,
                        'use_centerline_stage4': True, 'bridge_kind': bridge['kind'],
                        'left': bridge['left'], 'right': bridge['right']})
        elif has_left != has_right:
            row.update({'confidence': 'medium', 'class': 'one_sided_measured_boundary',
                        'width_for_stage4': None, 'use_centerline_stage4': True})
        elif has_left and has_right:
            row.update({'confidence': 'low', 'class': 'two_sided_abnormal_width_open_candidate',
                        'width_for_stage4': None, 'use_centerline_stage4': False})
        else:
            row.update({'confidence': 'low', 'class': 'no_boundary_default_width_candidate',
                        'width_for_stage4': None, 'use_centerline_stage4': False})
        rows.append(row)
    return rows


def write_cross_section_csv(path, rows):
    with open(path, 'w') as handle:
        writer = csv.writer(handle)
        writer.writerow(('sample_id', 'x', 'y', 'tangent_x', 'tangent_y', 'confidence', 'candidate_class',
                         'use_for_stage4_road_surface_candidate', 'use_for_stage4_centerline',
                         'use_for_stage4_complete_candidate_centerline',
                         'local_width_m', 'is_measured_true_width', 'bridge_kind',
                         'left_boundary_distance_m', 'right_boundary_distance_m', 'meaning'))
        meanings = {
            'two_sided_measured_road_core': '双边实测道路核心；可用于阶段4道路宽度与中心线规则化',
            'two_sided_continuity_bridge': '两端双边锚点间的受限桥接；可用于中心线连续性，宽度非实测',
            'one_sided_measured_boundary': '仅一侧实测长边；可用于中心线方向，不能作为真实道路宽度',
            'two_sided_abnormal_width_open_candidate': '双边存在但宽度异常；仅作为路口/广场开放面候选',
            'no_boundary_default_width_candidate': '无边界默认宽度候选；仅作为最低置信度开放通行面',
        }
        for item in rows:
            writer.writerow((item['id'], '%.6f' % item['x'], '%.6f' % item['y'],
                             '%.9f' % item['tx'], '%.9f' % item['ty'], item['confidence'], item['class'],
                             int(item['use_road_surface_candidate']), int(item['use_centerline_stage4']),
                             int(item['use_complete_candidate_centerline']),
                             '' if item['width_for_stage4'] is None else '%.6f' % item['width_for_stage4'],
                             int(item['true_width']), item['bridge_kind'],
                             '' if item['left'] is None else '%.6f' % item['left'],
                             '' if item['right'] is None else '%.6f' % item['right'], meanings[item['class']]))


def write_centerline(path, rows, ground):
    points = []
    colors = {'high': base.packed_rgb(55, 125, 230), 'medium_bridge': base.packed_rgb(45, 205, 185),
              'medium_single': base.packed_rgb(70, 175, 225)}
    for item in rows:
        if not item['use_centerline_stage4']:
            continue
        normal_x, normal_y = -item['ty'], item['tx']
        if item['class'] in ('two_sided_measured_road_core', 'two_sided_continuity_bridge'):
            center_x = item['x'] + normal_x * (item['left'] - item['right']) * 0.5
            center_y = item['y'] + normal_y * (item['left'] - item['right']) * 0.5
            color = colors['high'] if item['confidence'] == 'high' else colors['medium_bridge']
        else:
            # 单侧边界不声称可恢复真实中线，仅将采集轨迹作为阶段 4 的方向支撑。
            center_x, center_y, color = item['x'], item['y'], colors['medium_single']
        z = closest_ground_z(ground, center_x, center_y)
        if z is not None:
            points.append((center_x, center_y, z + 0.05, color))
    base.write_pcd(path, points)
    return len(points)


def write_complete_candidate_centerline(path, rows, ground):
    """输出贯通阶段 4 搜索范围的中心线候选，不等同于最终道路拓扑。"""
    points = []
    colors = {
        'high': base.packed_rgb(55, 125, 230),
        'medium': base.packed_rgb(45, 205, 185),
        'inferred': base.packed_rgb(190, 190, 255),
    }
    for item in rows:
        if not item['use_complete_candidate_centerline']:
            continue
        normal_x, normal_y = -item['ty'], item['tx']
        if item['class'] in ('two_sided_measured_road_core', 'two_sided_continuity_bridge'):
            center_x = item['x'] + normal_x * (item['left'] - item['right']) * 0.5
            center_y = item['y'] + normal_y * (item['left'] - item['right']) * 0.5
        else:
            # 没有可靠双侧宽度时，固定宽度延续严格以采集轨迹为中心，不随单侧杂边摆动。
            center_x, center_y = item['x'], item['y']
        z = closest_ground_z(ground, center_x, center_y)
        if z is not None:
            color = colors['high'] if item['confidence'] == 'high' else (
                colors['medium'] if item['confidence'] == 'medium' else colors['inferred'])
            points.append((center_x, center_y, z + 0.06, color))
    base.write_pcd(path, points)
    return len(points)


def main():
    parser = argparse.ArgumentParser(description='阶段 3 收束：道路/开放面证据交接包')
    parser.add_argument('--stage02', required=True)
    parser.add_argument('--stage03a', required=True)
    parser.add_argument('--stage032', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--overwrite', action='store_true',
                        help='允许覆盖同一阶段 3 交接包；只写入本脚本管理的文件。')
    args = parser.parse_args()
    stage02 = os.path.abspath(args.stage02)
    stage03a = os.path.abspath(args.stage03a)
    stage032 = os.path.abspath(args.stage032)
    output = os.path.abspath(args.output)
    for label, path in (('阶段 2', stage02), ('阶段 3A', stage03a), ('阶段 3.2', stage032)):
        if not os.path.isdir(path):
            fail(label + ' 目录不存在：' + path)
    if os.path.exists(output) and os.path.exists(os.path.join(output, 'stage03_complete.json')) and not args.overwrite:
        fail('阶段 3 最终交接包已存在，拒绝覆盖：' + output)
    if os.path.exists(output):
        action = '覆盖既有交接包' if args.overwrite else '恢复未完成输出目录'
        print('[scene-stage03-final] {0}：{1}'.format(action, output))
    stage02_evidence = os.path.join(stage02, 'evidence')
    source = os.path.join(stage032, 'evidence')
    ground = base.read_ground(os.path.join(stage02_evidence, 'geometric_observation_grid.csv'))
    cross_sections = read_cross_sections(os.path.join(source, 'trajectory_boundary_cross_sections.csv'))
    bridge_records = read_bridge_records(os.path.join(source, 'road_core_bridge_records.csv'))
    rows = classify_cross_sections(cross_sections, bridge_records)
    layers = {
        'road_core_medium_continuity_support.pcd': 'road_core_bridged_candidate_support.pcd',
        'sidewalk_or_shoulder_candidate_support.pcd': 'sidewalk_between_boundary_candidate_support.pcd',
        'one_sided_open_candidate_support.pcd': 'one_sided_open_junction_candidate_support.pcd',
        'open_area_continuation_candidate_support.pcd': 'open_area_continuation_candidate_support.pcd',
        'default_width_low_confidence_support.pcd': 'default_width_corridor_candidate_support.pcd',
        'junction_turn_node_candidate_support.pcd': 'junction_turn_node_candidate_support.pcd',
    }
    base.create_directory(os.path.join(output, 'evidence'))
    base.create_directory(os.path.join(output, 'validation'))
    layer_counts = {}
    high_points = read_cell_layer(os.path.join(source, 'road_core_direct_candidate_cells.csv'),
                                  base.packed_rgb(45, 175, 100))
    base.write_pcd(os.path.join(output, 'evidence', 'road_core_high_confidence_support.pcd'), high_points)
    layer_counts['road_core_high_confidence_support.pcd'] = len(high_points)
    for destination, origin in layers.items():
        points = read_binary_layer(os.path.join(source, origin))
        base.write_pcd(os.path.join(output, 'evidence', destination), points)
        layer_counts[destination] = len(points)
    observed_cells = read_candidate_cells(
        os.path.join(source, 'road_core_direct_candidate_cells.csv'), 'two_sided_measured', True, 'measured', 'high')
    bridge_cells = read_candidate_cells(
        os.path.join(source, 'road_core_bridged_candidate_cells.csv'), 'two_sided_continuity_bridge', False,
        'interpolated_from_two_sided_anchors', 'medium')
    fallback_cells = read_candidate_cells(
        os.path.join(source, 'default_width_corridor_candidate_cells.csv'), 'fixed_width_fallback', False,
        'fixed_width_fallback', 'inferred')
    complete_road = write_complete_road_candidate(
        os.path.join(output, 'evidence', 'road_surface_candidate_complete_cells.csv'),
        os.path.join(output, 'evidence', 'road_surface_candidate_complete_support.pcd'),
        observed_cells, bridge_cells, fallback_cells)
    layer_counts['road_surface_candidate_complete_support.pcd'] = len(complete_road)
    write_cross_section_csv(os.path.join(output, 'evidence', 'road_cross_section_confidence.csv'), rows)
    centerline_count = write_centerline(os.path.join(output, 'evidence', 'centerline_high_medium_support.pcd'), rows, ground)
    complete_centerline_count = write_complete_candidate_centerline(
        os.path.join(output, 'evidence', 'centerline_candidate_complete_support.pcd'), rows, ground)
    confidence_counts = {}
    class_counts = {}
    for item in rows:
        confidence_counts[item['confidence']] = confidence_counts.get(item['confidence'], 0) + 1
        class_counts[item['class']] = class_counts.get(item['class'], 0) + 1
    report = {
        'schema': 'fast_livo_scene_pipeline_stage03_final_handoff/v1',
        'purpose': '阶段4交接：分层道路、开放面、中心线、宽度置信度与路口候选；不改写任何前置阶段。',
        'source_stage02': stage02,
        'source_stage03a': stage03a,
        'source_stage032': stage032,
        'cross_section_confidence_counts': confidence_counts,
        'cross_section_class_counts': class_counts,
        'high_medium_centerline_points': centerline_count,
        'complete_candidate_centerline_points': complete_centerline_count,
        'complete_road_candidate_cells': len(complete_road),
        'layer_point_counts': layer_counts,
        'stage4_policy': [
            'road_surface_candidate_complete 包含 observed 与 inferred 两类道路候选，是阶段4搜索路沿与人行道的连续工作面。',
            'centerline_high_medium_support 仅含 high/medium，可作为后续道路拓扑确认支撑。',
            'centerline_candidate_complete_support 包含固定宽度延续，仅供阶段4连续搜索，不能直接作为最终道路中心线。',
            '仅 two_sided_measured_road_core 的 local_width_m 可被解释为实测道路宽度。',
            '连续桥接宽度仅用于几何连续性；单侧边界不输出真实道路宽度。',
            '完整道路候选中的 fixed_width_fallback 是轨迹、稳定地面与自由空间共同支持的推断道路成员，可用于阶段4边界搜索和连续性判定；不能用于真实道路宽度规则化或事实渲染。',
            '开放延续、宽度异常双边和单侧开放面必须在阶段4判为道路、路口、入口广场或其它开放面后才可渲染。',
        ],
    }
    with open(os.path.join(output, 'validation', 'stage03_final_handoff_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'stage03_complete.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '3_final_consolidation',
                   'source_stage02': stage02, 'source_stage03a': stage03a, 'source_stage032': stage032,
                   'policy': '只读前置证据，阶段4必须遵守 road_cross_section_confidence.csv 的置信度约束。'},
                  handle, ensure_ascii=False, indent=2)
        handle.write('\n')
    print('[scene-stage03-final] 完成：横断面 high/medium/low={0}/{1}/{2}，阶段4中心线点={3}，输出={4}'.format(
        confidence_counts.get('high', 0), confidence_counts.get('medium', 0),
        confidence_counts.get('low', 0), centerline_count, output))


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as error:
        print('[scene-stage03-final] ' + str(error), file=sys.stderr)
        sys.exit(1)
