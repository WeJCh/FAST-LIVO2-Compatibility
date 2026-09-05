#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段5 R2：面向导航标识的人工竖直结构精炼。

本脚本只读取阶段5 R1的真实稳定体素和对象审计表，不重读或改写原始点云。
它不生成完整建筑轮廓；输出的 building_marker_candidate 仍是附着于真实可见
竖直面上的导航标识候选。阶段4道路语境不参与本脚本的分类。
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict, Counter

from common.geometry import packed_rgb, read_binary_pcd, write_pcd


GRID_M = 4.0
SOURCE_TYPES = (
    'visible_facade_candidate', 'wall_candidate',
    'uncertain_vertical_structure', 'vegetation_or_nonplanar',
    'pole_or_trunk_candidate',
)


def fail(message):
    raise RuntimeError(message)


def number(row, name):
    value = row.get(name, '')
    if not value:
        return float('nan')
    return float(value)


def read_objects(path):
    """读取R1对象及其局部平面参数，保持R1对象ID作为可追溯来源。"""
    required = (
        'object_id', 'object_type', 'center_x', 'center_y', 'base_z', 'top_z',
        'tangent_x', 'tangent_y', 'normal_x', 'normal_y', 'visible_length_m',
        'plane_width_rms_m', 'visible_height_m', 'projection_coverage',
        'confidence', 'source_column_count', 'source_stable_voxel_count',
        'nearest_stable_ground_z',
    )
    result = []
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in required):
            fail('阶段5 R1对象审计字段不兼容：' + path)
        for row in reader:
            item = dict(row)
            item['id'] = int(row['object_id'])
            for name in required[2:]:
                item[name] = number(row, name)
            item['support_points'] = []
            result.append(item)
    if not result:
        fail('阶段5 R1对象审计表为空')
    return result


def object_padding(item):
    """R1拟合宽度较小时也保留半个体素余量，避免真实支持点被边界截掉。"""
    return max(0.45, min(1.50, 3.0 * item['plane_width_rms_m'] + 0.25))


def build_index(objects):
    """按来源类别分别建立二维索引，避免相邻不同类别对象相互抢占支持点。"""
    indices = dict((name, defaultdict(list)) for name in SOURCE_TYPES)
    for item in objects:
        if item['object_type'] not in indices:
            fail('R1存在未知对象类别：' + item['object_type'])
        half_length = item['visible_length_m'] / 2.0 + 0.45
        padding = object_padding(item)
        extent_x = abs(item['tangent_x']) * half_length + abs(item['normal_x']) * padding
        extent_y = abs(item['tangent_y']) * half_length + abs(item['normal_y']) * padding
        min_x = int(math.floor((item['center_x'] - extent_x) / GRID_M))
        max_x = int(math.floor((item['center_x'] + extent_x) / GRID_M))
        min_y = int(math.floor((item['center_y'] - extent_y) / GRID_M))
        max_y = int(math.floor((item['center_y'] + extent_y) / GRID_M))
        for ix in range(min_x, max_x + 1):
            for iy in range(min_y, max_y + 1):
                indices[item['object_type']][(ix, iy)].append(item)
    return indices


def assign_points(objects, indices, source_dir):
    """将R1真实体素回关联到其来源对象，并计算R2可解释特征。

    R1的可视PCD只有类别颜色、没有对象ID字段，因此这里按R1已记录的局部平面
    范围回关联。类别索引和最小归一化残差使相邻平行立面不会互相抢占多数点。
    """
    unmatched = Counter()
    # 非平面植被和杆体没有可靠的R1局部平面包围盒，后续直接继承其原始支持层。
    # 这里只对需要升级/拒绝的立面、墙面和不确定平面做精确回关联。
    for source_type in ('visible_facade_candidate', 'wall_candidate',
                        'uncertain_vertical_structure'):
        path = os.path.join(source_dir, source_type + '_support.pcd')
        if not os.path.isfile(path):
            fail('缺少R1来源支持层：' + path)
        for x, y, z, rgb in read_binary_pcd(path):
            candidates = indices[source_type].get((int(math.floor(x / GRID_M)), int(math.floor(y / GRID_M))), ())
            best = None
            for item in candidates:
                dx, dy = x - item['center_x'], y - item['center_y']
                along = dx * item['tangent_x'] + dy * item['tangent_y']
                cross = dx * item['normal_x'] + dy * item['normal_y']
                half_length = item['visible_length_m'] / 2.0 + 0.45
                padding = object_padding(item)
                if abs(along) > half_length or abs(cross) > padding:
                    continue
                if z < item['base_z'] - 0.45 or z > item['top_z'] + 0.45:
                    continue
                score = abs(cross) / padding + max(0.0, abs(along) - item['visible_length_m'] / 2.0) / 0.45
                if best is None or score < best[0]:
                    best = (score, item, along, cross)
            if best is None:
                unmatched[source_type] += 1
                continue
            _, item, along, cross = best
            item['support_points'].append((x, y, z, rgb, along, cross))
    return unmatched


def percentile(values, fraction):
    if not values:
        return float('nan')
    ordered = sorted(values)
    return ordered[int(round((len(ordered) - 1) * fraction))]


def enrich_features(item):
    points = item['support_points']
    if not points:
        item.update({
            'assigned_support_point_count': 0,
            'cross_p90_m': float('nan'), 'rgb_std_mean': float('nan'),
            'grid_fill_ratio': 0.0, 'base_above_ground_m': float('nan'),
        })
        return
    cross = [abs(point[5]) for point in points]
    colors = [[(point[3] >> shift) & 255 for point in points] for shift in (16, 8, 0)]
    color_std = []
    for channel in colors:
        mean = sum(channel) / float(len(channel))
        color_std.append(math.sqrt(sum((value - mean) ** 2 for value in channel) / float(len(channel))))
    along_bins = int(math.ceil(item['visible_length_m'] / 0.25)) + 1
    height_bins = int(math.ceil(item['visible_height_m'] / 0.25)) + 1
    occupied = set()
    for point in points:
        along_index = int(math.floor((point[4] + item['visible_length_m'] / 2.0) / 0.25))
        height_index = int(math.floor((point[2] - item['base_z']) / 0.25))
        occupied.add((along_index, height_index))
    ground_z = item['nearest_stable_ground_z']
    item.update({
        'assigned_support_point_count': len(points),
        'cross_p90_m': percentile(cross, 0.90),
        'rgb_std_mean': sum(color_std) / 3.0,
        'grid_fill_ratio': min(1.0, len(occupied) / float(max(1, along_bins * height_bins))),
        'base_above_ground_m': item['base_z'] - ground_z if math.isfinite(ground_z) else float('nan'),
    })


def vegetation_like(item):
    """只拒绝有多项一致证据的植物样候选，避免以颜色单独误杀灰墙或彩色建筑。"""
    if item['assigned_support_point_count'] == 0:
        return True, '没有可回关联的R1真实稳定体素'
    elevated_sparse = (math.isfinite(item['base_above_ground_m']) and
                        item['base_above_ground_m'] > 1.20 and item['grid_fill_ratio'] < 0.36)
    rough_sparse = (item['cross_p90_m'] > 0.29 and item['rgb_std_mean'] > 38.0 and
                    item['grid_fill_ratio'] < 0.20)
    if elevated_sparse:
        return True, '离稳定地面较高且垂直覆盖稀疏，符合树冠/遮挡结构特征'
    if rough_sparse:
        return True, '横向厚度、颜色离散度和稀疏覆盖共同指向非平面植被'
    return False, ''


def refine(item):
    """输出导航语义；建筑标识均来自真实支持点，不引入未观测的建筑深度。"""
    source = item['object_type']
    if source == 'vegetation_or_nonplanar':
        return 'vegetation_or_nonplanar', 'R1已判定为宽厚非平面垂直结构'
    if source == 'pole_or_trunk_candidate':
        return 'pole_or_trunk_context', 'R1局部水平跨度过小，不作为建筑标识'
    is_vegetation, reason = vegetation_like(item)
    if is_vegetation:
        return 'vegetation_like_rejected', reason

    # R1橙色立面直接保留，除非有植被样反证。
    if source == 'visible_facade_candidate':
        return 'building_marker_candidate', 'R1可见立面候选且未出现植被样反证'
    # R1黄色墙面中，足够高、长、平且连续者可作为导航建筑标识；几何仍保持 partial。
    if source == 'wall_candidate':
        if (item['visible_length_m'] >= 2.0 and item['visible_height_m'] >= 2.5 and
                item['plane_width_rms_m'] <= 0.22 and item['projection_coverage'] >= 0.60 and
                item['confidence'] >= 0.40):
            return 'building_marker_candidate', '高/长/窄/连续的R1墙面候选，提升为导航建筑标识候选'
        return 'vertical_structure_context', '墙面几何不足以稳定提升为建筑导航标识'
    # 放宽R1不确定层的语义门槛，以保留被窗洞、装饰或遮挡切碎的可能立面。
    if source == 'uncertain_vertical_structure':
        if (item['visible_length_m'] >= 1.8 and item['visible_height_m'] >= 2.5 and
                item['plane_width_rms_m'] <= 0.24 and item['projection_coverage'] >= 0.60 and
                item['confidence'] >= 0.25 and item['source_column_count'] >= 6):
            return 'building_marker_possible', '不确定竖直面满足放宽后的导航标识几何条件'
        return 'vertical_structure_context', '不确定竖直面未满足放宽后的导航标识条件'
    fail('未处理的R1类别：' + source)


def write_records(path, objects):
    fields = (
        'r2_object_id', 'r1_object_id', 'r1_object_type', 'r2_navigation_class',
        'geometry_evidence_state', 'semantic_evidence_state', 'completeness',
        'center_x', 'center_y', 'base_z', 'top_z', 'tangent_x', 'tangent_y',
        'normal_x', 'normal_y', 'visible_length_m', 'plane_width_rms_m',
        'visible_height_m', 'projection_coverage', 'r1_confidence',
        'assigned_support_point_count', 'cross_p90_m', 'rgb_std_mean',
        'grid_fill_ratio', 'base_above_ground_m', 'decision_reason',
    )
    with open(path, 'w') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for r2_id, item in enumerate(objects):
            row = {
                'r2_object_id': r2_id, 'r1_object_id': item['id'],
                'r1_object_type': item['object_type'], 'r2_navigation_class': item['r2_class'],
                'geometry_evidence_state': 'observed',
                'semantic_evidence_state': 'inferred' if item['r2_class'].startswith('building_marker') else 'unknown',
                'completeness': 'partial',
                'decision_reason': item['decision_reason'],
            }
            for key in fields:
                if key in item:
                    row[key] = item[key]
            writer.writerow(row)


def write_class_pcds(evidence_dir, objects, source_dir):
    colors = {
        'building_marker_candidate': packed_rgb(238, 122, 45),
        'building_marker_possible': packed_rgb(255, 181, 74),
        'vegetation_like_rejected': packed_rgb(69, 177, 89),
        'vegetation_or_nonplanar': packed_rgb(69, 177, 89),
        'pole_or_trunk_context': packed_rgb(54, 183, 198),
        'vertical_structure_context': packed_rgb(150, 150, 160),
    }
    grouped = defaultdict(list)
    for item in objects:
        for x, y, z, _rgb, _along, _cross in item['support_points']:
            grouped[item['r2_class']].append((x, y, z, colors[item['r2_class']]))
    # 这两层不以平面拟合为前提，直接复用R1真实稳定体素并按R2类别重新着色。
    direct_layers = (
        ('vegetation_or_nonplanar', 'vegetation_or_nonplanar'),
        ('pole_or_trunk_candidate', 'pole_or_trunk_context'),
    )
    for source_type, target_class in direct_layers:
        path = os.path.join(source_dir, source_type + '_support.pcd')
        for x, y, z, _rgb in read_binary_pcd(path):
            grouped[target_class].append((x, y, z, colors[target_class]))
    for name, color in colors.items():
        # 没有点的层不写空PCD，以免CloudCompare等读取器报错。
        points = grouped.get(name, [])
        if points:
            write_pcd(os.path.join(evidence_dir, name + '_support.pcd'), points)
    return dict((name, len(points)) for name, points in grouped.items())


def write_report(path, objects, unmatched, layer_counts):
    classes = Counter(item['r2_class'] for item in objects)
    report = {
        'schema': 'fast_livo_scene_pipeline_stage05_navigation_structure_refinement/v1',
        'status': 'complete',
        'purpose': '将阶段5 R1的真实局部竖直面精炼为导航建筑标识候选与植被/上下文层；不补完整建筑。',
        'r1_object_count': len(objects),
        'r2_class_counts': dict(classes),
        'r2_support_point_counts': layer_counts,
        'unmatched_r1_support_points_by_source_class': dict(unmatched),
        'rules': {
            'building_marker_candidate': 'R1立面候选，或满足高/长/窄/连续条件的墙面候选；均须无植被样反证。',
            'building_marker_possible': '满足放宽条件的不确定竖直面，主要用于保留可能受窗洞或遮挡影响的可见建筑。',
            'vegetation_like_rejected': '以离地稀疏或厚度+颜色离散度+稀疏覆盖的组合反证拒绝，颜色单独不构成拒绝。',
        },
        'limits': [
            'R2没有完整建筑轮廓、建筑深度或不可见侧面的证据；所有对象仍为partial。',
            '无人工标注时，局部平整绿篱与真实墙面仍可能重叠，R2只降低而不能宣称消除全部误判。',
            '阶段6只能将building_marker_candidate/possible作为带inferred语义的导航代理来源，不能当作完整建筑底图。',
        ],
    }
    with open(path, 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')


def main():
    parser = argparse.ArgumentParser(description='阶段5 R2导航结构精炼')
    parser.add_argument('--stage05-r1', required=True, help='阶段5 R1输出目录')
    parser.add_argument('--output', required=True, help='新的R2输出目录，必须不存在')
    arguments = parser.parse_args()
    stage05_r1 = os.path.abspath(arguments.stage05_r1)
    output = os.path.abspath(arguments.output)
    if os.path.exists(output):
        fail('R2输出目录已存在，拒绝覆盖：' + output)
    records = os.path.join(stage05_r1, 'evidence', 'visible_scene_object_records.csv')
    source_dir = os.path.join(stage05_r1, 'evidence')
    if not os.path.isfile(records):
        fail('缺少阶段5 R1对象审计表：' + records)
    objects = read_objects(records)
    indices = build_index(objects)
    unmatched = assign_points(objects, indices, source_dir)
    for item in objects:
        enrich_features(item)
        item['r2_class'], item['decision_reason'] = refine(item)
    os.makedirs(os.path.join(output, 'evidence'))
    os.makedirs(os.path.join(output, 'validation'))
    write_records(os.path.join(output, 'evidence', 'navigation_structure_records.csv'), objects)
    layer_counts = write_class_pcds(os.path.join(output, 'evidence'), objects, source_dir)
    write_report(os.path.join(output, 'validation', 'stage05_navigation_structure_refinement_report.json'), objects, unmatched, layer_counts)
    with open(os.path.join(output, 'stage05_r2_complete.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '05_navigation_structure_refinement',
                   'source_stage05_r1': stage05_r1,
                   'records': 'evidence/navigation_structure_records.csv'}, handle,
                  ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    print('阶段5 R2完成：对象=%d，类别=%s' % (len(objects), dict(Counter(item['r2_class'] for item in objects))))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段5 R2失败：%s' % error, file=sys.stderr)
        sys.exit(1)
