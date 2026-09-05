#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段5 R3：道路上方反证与非平面植被层中的局部墙面回收。

R3仍只输出真实稳定体素。道路实测面只作为“不能是建筑标识”的反证，
绝不按道路距离补建筑；从绿色层回收的结构只标为 possible，且必须同时
满足局部窄平面、垂直连续、稳定地面邻接和非道路上方条件。
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys
from collections import Counter, defaultdict

import stage05_navigation_structure_refinement as r2
from common.geometry import packed_rgb, read_binary_pcd, write_pcd


ROAD_GRID_M = 0.20
GREEN_GRID_M = 0.25
TILE_M = 10.0


def fail(message):
    raise RuntimeError(message)


def read_observed_road(path):
    """只读取双边实测高置信道路格，桥接/固定宽度道路不能用于排除建筑。"""
    result = {}
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('cell_ix', 'cell_iy', 'ground_z', 'observed',
                'road_candidate_primary_source', 'road_membership_confidence')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段3道路格字段不兼容：' + path)
        for row in reader:
            if not (row['observed'] == '1' and
                    row['road_candidate_primary_source'] == 'two_sided_measured' and
                    row['road_membership_confidence'] == 'high'):
                continue
            result[(int(row['cell_ix']), int(row['cell_iy']))] = float(row['ground_z'])
    if not result:
        fail('阶段3中没有可用于R3反证的双边实测道路格')
    return result


def read_stable_ground(path):
    result = {}
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('cell_ix', 'cell_iy', 'ground_mean_z', 'stable_ground', 'free_frame_count')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段2几何网格字段不兼容：' + path)
        for row in reader:
            if row['stable_ground'] == '1' and int(row['free_frame_count']) >= 1:
                result[(int(row['cell_ix']), int(row['cell_iy']))] = float(row['ground_mean_z'])
    if not result:
        fail('阶段2中没有稳定地面')
    return result


def road_metrics(points, road):
    """返回支持点位于实测道路平面上方的比例；仅作排除反证。"""
    road_points = 0
    above_points = 0
    for x, y, z, _rgb, _along, _cross in points:
        z_road = road.get((int(math.floor(x / ROAD_GRID_M)), int(math.floor(y / ROAD_GRID_M))))
        if z_road is None:
            continue
        road_points += 1
        if z > z_road + 0.60:
            above_points += 1
    total = float(max(1, len(points)))
    return road_points / total, above_points / float(max(1, road_points))


def road_overhead_or_dynamic(points, road):
    overlap, above = road_metrics(points, road)
    return overlap >= 0.55 and above >= 0.60, overlap, above


def nearest_ground(ground, x, y, radius_m=1.0):
    base_x = int(math.floor(x / ROAD_GRID_M))
    base_y = int(math.floor(y / ROAD_GRID_M))
    radius = int(math.ceil(radius_m / ROAD_GRID_M))
    best = None
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            z = ground.get((base_x + dx, base_y + dy))
            if z is None:
                continue
            gx = (base_x + dx + 0.5) * ROAD_GRID_M
            gy = (base_y + dy + 0.5) * ROAD_GRID_M
            distance = math.hypot(gx - x, gy - y)
            if distance <= radius_m and (best is None or distance < best[0]):
                best = (distance, z)
    return best


def green_columns(points):
    """把R1绿色真实体素按0.25m平面列累计，不产生任何新点。"""
    columns = {}
    for index, point in enumerate(points):
        key = (int(math.floor(point[0] / GREEN_GRID_M)),
               int(math.floor(point[1] / GREEN_GRID_M)))
        item = columns.get(key)
        if item is None:
            item = {'indices': [], 'min_z': point[2], 'max_z': point[2], 'z_bins': set()}
            columns[key] = item
        item['indices'].append(index)
        item['min_z'] = min(item['min_z'], point[2])
        item['max_z'] = max(item['max_z'], point[2])
        item['z_bins'].add(int(math.floor(point[2] / GREEN_GRID_M)))
    return columns


def fit_line(keys):
    if not keys:
        return None
    mean_x = sum((key[0] + 0.5) * GREEN_GRID_M for key in keys) / float(len(keys))
    mean_y = sum((key[1] + 0.5) * GREEN_GRID_M for key in keys) / float(len(keys))
    xx = xy = yy = 0.0
    for key in keys:
        dx = (key[0] + 0.5) * GREEN_GRID_M - mean_x
        dy = (key[1] + 0.5) * GREEN_GRID_M - mean_y
        xx += dx * dx
        xy += dx * dy
        yy += dy * dy
    angle = 0.5 * math.atan2(2.0 * xy, xx - yy)
    tx, ty = math.cos(angle), math.sin(angle)
    projections, residuals, bins = [], [], set()
    for key in keys:
        dx = (key[0] + 0.5) * GREEN_GRID_M - mean_x
        dy = (key[1] + 0.5) * GREEN_GRID_M - mean_y
        projection = dx * tx + dy * ty
        residual = -dx * ty + dy * tx
        projections.append(projection)
        residuals.append(residual)
        bins.add(int(math.floor(projection / GREEN_GRID_M)))
    length = max(projections) - min(projections) + GREEN_GRID_M
    width = math.sqrt(sum(value * value for value in residuals) / float(len(residuals)))
    coverage = min(1.0, len(bins) / float(max(1, int(math.ceil(length / GREEN_GRID_M)))))
    return {'x': mean_x, 'y': mean_y, 'tx': tx, 'ty': ty,
            'length': length, 'width': width, 'coverage': coverage}


def recover_green_planes(points, ground, road):
    """在绿色非平面层内保守搜索局部窄平面，返回恢复点索引与审计记录。"""
    columns = green_columns(points)
    candidates = [key for key, item in columns.items()
                  if item['max_z'] - item['min_z'] >= 2.2 and len(item['z_bins']) >= 5]
    tile_cells = int(round(TILE_M / GREEN_GRID_M))
    tiles = defaultdict(list)
    for key in candidates:
        tiles[(int(math.floor(key[0] / float(tile_cells))),
               int(math.floor(key[1] / float(tile_cells))))].append(key)
    selected_columns = set()
    records = []
    for tile_key in sorted(tiles):
        remaining = list(tiles[tile_key])
        # 极大块通常是完整树冠；不在其中强行寻找立面，避免用偶然叶片线制造建筑。
        if len(remaining) < 12 or len(remaining) > 900:
            continue
        state = (tile_key[0] * 73856093 + tile_key[1] * 19349663) & 0xffffffff
        for _pass in range(3):
            if len(remaining) < 12:
                break
            best = None
            best_score = 0.0
            iterations = min(120, max(48, len(remaining) * 2))
            for _ in range(iterations):
                state = (1664525 * state + 1013904223) & 0xffffffff
                a = remaining[(state >> 8) % len(remaining)]
                state = (1664525 * state + 1013904223) & 0xffffffff
                b = remaining[(state >> 8) % len(remaining)]
                ax, ay = (a[0] + 0.5) * GREEN_GRID_M, (a[1] + 0.5) * GREEN_GRID_M
                bx, by = (b[0] + 0.5) * GREEN_GRID_M, (b[1] + 0.5) * GREEN_GRID_M
                dx, dy = bx - ax, by - ay
                norm = math.hypot(dx, dy)
                if norm < 0.8:
                    continue
                nx, ny = -dy / norm, dx / norm
                inliers = []
                for key in remaining:
                    px = (key[0] + 0.5) * GREEN_GRID_M - ax
                    py = (key[1] + 0.5) * GREEN_GRID_M - ay
                    if abs(px * nx + py * ny) <= 0.24:
                        inliers.append(key)
                if len(inliers) < 12:
                    continue
                fit = fit_line(inliers)
                if fit['length'] < 3.0 or fit['width'] > 0.22 or fit['coverage'] < 0.65:
                    continue
                score = len(inliers) * fit['coverage'] * (1.0 - fit['width'] / 0.25)
                if score > best_score:
                    best, best_score = (inliers, fit), score
            if best is None:
                break
            keys, fit = best
            base_z = min(columns[key]['min_z'] for key in keys)
            top_z = max(columns[key]['max_z'] for key in keys)
            height = top_z - base_z
            ground_item = nearest_ground(ground, fit['x'], fit['y'])
            source_indices = []
            for key in keys:
                source_indices.extend(columns[key]['indices'])
            support = [(points[index][0], points[index][1], points[index][2], points[index][3], 0.0, 0.0)
                       for index in source_indices]
            reject, road_overlap, road_above = road_overhead_or_dynamic(support, road)
            ground_ok = ground_item is not None and -1.0 <= base_z - ground_item[1] <= 1.5
            if height >= 2.5 and ground_ok and not reject:
                selected_columns.update(keys)
                records.append({
                    'record_type': 'green_recovered_planar_possible', 'center_x': fit['x'], 'center_y': fit['y'],
                    'base_z': base_z, 'top_z': top_z, 'tangent_x': fit['tx'], 'tangent_y': fit['ty'],
                    'visible_length_m': fit['length'], 'plane_width_rms_m': fit['width'],
                    'visible_height_m': height, 'projection_coverage': fit['coverage'],
                    'source_column_count': len(keys), 'source_point_count': len(source_indices),
                    'nearest_ground_distance_m': ground_item[0], 'base_above_ground_m': base_z - ground_item[1],
                    'road_overlap_fraction': road_overlap, 'road_above_fraction': road_above,
                    'decision_reason': '绿色非平面层中检测到贴地、连续、窄且不悬在实测道路上的局部平面；仅提升为可能建筑标识。',
                })
            selected = set(keys)
            remaining = [key for key in remaining if key not in selected]
    recovered_indices = set()
    for key in selected_columns:
        recovered_indices.update(columns[key]['indices'])
    return recovered_indices, records


def classify_r2_objects(objects, road):
    """将R2建筑候选中显著悬在实测道路上方的结构降级为动态/树冠反证层。"""
    for item in objects:
        item['road_overlap_fraction'], item['road_above_fraction'] = road_metrics(item['support_points'], road)
        old = item['r2_class']
        reject, _, _ = road_overhead_or_dynamic(item['support_points'], road)
        if old in ('building_marker_candidate', 'building_marker_possible') and reject:
            item['r3_class'] = 'road_overhead_or_dynamic_rejected'
            item['r3_reason'] = '真实支持点大比例位于双边实测道路上方，保守排除动态幻影/树冠悬垂结构'
        else:
            item['r3_class'] = old
            item['r3_reason'] = item['decision_reason']


def write_records(path, objects, recovered):
    fields = ('record_id', 'record_type', 'r1_object_id', 'r1_object_type', 'r2_navigation_class',
              'r3_navigation_class', 'geometry_evidence_state', 'semantic_evidence_state', 'completeness',
              'center_x', 'center_y', 'base_z', 'top_z', 'tangent_x', 'tangent_y',
              'visible_length_m', 'plane_width_rms_m', 'visible_height_m', 'projection_coverage',
              'source_column_count', 'source_point_count', 'nearest_ground_distance_m',
              'base_above_ground_m', 'road_overlap_fraction', 'road_above_fraction',
              'decision_reason')
    with open(path, 'w') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        index = 0
        for item in objects:
            writer.writerow({
                'record_id': index, 'record_type': 'r2_object', 'r1_object_id': item['id'],
                'r1_object_type': item['object_type'], 'r2_navigation_class': item['r2_class'],
                'r3_navigation_class': item['r3_class'], 'geometry_evidence_state': 'observed',
                'semantic_evidence_state': 'inferred' if item['r3_class'].startswith('building_marker') else 'unknown',
                'completeness': 'partial', 'center_x': item['center_x'], 'center_y': item['center_y'],
                'base_z': item['base_z'], 'top_z': item['top_z'], 'tangent_x': item['tangent_x'],
                'tangent_y': item['tangent_y'], 'visible_length_m': item['visible_length_m'],
                'plane_width_rms_m': item['plane_width_rms_m'], 'visible_height_m': item['visible_height_m'],
                'projection_coverage': item['projection_coverage'], 'source_column_count': item['source_column_count'],
                'source_point_count': len(item['support_points']),
                'road_overlap_fraction': item['road_overlap_fraction'], 'road_above_fraction': item['road_above_fraction'],
                'decision_reason': item['r3_reason'],
            })
            index += 1
        for item in recovered:
            row = dict(item)
            row.update({'record_id': index, 'r1_object_id': '', 'r1_object_type': 'vegetation_or_nonplanar',
                        'r2_navigation_class': 'vegetation_or_nonplanar',
                        'r3_navigation_class': 'building_marker_possible_recovered',
                        'geometry_evidence_state': 'observed', 'semantic_evidence_state': 'inferred',
                        'completeness': 'partial'})
            writer.writerow(row)
            index += 1


def write_pcds(evidence_dir, objects, green_points, recovered_indices, recovered):
    colors = {
        'building_marker_candidate': packed_rgb(238, 122, 45),
        'building_marker_possible': packed_rgb(255, 181, 74),
        'road_overhead_or_dynamic_rejected': packed_rgb(210, 75, 95),
        'vegetation_or_nonplanar': packed_rgb(69, 177, 89),
        'pole_or_trunk_context': packed_rgb(54, 183, 198),
        'vertical_structure_context': packed_rgb(150, 150, 160),
        'vegetation_like_rejected': packed_rgb(69, 177, 89),
        'building_marker_possible_recovered': packed_rgb(255, 195, 89),
    }
    layers = defaultdict(list)
    for item in objects:
        for x, y, z, _rgb, _along, _cross in item['support_points']:
            layers[item['r3_class']].append((x, y, z, colors[item['r3_class']]))
    # 杆体保持R1完整真实支持层；绿色层去掉被局部平面回收的体素，避免双重显示。
    pole_path = os.path.join(os.path.dirname(evidence_dir), 'stage05_visible_scene_objects', 'evidence', 'pole_or_trunk_candidate_support.pcd')
    # 上述相对路径不可靠，调用方会传入R1来源；此变量在下方被替换。
    _ = pole_path
    for index, point in enumerate(green_points):
        if index in recovered_indices:
            layers['building_marker_possible_recovered'].append((point[0], point[1], point[2], colors['building_marker_possible_recovered']))
        else:
            layers['vegetation_or_nonplanar'].append((point[0], point[1], point[2], colors['vegetation_or_nonplanar']))
    for name, points in layers.items():
        if points:
            write_pcd(os.path.join(evidence_dir, name + '_support.pcd'), points)
    return dict((name, len(points)) for name, points in layers.items())


def add_pole_layer(layers_dir, source_dir, counts):
    points = [(x, y, z, packed_rgb(54, 183, 198)) for x, y, z, _rgb in
              read_binary_pcd(os.path.join(source_dir, 'pole_or_trunk_candidate_support.pcd'))]
    write_pcd(os.path.join(layers_dir, 'pole_or_trunk_context_support.pcd'), points)
    counts['pole_or_trunk_context'] = len(points)


def main():
    parser = argparse.ArgumentParser(description='阶段5 R3道路反证与绿色层局部平面回收')
    parser.add_argument('--stage05-r1', required=True)
    parser.add_argument('--stage05-r2', required=True)
    parser.add_argument('--stage02', required=True)
    parser.add_argument('--stage03', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = os.path.abspath(args.output)
    if os.path.exists(output):
        fail('R3输出目录已存在，拒绝覆盖：' + output)
    stage05_r1 = os.path.abspath(args.stage05_r1)
    stage02 = os.path.abspath(args.stage02)
    stage03 = os.path.abspath(args.stage03)
    source_dir = os.path.join(stage05_r1, 'evidence')
    objects = r2.read_objects(os.path.join(source_dir, 'visible_scene_object_records.csv'))
    unmatched = r2.assign_points(objects, r2.build_index(objects), source_dir)
    for item in objects:
        r2.enrich_features(item)
        item['r2_class'], item['decision_reason'] = r2.refine(item)
    road = read_observed_road(os.path.join(stage03, 'evidence', 'road_surface_candidate_complete_cells.csv'))
    ground = read_stable_ground(os.path.join(stage02, 'evidence', 'geometric_observation_grid.csv'))
    classify_r2_objects(objects, road)
    green_points = read_binary_pcd(os.path.join(source_dir, 'vegetation_or_nonplanar_support.pcd'))
    recovered_indices, recovered = recover_green_planes(green_points, ground, road)
    os.makedirs(os.path.join(output, 'evidence'))
    os.makedirs(os.path.join(output, 'validation'))
    evidence_dir = os.path.join(output, 'evidence')
    layer_counts = write_pcds(evidence_dir, objects, green_points, recovered_indices, recovered)
    add_pole_layer(evidence_dir, source_dir, layer_counts)
    write_records(os.path.join(evidence_dir, 'navigation_structure_r3_records.csv'), objects, recovered)
    class_counts = Counter(item['r3_class'] for item in objects)
    class_counts['building_marker_possible_recovered'] = len(recovered)
    report = {
        'schema': 'fast_livo_scene_pipeline_stage05_navigation_structure_refinement/v3',
        'status': 'complete',
        'purpose': '以双边实测道路面排除道路上方动态/树冠结构，并从绿色非平面层保守回收局部贴地平面。',
        'r1_object_count': len(objects), 'r3_class_counts': dict(class_counts),
        'r3_support_point_counts': layer_counts,
        'green_source_point_count': len(green_points), 'green_recovered_point_count': len(recovered_indices),
        'unmatched_r1_planar_support_points_by_source_class': dict(unmatched),
        'limits': [
            '道路反证只使用双边实测高置信道路格；没有使用桥接或固定宽度道路。',
            '绿色层回收只产生possible标识，不形成完整建筑块。',
            '动态物与静态细杆、局部平整绿篱仍可能混淆；必须人工复核橙/浅橙/红三层。',
        ],
    }
    with open(os.path.join(output, 'validation', 'stage05_navigation_structure_r3_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'stage05_r3_complete.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '05_navigation_structure_refinement_r3',
                   'source_stage05_r1': stage05_r1, 'source_stage05_r2': os.path.abspath(args.stage05_r2),
                   'records': 'evidence/navigation_structure_r3_records.csv'}, handle,
                  ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    print('阶段5 R3完成：道路上方拒绝=%d，绿色层局部平面回收=%d。' % (
        class_counts['road_overhead_or_dynamic_rejected'], len(recovered)))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段5 R3失败：%s' % error, file=sys.stderr)
        sys.exit(1)
