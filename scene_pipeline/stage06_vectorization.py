#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 6：把冻结的道路、边界和可见对象证据整理为可审计矢量地图。

本脚本刻意不做以下事情：

* 不重新判定阶段 3--5 的证据，也不读取 archive 中的试验结果；
* 不用固定宽度候选填补为 observed 道路；
* 不把可见立面延伸为完整建筑轮廓；
* 不生成产品颜色、阴影或网页。那些属于阶段 7。

输出同时保留几何证据状态与语义证据状态。GeoJSON 中的坐标是 FAST-LIVO2 的
局部 map 坐标（米），没有被虚构成 WGS84 经纬度。
"""

from __future__ import print_function

import argparse
import csv
import hashlib
import json
import math
import os
import sys
from collections import Counter, defaultdict, deque

from common import geometry as base


SCHEMA = 'fast_livo_scene_pipeline_stage06_vector_map/v1'
GRID_M = base.GRID_M


def fail(message):
    raise RuntimeError(message)


def sha256_file(path):
    """计算输入哈希；只读冻结包时使用。"""
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        while True:
            block = handle.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def verify_frozen_tree(tree):
    """复用冻结包的严格语义：目录中不能新增、缺失或替换文件。"""
    root = tree['path']
    if not os.path.isdir(root):
        fail('%s目录缺失：%s' % (tree.get('label', '冻结输出'), root))
    expected = dict((item['relative_path'], item) for item in tree['files'])
    actual = set()
    digest = hashlib.sha256()
    for current, _, names in os.walk(root):
        for name in sorted(names):
            path = os.path.join(current, name)
            relative = os.path.relpath(path, root)
            actual.add(relative)
            entry = expected.get(relative)
            if entry is None:
                fail('%s出现未冻结文件：%s' % (tree.get('label', '冻结输出'), relative))
            value = sha256_file(path)
            if value != entry['sha256']:
                fail('%s文件SHA-256不匹配：%s' % (tree.get('label', '冻结输出'), relative))
            digest.update(relative.encode('utf-8'))
            digest.update(b'\0')
            digest.update(value.encode('ascii'))
            digest.update(b'\0')
    if actual != set(expected):
        missing = sorted(set(expected).difference(actual))
        fail('%s缺少冻结文件：%s' % (tree.get('label', '冻结输出'), ', '.join(missing)))
    if digest.hexdigest() != tree['tree_sha256']:
        fail('%s输出树SHA-256不匹配' % tree.get('label', '冻结输出'))


def verify_frozen_handoff(path, manifest_name, expected_stage):
    """验证交接包本身、其绑定输入、实现以及已接受的输出树。"""
    manifest_path = os.path.join(path, manifest_name)
    if not os.path.isfile(manifest_path):
        fail('缺少冻结清单：' + manifest_path)
    with open(manifest_path, 'r') as handle:
        manifest = json.load(handle)
    if manifest.get('status') != 'frozen':
        fail('%s状态不是 frozen' % expected_stage)
    if manifest.get('stage') != expected_stage:
        fail('冻结清单阶段不匹配：期望%s，实际%s' % (expected_stage, manifest.get('stage')))
    for item in manifest.get('upstream_frozen_inputs', []) + manifest.get('implementation_files', []):
        input_path = item['path']
        if not os.path.isfile(input_path):
            fail('%s缺失：%s' % (item.get('label', '冻结输入'), input_path))
        if sha256_file(input_path) != item['sha256']:
            fail('%s SHA-256不匹配：%s' % (item.get('label', '冻结输入'), input_path))
    for tree in manifest.get('accepted_output_trees', []):
        verify_frozen_tree(tree)
    return manifest


def read_csv(path, required_fields):
    """读取并检查 CSV 字段，避免静默使用错误版本的上游输出。"""
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        missing = [name for name in required_fields if name not in fields]
        if missing:
            fail('CSV字段不兼容：%s；缺少%s' % (path, ', '.join(missing)))
        return list(reader)


def write_json(path, value):
    with open(path, 'w') as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')


def write_geojson(path, features, layer_name, description):
    value = {
        'type': 'FeatureCollection',
        'name': layer_name,
        'crs_note': '坐标为FAST-LIVO2回环优化后的局部map坐标（米），不是WGS84经纬度。',
        'description': description,
        'features': features,
    }
    write_json(path, value)


def feature(feature_id, geometry, properties):
    result = {'type': 'Feature', 'id': feature_id, 'geometry': geometry, 'properties': properties}
    return result


def float_or_none(value):
    return None if value is None or value == '' else float(value)


def int_or_none(value):
    return None if value is None or value == '' else int(value)


def bool_value(value):
    return value in ('1', 'true', 'True', 1, True)


def road_axis_xy(row):
    """从阶段3左右边界距离恢复道路中轴，而非错误使用机器狗轨迹参考点。

    ``x,y`` 是横断面（通常也是机器狗）参考位置。仅当同一横断面拥有左右距离时，
    才能用其法向把参考点平移到道路中轴；单侧、开放区和默认宽度候选没有这种资格。
    """
    reference_x, reference_y = float(row['x']), float(row['y'])
    candidate = row['candidate_class']
    left = float_or_none(row['left_boundary_distance_m'])
    right = float_or_none(row['right_boundary_distance_m'])
    if candidate not in ('two_sided_measured_road_core', 'two_sided_continuity_bridge') or left is None or right is None:
        return reference_x, reference_y, None
    normal_x, normal_y = -float(row['tangent_y']), float(row['tangent_x'])
    offset = (left - right) * 0.5
    return reference_x + normal_x * offset, reference_y + normal_y * offset, offset


def evidence_for_road_cell(row):
    """阶段 3 的栅格来源是事实，不能按颜色或渲染需要重新解释。"""
    source = row['road_candidate_primary_source']
    if source == 'two_sided_measured' and bool_value(row['observed']):
        return 'observed', 'observed', 'road_surface_observed'
    if source == 'two_sided_continuity_bridge':
        return 'observed', 'inferred', 'road_search_surface_continuity'
    if source == 'fixed_width_fallback':
        return 'observed', 'inferred', 'road_search_surface_fixed_width'
    return 'observed', 'unknown', 'road_search_surface_unknown'


def grid_components(cells):
    """按四连通拆分栅格；对角接触不能被错误合并成一条道路。"""
    remaining = set(cells)
    components = []
    while remaining:
        start = remaining.pop()
        component = {start}
        queue = deque([start])
        while queue:
            ix, iy = queue.popleft()
            for neighbor in ((ix - 1, iy), (ix + 1, iy), (ix, iy - 1), (ix, iy + 1)):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return components


def signed_area(ring):
    area = 0.0
    for first, second in zip(ring, ring[1:]):
        area += first[0] * second[1] - second[0] * first[1]
    return area * 0.5


def point_in_ring(point, ring):
    """射线法，仅用于将栅格环洞归属到包含它的外环。"""
    inside = False
    x, y = point
    for first, second in zip(ring, ring[1:]):
        x1, y1 = first
        x2, y2 = second
        if (y1 > y) != (y2 > y):
            hit_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < hit_x:
                inside = not inside
    return inside


def cell_component_geometry(component):
    """将0.2 m实测栅格并集转换为不插值的Polygon/MultiPolygon边界。

    边界只来自存在的原始栅格边；这不是平滑、外推或道路补面。洞被保留，以免把未观测
    小片区错误涂成道路或人行道。
    """
    edges = set()
    for ix, iy in component:
        # 以网格索引表示顶点，四条边逆时针。共享边会互相抵消。
        cell_edges = (((ix, iy), (ix + 1, iy)), ((ix + 1, iy), (ix + 1, iy + 1)),
                      ((ix + 1, iy + 1), (ix, iy + 1)), ((ix, iy + 1), (ix, iy)))
        for edge in cell_edges:
            opposite = (edge[1], edge[0])
            if opposite in edges:
                edges.remove(opposite)
            else:
                edges.add(edge)
    outgoing = defaultdict(list)
    for start, end in edges:
        outgoing[start].append(end)
    rings = []
    while edges:
        start, end = next(iter(edges))
        ring = [start]
        current = start
        next_vertex = end
        while True:
            edge = (current, next_vertex)
            if edge not in edges:
                fail('栅格边界追踪失败，出现重复或断裂边')
            edges.remove(edge)
            current = next_vertex
            ring.append(current)
            if current == start:
                break
            candidates = [item for item in outgoing[current] if (current, item) in edges]
            if not candidates:
                fail('栅格边界出现断裂，无法安全矢量化')
            if len(candidates) == 1:
                next_vertex = candidates[0]
            else:
                # 四连通栅格在“只碰到一个角”的位置会有多个候选边。它们不表示可将
                # 两块道路合为一体；按最小正转角（优先左转）继续，可让两个环各自闭合。
                incoming_x = current[0] - ring[-2][0]
                incoming_y = current[1] - ring[-2][1]
                direction = {(1, 0): 0, (0, 1): 1, (-1, 0): 2, (0, -1): 3}
                previous_dir = direction[(incoming_x, incoming_y)]
                scored = []
                for candidate in candidates:
                    delta = (candidate[0] - current[0], candidate[1] - current[1])
                    candidate_dir = direction.get(delta)
                    if candidate_dir is None:
                        fail('栅格边界含非轴向边')
                    turn = (candidate_dir - previous_dir) % 4
                    # 共享边已经抵消，仍出现反向边说明输入边界存在矛盾，不能猜测。
                    if turn == 2:
                        continue
                    scored.append((turn, candidate))
                if not scored:
                    fail('栅格边界后继与当前边矛盾')
                next_vertex = min(scored, key=lambda item: item[0])[1]
        coordinate_ring = [[vertex[0] * GRID_M, vertex[1] * GRID_M] for vertex in ring]
        rings.append(coordinate_ring)
    outer = [ring for ring in rings if signed_area(ring) > 0.0]
    holes = [ring for ring in rings if signed_area(ring) < 0.0]
    # 理论上外环均为正向。异常时宁可单独输出，不能擅自填洞。
    if not outer:
        outer = rings
        holes = []
    polygons = [[ring] for ring in outer]
    for hole in holes:
        probe = hole[0]
        containers = [index for index, ring in enumerate(outer) if point_in_ring(probe, ring)]
        if containers:
            polygons[containers[0]].append(hole)
        else:
            # 不安全的洞关系作为独立多边形输出，避免丢失几何。
            polygons.append([hole])
    if len(polygons) == 1:
        return {'type': 'Polygon', 'coordinates': polygons[0]}
    return {'type': 'MultiPolygon', 'coordinates': polygons}


def rdp_indices(points, tolerance):
    """Ramer--Douglas--Peucker：仅删除共线近似站点，保留原始采样位置。"""
    if len(points) <= 2:
        return list(range(len(points)))

    def distance_to_line(point, first, last):
        dx = last[0] - first[0]
        dy = last[1] - first[1]
        length = math.hypot(dx, dy)
        if length < 1e-12:
            return math.hypot(point[0] - first[0], point[1] - first[1])
        return abs(dy * point[0] - dx * point[1] + last[0] * first[1] - last[1] * first[0]) / length

    kept = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        first, last = stack.pop()
        candidate, maximum = None, tolerance
        for index in range(first + 1, last):
            value = distance_to_line(points[index], points[first], points[last])
            if value > maximum:
                candidate, maximum = index, value
        if candidate is not None:
            kept.add(candidate)
            stack.append((first, candidate))
            stack.append((candidate, last))
    return sorted(kept)


def grouped_cross_sections(rows):
    """按相邻采样、类别和状态切段，不跨越低证据段或类别边界。"""
    groups = []
    current = []
    current_key = None
    previous_id = None
    for row in rows:
        source = row['candidate_class']
        if source == 'two_sided_measured_road_core':
            geometry_state, semantic_state, layer = 'observed', 'observed', 'road_centerline_observed'
        elif source == 'two_sided_continuity_bridge':
            geometry_state, semantic_state, layer = 'observed', 'inferred', 'road_centerline_continuity'
        elif source == 'one_sided_measured_boundary':
            geometry_state, semantic_state, layer = 'observed', 'unknown', 'trajectory_reference_direction_candidate'
        elif source in ('two_sided_abnormal_width_open_candidate', 'no_boundary_default_width_candidate'):
            geometry_state, semantic_state, layer = 'observed', 'unknown', 'junction_or_open_area_candidate'
        else:
            fail('未识别的阶段3横断面类别：' + source)
        key = (source, row['confidence'], geometry_state, semantic_state, layer,
               row['is_measured_true_width'], row['bridge_kind'])
        sample_id = int(row['sample_id'])
        # 连续样本的期望间距小于0.5 m；大跳跃必须拆开，防止跨回环错误连线。
        if current and (key != current_key or sample_id != previous_id + 1):
            groups.append((current_key, current))
            current = []
        current_key = key
        current.append(row)
        previous_id = sample_id
    if current:
        groups.append((current_key, current))
    return groups


def cross_section_features(rows):
    features = []
    profiles = []
    for index, (key, group) in enumerate(grouped_cross_sections(rows)):
        source, confidence, geometry_state, semantic_state, layer, measured, bridge_kind = key
        # 双边段使用由左右边界推得的道路中轴；其它段保留轨迹参考，且不会被命名为中心线。
        points = [road_axis_xy(row)[:2] for row in group]
        if len(points) == 1:
            # 单站点没有线方向；作为 Point 输出，不把它人为延伸。
            geometry = {'type': 'Point', 'coordinates': list(points[0])}
            retained_ids = [int(group[0]['sample_id'])]
        else:
            indices = rdp_indices(points, 0.10)
            geometry = {'type': 'LineString', 'coordinates': [list(points[item]) for item in indices]}
            retained_ids = [int(group[item]['sample_id']) for item in indices]
        widths = [float_or_none(row['local_width_m']) for row in group]
        measured_widths = [value for value in widths if value is not None]
        properties = {
            'layer': layer,
            'source_stage': '03_final_handoff',
            'source_csv': 'evidence/road_cross_section_confidence.csv',
            'source_sample_id_start': int(group[0]['sample_id']),
            'source_sample_id_end': int(group[-1]['sample_id']),
            'retained_source_sample_ids': retained_ids,
            'reference_position_kind': 'cross_section_or_trajectory_reference',
            'axis_position_kind': 'two_sided_boundary_midpoint' if source in (
                'two_sided_measured_road_core', 'two_sided_continuity_bridge') else 'not_available_use_reference_only',
            'candidate_class': source,
            'confidence': confidence,
            'geometry_evidence_state': geometry_state,
            'semantic_evidence_state': semantic_state,
            'evidence_state': semantic_state,
            'width_evidence_state': 'observed' if measured == '1' else ('inferred' if source == 'two_sided_continuity_bridge' else 'unknown'),
            'width_min_m': min(measured_widths) if measured_widths else None,
            'width_max_m': max(measured_widths) if measured_widths else None,
            'width_mean_m': sum(measured_widths) / len(measured_widths) if measured_widths else None,
            'bridge_kind': bridge_kind or None,
            'render_policy': 'eligible' if layer == 'road_centerline_observed' else 'not_final_road',
        }
        features.append(feature('cross_section_segment_%04d' % index, geometry, properties))
        for row in group:
            axis_x, axis_y, axis_offset = road_axis_xy(row)
            profiles.append({
                'sample_id': int(row['sample_id']), 'x': float(row['x']), 'y': float(row['y']),
                'road_axis_x': axis_x if axis_offset is not None else None,
                'road_axis_y': axis_y if axis_offset is not None else None,
                'axis_offset_from_reference_m': axis_offset,
                'tangent_x': float(row['tangent_x']), 'tangent_y': float(row['tangent_y']),
                'candidate_class': source, 'confidence': confidence,
                'geometry_evidence_state': geometry_state, 'semantic_evidence_state': semantic_state,
                'evidence_state': semantic_state,
                'local_width_m': float_or_none(row['local_width_m']),
                'width_evidence_state': 'observed' if row['is_measured_true_width'] == '1' else (
                    'inferred' if source == 'two_sided_continuity_bridge' else 'unknown'),
                'left_boundary_distance_m': float_or_none(row['left_boundary_distance_m']),
                'right_boundary_distance_m': float_or_none(row['right_boundary_distance_m']),
                'bridge_kind': row['bridge_kind'] or None,
            })
    return features, profiles


def road_surface_features(rows):
    """分别输出实测道路面和两类只能检索、不能当最终道路的推断搜索面。"""
    groups = defaultdict(list)
    for row in rows:
        geometry_state, semantic_state, layer = evidence_for_road_cell(row)
        groups[(layer, geometry_state, semantic_state, row['road_candidate_primary_source'],
                row['road_membership_confidence'], row['width_source'])].append(row)
    features = []
    serial = 0
    for key in sorted(groups):
        layer, geometry_state, semantic_state, source, confidence, width_source = key
        records = groups[key]
        cells = set((int(row['cell_ix']), int(row['cell_iy'])) for row in records)
        z_values = [float(row['ground_z']) for row in records]
        support = [int(row['cross_section_support_count']) for row in records]
        for component_index, component in enumerate(grid_components(cells)):
            component_records = [row for row in records if (int(row['cell_ix']), int(row['cell_iy'])) in component]
            component_z = [float(row['ground_z']) for row in component_records]
            component_support = [int(row['cross_section_support_count']) for row in component_records]
            properties = {
                'layer': layer,
                'source_stage': '03_final_handoff',
                'source_csv': 'evidence/road_surface_candidate_complete_cells.csv',
                'road_candidate_primary_source': source,
                'road_membership_confidence': confidence,
                'geometry_evidence_state': geometry_state,
                'semantic_evidence_state': semantic_state,
                'evidence_state': semantic_state,
                'width_source': width_source,
                'cell_count': len(component),
                'grid_resolution_m': GRID_M,
                'ground_z_min_m': min(component_z),
                'ground_z_max_m': max(component_z),
                'cross_section_support_count_min': min(component_support),
                'cross_section_support_count_max': max(component_support),
                'render_policy': 'eligible' if layer == 'road_surface_observed' else 'not_final_road',
                'warning': None if layer == 'road_surface_observed' else '该面只保留阶段3连续搜索证据，不是实测道路宽度或最终道路面。',
            }
            features.append(feature('road_surface_%04d' % serial, cell_component_geometry(component), properties))
            serial += 1
    return features


def boundary_features(records):
    curb, unknown = [], []
    for row in records:
        geometry = {'type': 'LineString', 'coordinates': [[float(row['start_x']), float(row['start_y'])],
                                                            [float(row['end_x']), float(row['end_y'])]]}
        properties = {
            'source_stage': '04_r7',
            'source_csv': 'evidence/boundary_semantic_records.csv',
            'boundary_band_id': int(row['boundary_band_id']),
            'boundary_source': row['boundary_source'],
            'source_primitive_ids': row['source_primitive_ids'],
            'side': row['side'],
            'semantic_class': row['semantic_class'],
            'classification_confidence': float(row['classification_confidence']),
            'geometry_evidence_state': row['evidence_state'],
            'semantic_evidence_state': row['evidence_state'],
            'evidence_state': row['evidence_state'],
            'continuous_length_m': float(row['continuous_length_m']),
            'relative_road_distance_m': float(row['relative_road_distance_m']),
            'local_height_delta_m': float(row['local_height_delta_m']),
            'direction_angle_deg': float(row['direction_angle_deg']),
            'hard_edge_support_fraction': float(row['hard_edge_support_fraction']),
            'road_support_fraction': float(row['road_support_fraction']),
            'rejection_reason': row['rejection_reason'] or None,
            'render_policy': 'eligible_as_curb_candidate' if row['semantic_class'] == 'curb_candidate' else 'not_curb_unknown',
        }
        item = feature('boundary_%s' % row['boundary_band_id'], geometry, properties)
        if row['semantic_class'] == 'curb_candidate':
            curb.append(item)
        else:
            unknown.append(item)
    return curb, unknown


def sidewalk_footprint_features(pcd_path, records):
    """确认人行道面几何只来自确认层PCD中的稳定地面栅格，不由边界外推。"""
    points = base.read_binary_pcd(pcd_path)
    cells = set(base.grid_key(x, y) for x, y, _, _ in points)
    if not cells:
        return []
    confirmed = [row for row in records if row['classification'] == 'confirmed_sidewalk_or_shoulder']
    observed = [row for row in confirmed if row['evidence_state'] == 'observed']
    feature_list = []
    for index, component in enumerate(grid_components(cells)):
        properties = {
            'layer': 'sidewalk_or_shoulder_confirmed',
            'source_stage': '04_r7',
            'source_geometry': 'evidence/sidewalk_confirmed_support.pcd',
            'source_records': 'evidence/sidewalk_surface_records.csv',
            'classification': 'confirmed_sidewalk_or_shoulder',
            'geometry_evidence_state': 'observed',
            'semantic_evidence_state': 'observed',
            'evidence_state': 'observed',
            'cell_count': len(component),
            'grid_resolution_m': GRID_M,
            'confirmed_surface_record_count': len(confirmed),
            'observed_surface_record_count': len(observed),
            'record_link_note': '阶段4审计CSV未保存每个确认地面栅格到surface_id的一对一映射；本面只可回溯至确认层PCD和确认记录集合。',
            'render_policy': 'eligible',
        }
        feature_list.append(feature('confirmed_sidewalk_%04d' % index, cell_component_geometry(component), properties))
    return feature_list


def constraint_features(records):
    constraints, context = [], []
    for row in records:
        geometry = {'type': 'LineString', 'coordinates': [[float(row['start_x']), float(row['start_y'])],
                                                            [float(row['end_x']), float(row['end_y'])]]}
        is_constraint = row['candidate_status'] == 'inferred_road_edge_constraint'
        properties = {
            'source_stage': '04_2_r2',
            'source_csv': 'evidence/inferred_road_edge_constraint_records.csv',
            'boundary_band_id': int(row['boundary_band_id']),
            'candidate_status': row['candidate_status'],
            'side': row['side'],
            'geometry_evidence_state': row['evidence_state'],
            'semantic_evidence_state': row['evidence_state'],
            'evidence_state': row['evidence_state'],
            'constraint_confidence': float(row['constraint_confidence']),
            'continuous_length_m': float(row['continuous_length_m']),
            'relative_road_distance_m': float(row['relative_road_distance_m']),
            'direction_angle_deg': float(row['direction_angle_deg']),
            'nearest_junction_distance_m': float(row['nearest_junction_distance_m']),
            'reason': row['reason'],
            'render_policy': 'not_final_road_boundary',
            'warning': '仅可作阶段6低置信度道路延续约束；不是observed道路、道路宽度或路沿。',
        }
        item = feature('road_edge_%s' % row['boundary_band_id'], geometry, properties)
        (constraints if is_constraint else context).append(item)
    return constraints, context


def object_features(records):
    """对象只导出实际拟合得到的可见线段，绝不输出完整建筑面或方块。"""
    groups = defaultdict(list)
    for row in records:
        length = float(row['visible_length_m'])
        tx, ty = float(row['tangent_x']), float(row['tangent_y'])
        cx, cy = float(row['center_x']), float(row['center_y'])
        half = length * 0.5
        # tangent 来自阶段5局部平面拟合；端点是该可见拟合段的表示，不是建筑外轮廓。
        coordinates = [[cx - tx * half, cy - ty * half, float(row['base_z'])],
                       [cx + tx * half, cy + ty * half, float(row['base_z'])]]
        object_class = row['r3_navigation_class']
        if object_class == 'building_marker_candidate':
            target = 'building_marker_candidates'
        elif object_class in ('building_marker_possible', 'building_marker_possible_recovered'):
            target = 'building_marker_possible'
        else:
            target = 'scene_object_context'
        properties = {
            'layer': target,
            'source_stage': '05_r3',
            'source_csv': 'evidence/navigation_structure_r3_records.csv',
            'record_id': int(row['record_id']),
            # R3绿色层回收对象并非都来自R1对象，因此R1编号在冻结CSV中合法地为空。
            'r1_object_id': int_or_none(row['r1_object_id']),
            'r1_object_type': row['r1_object_type'],
            'r2_navigation_class': row['r2_navigation_class'],
            'r3_navigation_class': object_class,
            'geometry_evidence_state': row['geometry_evidence_state'],
            'semantic_evidence_state': row['semantic_evidence_state'],
            'evidence_state': row['semantic_evidence_state'],
            'completeness': row['completeness'],
            'base_z_m': float(row['base_z']),
            'top_z_m': float(row['top_z']),
            'visible_length_m': length,
            'visible_height_m': float(row['visible_height_m']),
            'plane_width_rms_m': float(row['plane_width_rms_m']),
            'projection_coverage': float(row['projection_coverage']),
            'source_column_count': float(row['source_column_count']),
            'source_point_count': int(float(row['source_point_count'])),
            'nearest_ground_distance_m': float_or_none(row['nearest_ground_distance_m']),
            'base_above_ground_m': float_or_none(row['base_above_ground_m']),
            'road_overlap_fraction': float(row['road_overlap_fraction']),
            'road_above_fraction': float(row['road_above_fraction']),
            'decision_reason': row['decision_reason'],
            'geometry_note': 'LineString表示阶段5拟合到的可见立面/垂直结构主方向和可见长度，非完整建筑轮廓。',
            'render_policy': 'optional_building_marker' if target != 'scene_object_context' else 'context_not_building',
        }
        groups[target].append(feature('scene_object_%s' % row['record_id'],
                                      {'type': 'LineString', 'coordinates': coordinates}, properties))
    return groups


def open_area_features(cross_rows, pcd_path):
    """低证据横断面只保留为开放区候选线；其PCD栅格也单独保留为unknown面。"""
    samples = [row for row in cross_rows if row['candidate_class'] in (
        'two_sided_abnormal_width_open_candidate', 'no_boundary_default_width_candidate')]
    sample_features = []
    for index, (key, group) in enumerate(grouped_cross_sections(samples)):
        points = [[float(row['x']), float(row['y'])] for row in group]
        geometry = {'type': 'Point', 'coordinates': points[0]} if len(points) == 1 else {
            'type': 'LineString', 'coordinates': points}
        sample_features.append(feature('open_area_samples_%04d' % index, geometry, {
            'layer': 'junction_or_open_area_candidate', 'source_stage': '03_final_handoff',
            'source_csv': 'evidence/road_cross_section_confidence.csv',
            'candidate_class': group[0]['candidate_class'], 'confidence': group[0]['confidence'],
            'geometry_evidence_state': 'observed', 'semantic_evidence_state': 'unknown',
            'evidence_state': 'unknown', 'render_policy': 'not_final_road',
            'warning': '开放区/路口候选，不能按普通道路宽度补面。',
        }))
    pcd_cells = set(base.grid_key(x, y) for x, y, _, _ in base.read_binary_pcd(pcd_path))
    surface_features = []
    for index, component in enumerate(grid_components(pcd_cells)):
        surface_features.append(feature('open_area_support_%04d' % index, cell_component_geometry(component), {
            'layer': 'junction_or_open_area_support', 'source_stage': '04_r7',
            'source_geometry': 'evidence/junction_or_open_area_candidate_support.pcd',
            'geometry_evidence_state': 'observed', 'semantic_evidence_state': 'unknown',
            'evidence_state': 'unknown', 'cell_count': len(component), 'grid_resolution_m': GRID_M,
            'render_policy': 'not_final_road',
            'warning': '这是待复核的路口/开放区支持面，不是普通道路面。',
        }))
    return sample_features, surface_features


def write_width_profiles(path, profiles):
    fields = ('sample_id', 'x', 'y', 'road_axis_x', 'road_axis_y', 'axis_offset_from_reference_m',
              'tangent_x', 'tangent_y', 'candidate_class', 'confidence',
              'geometry_evidence_state', 'semantic_evidence_state', 'evidence_state', 'local_width_m',
              'width_evidence_state', 'left_boundary_distance_m', 'right_boundary_distance_m', 'bridge_kind')
    with open(path, 'w') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for item in profiles:
            writer.writerow(item)


def append_audit(rows, layer, features):
    for item in features:
        prop = item['properties']
        rows.append({
            'feature_id': item['id'], 'layer': layer, 'geometry_type': item['geometry']['type'],
            'source_stage': prop.get('source_stage'), 'source_reference': prop.get('source_csv', prop.get('source_geometry')),
            'geometry_evidence_state': prop.get('geometry_evidence_state'),
            'semantic_evidence_state': prop.get('semantic_evidence_state'),
            'evidence_state': prop.get('evidence_state'), 'confidence': prop.get('classification_confidence',
                prop.get('constraint_confidence', prop.get('road_membership_confidence', prop.get('confidence')))),
            'render_policy': prop.get('render_policy'),
            'source_id': prop.get('boundary_band_id', prop.get('record_id', prop.get('source_sample_id_start'))),
            'note': prop.get('warning', prop.get('geometry_note', prop.get('rejection_reason', prop.get('decision_reason', '')))),
        })


def write_audit(path, rows):
    fields = ('feature_id', 'layer', 'geometry_type', 'source_stage', 'source_reference',
              'geometry_evidence_state', 'semantic_evidence_state', 'evidence_state', 'confidence',
              'render_policy', 'source_id', 'note')
    with open(path, 'w') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def nearest_road_z(road_z, x, y):
    """为二维矢量复核线找最近道路地面高度。

    阶段4边界CSV本身没有端点Z；这个高度只为了让CloudCompare中线条能叠加查看，
    绝不是对路沿高度的重新估计。
    """
    center = base.grid_key(x, y)
    best = None
    for dx in range(-3, 4):
        for dy in range(-3, 4):
            key = (center[0] + dx, center[1] + dy)
            if key not in road_z:
                continue
            cx, cy = base.cell_center(key)
            distance = math.hypot(cx - x, cy - y)
            if best is None or distance < best[0]:
                best = (distance, road_z[key])
    return 0.0 if best is None else best[1]


def sample_line_2d(start, end, z_start, z_end, color, spacing=0.10):
    """按固定间距采样审计线；点密度只服务显示，不表示观测次数。"""
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, int(math.ceil(length / spacing)))
    result = []
    for index in range(count + 1):
        alpha = float(index) / float(count)
        result.append((start[0] * (1.0 - alpha) + end[0] * alpha,
                       start[1] * (1.0 - alpha) + end[1] * alpha,
                       z_start * (1.0 - alpha) + z_end * alpha, color))
    return result


def object_review_points(records, classes, color):
    """将阶段5可见拟合段写成底边+两根高度线，明确它不是建筑体块。"""
    points = []
    for row in records:
        if row['r3_navigation_class'] not in classes:
            continue
        length = float(row['visible_length_m'])
        tx, ty = float(row['tangent_x']), float(row['tangent_y'])
        cx, cy = float(row['center_x']), float(row['center_y'])
        base_z, top_z = float(row['base_z']), float(row['top_z'])
        half = length * 0.5
        left = (cx - tx * half, cy - ty * half)
        right = (cx + tx * half, cy + ty * half)
        points.extend(sample_line_2d(left, right, base_z, base_z, color))
        points.extend(sample_line_2d(left, left, base_z, top_z, color))
        points.extend(sample_line_2d(right, right, base_z, top_z, color))
    return points


def write_pcd_review_layers(pcd_dir, road_rows, cross_rows, boundary_rows, sidewalk_pcd_path,
                            constraint_rows, object_rows, open_pcd_path):
    """输出供CloudCompare叠加验收的矢量副本，所有颜色契约写入同目录JSON。

    这里的线点由阶段6矢量端点等距采样而来，不能拿点的稠密程度解释为多帧支持；
    面点则直接复制对应冻结支持层的栅格位置并改变显示色。
    """
    colors = {
        'road_surface_observed': base.packed_rgb(50, 145, 235),
        'road_surface_continuity': base.packed_rgb(170, 220, 70),
        'road_surface_fixed_width': base.packed_rgb(185, 155, 245),
        'centerline_observed': base.packed_rgb(245, 245, 245),
        'centerline_inferred': base.packed_rgb(255, 225, 65),
        'centerline_unknown': base.packed_rgb(145, 145, 145),
        'curb_candidate': base.packed_rgb(245, 155, 40),
        'unknown_boundary': base.packed_rgb(135, 135, 135),
        'sidewalk_confirmed': base.packed_rgb(75, 215, 230),
        'inferred_constraint': base.packed_rgb(180, 85, 245),
        'context_edge': base.packed_rgb(110, 75, 150),
        'open_area': base.packed_rgb(225, 75, 145),
        'building_candidate': base.packed_rgb(255, 130, 30),
        'building_possible': base.packed_rgb(255, 205, 110),
        'scene_context': base.packed_rgb(85, 175, 95),
    }
    road_z = {}
    road_points = []
    for row in road_rows:
        cell = (int(row['cell_ix']), int(row['cell_iy']))
        z = float(row['ground_z'])
        road_z[cell] = z
        source = row['road_candidate_primary_source']
        color_key = {'two_sided_measured': 'road_surface_observed',
                     'two_sided_continuity_bridge': 'road_surface_continuity',
                     'fixed_width_fallback': 'road_surface_fixed_width'}[source]
        road_points.append((float(row['x']), float(row['y']), z + 0.02, colors[color_key]))

    centerline_points = []
    trajectory_reference_points = []
    for row in cross_rows:
        candidate = row['candidate_class']
        if candidate == 'two_sided_measured_road_core':
            color_key = 'centerline_observed'
        elif candidate == 'two_sided_continuity_bridge':
            color_key = 'centerline_inferred'
        else:
            # 单侧/开放区没有左右边界中点，不是道路中心线；单列为轨迹参考语境。
            x, y = float(row['x']), float(row['y'])
            trajectory_reference_points.append((x, y, nearest_road_z(road_z, x, y) + 0.08,
                                                colors['centerline_unknown']))
            continue
        x, y, offset = road_axis_xy(row)
        if offset is None:
            fail('双边道路横断面缺少可用中轴偏移：样本' + row['sample_id'])
        centerline_points.append((x, y, nearest_road_z(road_z, x, y) + 0.08, colors[color_key]))

    boundary_points = []
    for row in boundary_rows:
        start = (float(row['start_x']), float(row['start_y']))
        end = (float(row['end_x']), float(row['end_y']))
        z_start = nearest_road_z(road_z, start[0], start[1]) + 0.10
        z_end = nearest_road_z(road_z, end[0], end[1]) + 0.10
        color_key = 'curb_candidate' if row['semantic_class'] == 'curb_candidate' else 'unknown_boundary'
        boundary_points.extend(sample_line_2d(start, end, z_start, z_end, colors[color_key]))

    sidewalk_points = [(x, y, z + 0.04, colors['sidewalk_confirmed'])
                       for x, y, z, _ in base.read_binary_pcd(sidewalk_pcd_path)]

    constraint_points, context_points = [], []
    for row in constraint_rows:
        start = (float(row['start_x']), float(row['start_y']))
        end = (float(row['end_x']), float(row['end_y']))
        z_start = nearest_road_z(road_z, start[0], start[1]) + 0.14
        z_end = nearest_road_z(road_z, end[0], end[1]) + 0.14
        target = constraint_points if row['candidate_status'] == 'inferred_road_edge_constraint' else context_points
        color_key = 'inferred_constraint' if target is constraint_points else 'context_edge'
        target.extend(sample_line_2d(start, end, z_start, z_end, colors[color_key]))

    open_points = [(x, y, z + 0.06, colors['open_area'])
                   for x, y, z, _ in base.read_binary_pcd(open_pcd_path)]
    building_candidate = object_review_points(object_rows, {'building_marker_candidate'}, colors['building_candidate'])
    building_possible = object_review_points(object_rows,
                                              {'building_marker_possible', 'building_marker_possible_recovered'},
                                              colors['building_possible'])
    scene_context = object_review_points(object_rows,
                                         {'pole_or_trunk_context', 'vertical_structure_context',
                                          'vegetation_like_rejected', 'vegetation_or_nonplanar',
                                          'road_overhead_or_dynamic_rejected'}, colors['scene_context'])

    layers = (
        ('01_road_surface_vector_review.pcd', road_points),
        ('02_road_centerline_vector_review.pcd', centerline_points),
        ('02b_trajectory_reference_context.pcd', trajectory_reference_points),
        ('03_boundary_vector_review.pcd', boundary_points),
        ('04_sidewalk_confirmed_vector_review.pcd', sidewalk_points),
        ('05_inferred_road_edge_constraint_vector_review.pcd', constraint_points),
        ('06_road_edge_context_review.pcd', context_points),
        ('07_junction_open_area_vector_review.pcd', open_points),
        ('08_building_marker_candidate_vector_review.pcd', building_candidate),
        ('09_building_marker_possible_vector_review.pcd', building_possible),
        ('10_scene_object_context_vector_review.pcd', scene_context),
    )
    all_points = []
    layer_counts = {}
    for name, points in layers:
        base.write_pcd(os.path.join(pcd_dir, name), points)
        all_points.extend(points)
        layer_counts[name] = len(points)
    # 这是仅供人工同时查看的便利副本；名称明确避免被阶段7误消费为单一“道路中心线”。
    combined_centerline_context = centerline_points + trajectory_reference_points
    combined_name = '02c_road_axis_with_trajectory_context_review.pcd'
    base.write_pcd(os.path.join(pcd_dir, combined_name), combined_centerline_context)
    layer_counts[combined_name] = len(combined_centerline_context)
    base.write_pcd(os.path.join(pcd_dir, 'stage06_vector_review_all.pcd'), all_points)
    contract = {
        'schema': SCHEMA + '/pcd-review-v1',
        'purpose': 'CloudCompare人工复核的阶段6矢量显示副本，不是新的观测证据。',
        'pcd_semantics': {
            'surface_points': '道路、确认人行道和开放区支持面点直接来自其冻结栅格/支持层位置；仅颜色和微小Z偏移用于显示。',
            'line_points': '中心线、边界和约束由阶段6矢量线等距0.10m采样；采样密度不代表观测次数。道路中轴由阶段3同横断面的左右边界距离计算，单侧/开放区轨迹另存为02b语境层。',
            'object_skeleton_points': '建筑/对象为阶段5可见拟合段的底边和高度线；不是建筑轮廓、外墙面或完整体块。',
            'boundary_z_note': '阶段4边界CSV没有端点Z，边界/约束显示线使用最近阶段3道路地面Z加显示偏移；不能据此验收路沿高度。',
        },
        'color_rgb': {
            '蓝色': '双边实测道路面', '黄绿色': '双边锚点连续桥接道路搜索面（inferred）',
            '淡紫色': '固定宽度道路搜索面（inferred）', '白色': '双边实测道路中轴',
            '黄色': '桥接道路中轴（inferred）', '灰色': '轨迹参考语境、未知道路方向或未知边界',
            '橙色': '路沿候选或建筑标识强候选（分别见文件名）', '浅橙色': '建筑标识可能候选',
            '青色': '确认人行道/路肩', '紫色': 'inferred道路边界约束',
            '洋红色': '路口/开放区候选支持', '绿色': '非建筑场景对象上下文',
        },
        'layer_point_counts': layer_counts,
        'all_layers_file': 'stage06_vector_review_all.pcd',
        'combined_centerline_context_file': combined_name,
        'combined_centerline_context_rule': '02c仅供CloudCompare便利查看：白/黄仍是道路中轴，灰色仍是轨迹参考语境；不得作为阶段7单一道路中心线输入。',
        'acceptance_rule': '必须结合各独立PCD及stage06_feature_audit.csv核验；不得仅凭合并颜色判断为observed。',
    }
    write_json(os.path.join(pcd_dir, 'stage06_pcd_review_contract.json'), contract)
    return layer_counts


def main():
    parser = argparse.ArgumentParser(description='阶段6：冻结证据到版本化矢量地图（不含渲染）')
    parser.add_argument('--run-dir', required=True, help='例如 Log/scene_evidence_20260814_01')
    parser.add_argument('--output', default=None, help='输出目录，默认 RUN_DIR/scene_pipeline_v4/stage06_vector_map_v1')
    args = parser.parse_args()
    run_dir = os.path.abspath(args.run_dir)
    v4 = os.path.join(run_dir, 'scene_pipeline_v4')
    output = os.path.abspath(args.output or os.path.join(v4, 'stage06_vector_map_v1'))
    if os.path.exists(output):
        fail('输出目录已存在，为保护既有结果拒绝覆盖：' + output)

    stage04_handoff = os.path.join(v4, 'stage04_final_handoff')
    stage05_handoff = os.path.join(v4, 'stage05_final_handoff')
    stage04_manifest = verify_frozen_handoff(stage04_handoff, 'stage04_freeze_manifest.json',
                                              '04_curb_sidewalk_extraction')
    stage05_manifest = verify_frozen_handoff(stage05_handoff, 'stage05_freeze_manifest.json',
                                              '05_visible_scene_objects')

    stage03 = os.path.join(v4, 'stage03_final_handoff')
    stage04 = stage04_manifest['accepted_baseline']['stage04_r7']
    stage042 = stage04_manifest['accepted_baseline']['stage042_r2']
    stage05 = stage05_manifest['accepted_baseline']['stage05_r3']
    paths = {
        'road_cells': os.path.join(stage03, 'evidence', 'road_surface_candidate_complete_cells.csv'),
        'cross_sections': os.path.join(stage03, 'evidence', 'road_cross_section_confidence.csv'),
        'boundaries': os.path.join(stage04, 'evidence', 'boundary_semantic_records.csv'),
        'sidewalk_records': os.path.join(stage04, 'evidence', 'sidewalk_surface_records.csv'),
        'sidewalk_pcd': os.path.join(stage04, 'evidence', 'sidewalk_confirmed_support.pcd'),
        'open_pcd': os.path.join(stage04, 'evidence', 'junction_or_open_area_candidate_support.pcd'),
        'constraints': os.path.join(stage042, 'evidence', 'inferred_road_edge_constraint_records.csv'),
        'objects': os.path.join(stage05, 'evidence', 'navigation_structure_r3_records.csv'),
    }
    road_rows = read_csv(paths['road_cells'], ('cell_ix', 'cell_iy', 'ground_z', 'cross_section_support_count',
                                                'road_candidate_primary_source', 'observed', 'width_source',
                                                'road_membership_confidence'))
    cross_rows = read_csv(paths['cross_sections'], ('sample_id', 'x', 'y', 'tangent_x', 'tangent_y', 'confidence',
                                                      'candidate_class', 'is_measured_true_width', 'local_width_m',
                                                      'bridge_kind', 'left_boundary_distance_m', 'right_boundary_distance_m'))
    boundary_rows = read_csv(paths['boundaries'], ('boundary_band_id', 'boundary_source', 'source_primitive_ids', 'side',
                                                    'start_x', 'start_y', 'end_x', 'end_y', 'continuous_length_m',
                                                    'relative_road_distance_m', 'local_height_delta_m', 'direction_angle_deg',
                                                    'hard_edge_support_fraction', 'road_support_fraction', 'semantic_class',
                                                    'classification_confidence', 'evidence_state', 'rejection_reason'))
    sidewalk_rows = read_csv(paths['sidewalk_records'], ('surface_id', 'classification', 'evidence_state'))
    constraint_rows = read_csv(paths['constraints'], ('boundary_band_id', 'side', 'start_x', 'start_y', 'end_x', 'end_y',
                                                       'candidate_status', 'evidence_state', 'constraint_confidence',
                                                       'continuous_length_m', 'relative_road_distance_m', 'direction_angle_deg',
                                                       'nearest_junction_distance_m', 'reason'))
    object_rows = read_csv(paths['objects'], ('record_id', 'r1_object_id', 'r1_object_type', 'r2_navigation_class',
                                               'r3_navigation_class', 'geometry_evidence_state', 'semantic_evidence_state',
                                               'completeness', 'center_x', 'center_y', 'base_z', 'top_z', 'tangent_x', 'tangent_y',
                                               'visible_length_m', 'plane_width_rms_m', 'visible_height_m', 'projection_coverage',
                                               'source_column_count', 'source_point_count', 'nearest_ground_distance_m',
                                               'base_above_ground_m', 'road_overlap_fraction', 'road_above_fraction', 'decision_reason'))

    os.makedirs(os.path.join(output, 'vectors'))
    os.makedirs(os.path.join(output, 'audit'))
    os.makedirs(os.path.join(output, 'pcd_review'))
    vector_dir = os.path.join(output, 'vectors')
    audit_dir = os.path.join(output, 'audit')
    pcd_dir = os.path.join(output, 'pcd_review')

    centerlines, profiles = cross_section_features(cross_rows)
    roads = road_surface_features(road_rows)
    curbs, unknown_boundaries = boundary_features(boundary_rows)
    sidewalks = sidewalk_footprint_features(paths['sidewalk_pcd'], sidewalk_rows)
    constraints, context_edges = constraint_features(constraint_rows)
    objects = object_features(object_rows)
    open_samples, open_support = open_area_features(cross_rows, paths['open_pcd'])

    write_geojson(os.path.join(vector_dir, 'road_centerline_segments.geojson'), centerlines,
                  'road_centerline_segments', '按阶段3横断面类别切段的道路中心线/方向候选。')
    write_geojson(os.path.join(vector_dir, 'road_surface_areas.geojson'), roads,
                  'road_surface_areas', '阶段3道路栅格的精确并集；推断搜索面不能作为最终道路。')
    write_width_profiles(os.path.join(vector_dir, 'road_width_profiles.csv'), profiles)
    write_geojson(os.path.join(vector_dir, 'curb_candidates.geojson'), curbs,
                  'curb_candidates', '阶段4确认语义为curb_candidate的边界；仍保留各自证据状态。')
    write_geojson(os.path.join(vector_dir, 'unknown_boundaries.geojson'), unknown_boundaries,
                  'unknown_boundaries', '阶段4未能安全分类的边界；不应渲染为路沿。')
    write_geojson(os.path.join(vector_dir, 'sidewalk_confirmed_areas.geojson'), sidewalks,
                  'sidewalk_confirmed_areas', '确认人行道/路肩稳定地面栅格并集。')
    write_geojson(os.path.join(vector_dir, 'inferred_road_edge_constraints.geojson'), constraints,
                  'inferred_road_edge_constraints', '阶段4.2唯一可用于低置信度道路延续的约束线。')
    write_geojson(os.path.join(vector_dir, 'road_edge_context_review.geojson'), context_edges,
                  'road_edge_context_review', '阶段4.2排除或人工复核边，不能作道路约束。')
    write_geojson(os.path.join(vector_dir, 'junction_or_open_area_candidates.geojson'), open_samples,
                  'junction_or_open_area_candidates', '低证据横断面；不得压成普通道路。')
    write_geojson(os.path.join(vector_dir, 'junction_or_open_area_support.geojson'), open_support,
                  'junction_or_open_area_support', '阶段4开放区候选支持栅格，不是最终道路面。')
    write_geojson(os.path.join(vector_dir, 'building_marker_candidates.geojson'), objects['building_marker_candidates'],
                  'building_marker_candidates', '可选导航建筑标识强候选；几何为可见拟合段，完整性均为partial。')
    write_geojson(os.path.join(vector_dir, 'building_marker_possible.geojson'), objects['building_marker_possible'],
                  'building_marker_possible', '可选导航建筑标识可能候选；需要在产品中弱化。')
    write_geojson(os.path.join(vector_dir, 'scene_object_context.geojson'), objects['scene_object_context'],
                  'scene_object_context', '植被、杆体和其他上下文，不得当作建筑块。')
    pcd_review_counts = write_pcd_review_layers(
        pcd_dir, road_rows, cross_rows, boundary_rows, paths['sidewalk_pcd'], constraint_rows, object_rows,
        paths['open_pcd'])

    audit_rows = []
    for layer_name, collection in (
            ('road_centerline_segments', centerlines), ('road_surface_areas', roads),
            ('curb_candidates', curbs), ('unknown_boundaries', unknown_boundaries),
            ('sidewalk_confirmed_areas', sidewalks), ('inferred_road_edge_constraints', constraints),
            ('road_edge_context_review', context_edges), ('junction_or_open_area_candidates', open_samples),
            ('junction_or_open_area_support', open_support), ('building_marker_candidates', objects['building_marker_candidates']),
            ('building_marker_possible', objects['building_marker_possible']), ('scene_object_context', objects['scene_object_context'])):
        append_audit(audit_rows, layer_name, collection)
    write_audit(os.path.join(audit_dir, 'stage06_feature_audit.csv'), audit_rows)

    input_validation = {
        'schema': SCHEMA,
        'stage04_freeze_verified': True,
        'stage05_freeze_verified': True,
        'stage04_manifest': stage04_handoff,
        'stage05_manifest': stage05_handoff,
        'consumed_paths': paths,
        'consumption_policy': [
            '只消费阶段4冻结包R7和阶段4.2 R2；R4.2只有inferred道路边界约束资格。',
            '只消费阶段5冻结包R3；R1/R2只通过R3记录链路审计，未混入归档迭代。',
            'PCD只用于其冻结图层已声明的空间支持几何；不从颜色反推语义。',
        ],
    }
    write_json(os.path.join(audit_dir, 'stage06_input_validation.json'), input_validation)
    by_layer = Counter(row['layer'] for row in audit_rows)
    report = {
        'schema': SCHEMA,
        'stage': '06_vectorization_and_regularization',
        'status': 'complete',
        'input_row_counts': {
            'road_cells': len(road_rows), 'cross_sections': len(cross_rows), 'boundary_records': len(boundary_rows),
            'sidewalk_surface_records': len(sidewalk_rows), 'road_edge_constraint_records': len(constraint_rows),
            'scene_object_records': len(object_rows),
        },
        'vector_feature_counts': dict(sorted(by_layer.items())),
        'pcd_review_point_counts': pcd_review_counts,
        'road_source_counts': dict(sorted(Counter(row['road_candidate_primary_source'] for row in road_rows).items())),
        'boundary_class_counts': dict(sorted(Counter(row['semantic_class'] for row in boundary_rows).items())),
        'object_class_counts': dict(sorted(Counter(row['r3_navigation_class'] for row in object_rows).items())),
        'rules_applied': [
            '道路中心线仅在相邻阶段3横断面、相同证据类别内切段；未跨类别或跨采样缺口连线。',
            '道路栅格面为原始0.2m格的精确并集；未施加平滑、平行或固定宽度补面。',
            '确认人行道面只来自阶段4确认支持层的稳定地面栅格；可能人行道不输出为面。',
            '建筑标识只表示阶段5的可见拟合段；不生成完整建筑块或建筑轮廓。',
        ],
        'known_limits': [
            '阶段3桥接和固定宽度候选仍是inferred搜索面；阶段7不能把它们当observed道路直接渲染。',
            '阶段4确认人行道审计CSV没有保存每个栅格到surface_id的一对一映射，因此矢量面按确认PCD集合回溯。',
            '阶段5冻结对象没有保存每个对象精确帧ID列表，只保留聚合支持量；阶段6不能伪造帧级来源。',
            'pcd_review中的边界/约束线是矢量显示副本；阶段4 CSV没有端点Z，显示高度不应用于路沿高差验收。',
        ],
    }
    write_json(os.path.join(audit_dir, 'stage06_vectorization_report.json'), report)
    write_json(os.path.join(output, 'stage06_complete.json'), {
        'schema': SCHEMA, 'stage': '06_vectorization_and_regularization', 'status': 'complete',
        'policy': '证据优先矢量化完成；阶段7只能消费本版本化矢量和其证据状态，不能反向补造几何。',
        'input_validation': 'audit/stage06_input_validation.json',
        'report': 'audit/stage06_vectorization_report.json',
        'pcd_review_contract': 'pcd_review/stage06_pcd_review_contract.json',
    })
    print('阶段6完成：输出=%s；矢量要素=%d' % (output, len(audit_rows)))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段6失败：%s' % error, file=sys.stderr)
        sys.exit(1)
