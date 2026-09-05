#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段7：仅用阶段6冻结矢量生成本地2.5D导航风格产品。

渲染是样式层，不是几何修复层。特别是：
* 只把双边实测道路画成正常道路；推断道路始终弱化且可关闭；
* 未知边界、路口/开放区和道路边约束不被画成确定道路或路沿；
* 建筑标识棱柱是可见立面线段的产品符号，仍是 partial / inferred，不是完整建筑。
"""

from __future__ import print_function

import argparse
import csv
import hashlib
import json
import math
import os
import sys
from collections import Counter


SCHEMA = 'fast_livo_scene_pipeline_stage07_navigation_2p5d/v1'


def fail(message):
    raise RuntimeError(message)


def read_json(path, label):
    try:
        with open(path, 'r') as handle:
            return json.load(handle)
    except Exception as error:
        fail('%s无法读取：%s' % (label, error))


def read_feature_collection(path, label):
    value = read_json(path, label)
    if value.get('type') != 'FeatureCollection' or not isinstance(value.get('features'), list):
        fail('%s不是FeatureCollection：%s' % (label, path))
    for item in value['features']:
        props = item.get('properties', {})
        for name in ('source_stage', 'geometry_evidence_state', 'semantic_evidence_state',
                     'evidence_state', 'render_policy'):
            if name not in props:
                fail('%s缺少渲染所需属性%s：%s' % (label, name, item.get('id')))
    return value['features']


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        while True:
            block = handle.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def fingerprint_file(path, label):
    if not os.path.isfile(path):
        fail('%s不存在或不是文件：%s' % (label, path))
    return {'path': os.path.abspath(path), 'bytes': os.path.getsize(path),
            'sha256': sha256_file(path)}


def read_localization_trajectory(path):
    """读取 FAST-LOCALIZATION 自动导出的地图系 IMU 轨迹，拒绝猜测字段或时序。"""
    source = fingerprint_file(path, '定位轨迹CSV')
    required = ('timestamp_sec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')
    points = []
    previous_timestamp = None
    try:
        with open(path, 'r') as handle:
            reader = csv.DictReader(handle)
            fieldnames = tuple(reader.fieldnames or ())
            missing = [name for name in required if name not in fieldnames]
            if missing:
                fail('定位轨迹CSV缺少字段：%s；要求FAST-LOCALIZATION自动导出的CSV' % ', '.join(missing))
            for row_number, row in enumerate(reader, 2):
                values = {}
                for name in required:
                    try:
                        values[name] = float(row[name])
                    except (TypeError, ValueError):
                        fail('定位轨迹CSV第%d行字段%s不是数值' % (row_number, name))
                    if not math.isfinite(values[name]):
                        fail('定位轨迹CSV第%d行字段%s不是有限数' % (row_number, name))
                if previous_timestamp is not None and values['timestamp_sec'] <= previous_timestamp:
                    fail('定位轨迹CSV时间戳必须严格递增；第%d行不满足' % row_number)
                previous_timestamp = values['timestamp_sec']
                quaternion_norm = math.sqrt(sum(values[name] * values[name]
                                                for name in ('qx', 'qy', 'qz', 'qw')))
                if quaternion_norm < 1e-6:
                    fail('定位轨迹CSV第%d行四元数退化' % row_number)
                for name in ('qx', 'qy', 'qz', 'qw'):
                    values[name] /= quaternion_norm
                points.append({'t': values['timestamp_sec'], 'x': values['x'], 'y': values['y'],
                               'z': values['z'], 'qx': values['qx'], 'qy': values['qy'],
                               'qz': values['qz'], 'qw': values['qw']})
    except OSError as error:
        fail('定位轨迹CSV无法读取：%s' % error)
    if len(points) < 2:
        fail('定位轨迹至少需要2个有效位姿点')
    return points, source


def map_provenance(map_dir):
    """记录 FAST-LOCALIZATION 载入地图所需的两个不可替代文件指纹。"""
    root = os.path.abspath(map_dir)
    if not os.path.isdir(root):
        fail('定位地图目录不存在：' + root)
    metadata = os.path.join(root, 'keyframes', 'metadata.yaml')
    optimized = os.path.join(root, 'loop_backend', 'optimized_keyframe_poses_imu.txt')
    fallback = os.path.join(root, 'keyframes', 'keyframe_poses_imu.txt')
    pose_file = optimized if os.path.isfile(optimized) else fallback
    return {'map_directory': root,
            'metadata': fingerprint_file(metadata, '定位地图metadata.yaml'),
            'imu_pose_file': fingerprint_file(pose_file, '定位地图IMU位姿文件'),
            'pose_source': 'optimized' if pose_file == optimized else 'unoptimized_fallback'}


def localization_overlay(path, map_dir, extent):
    points, source = read_localization_trajectory(path)
    provenance = map_provenance(map_dir)
    in_extent = sum(1 for point in points if extent[0] <= point['x'] <= extent[2] and
                    extent[1] <= point['y'] <= extent[3])
    if not in_extent:
        fail('定位轨迹与2.5D产品范围没有重叠；拒绝在未经确认的地图坐标系中叠加')
    overlay = {
        'kind': 'fast_localization_map_frame_imu_trajectory',
        'label': 'FAST-LOCALIZATION 定位算法输出（地图系 IMU 轨迹）',
        'color': '#e24848', 'pointCount': len(points),
        'startTimestampSec': points[0]['t'], 'endTimestampSec': points[-1]['t'],
        'points': points,
    }
    contract = {
        'enabled': True,
        'layer_label': overlay['label'],
        'coordinate_frame': 'FAST-LOCALIZATION map-frame / FAST-LIVO2 camera_init；调用者声明与本产品同一地图，脚本记录地图位姿/外参指纹但不能从历史CSV反向证明。',
        'source_trajectory_csv': source,
        'map_identity': provenance,
        'point_count': len(points), 'start_timestamp_sec': points[0]['t'],
        'end_timestamp_sec': points[-1]['t'], 'points_inside_product_extent': in_extent,
        'rendering': '红色轨迹线和带四元数航向的当前位姿标记；用于定位效果展示，不改变道路或场景证据状态。',
        'excluded_data': '该红色图层不混入RTK或定位误差指标；若产品另提供蓝色原始RTK图层，必须由raw_rtk_xy_overlay单独审计。',
    }
    return overlay, contract


def read_raw_rtk_xy_trajectory(path):
    """读取未筛选 RTK 的已投影地图 XY；绝不读取或传播任何 Z 相关字段。"""
    source = fingerprint_file(path, '未筛选RTK轨迹CSV')
    required = ('timestamp_sec', 'map_x', 'map_y')
    points = []
    previous_timestamp = None
    try:
        with open(path, 'r') as handle:
            reader = csv.DictReader(handle)
            fieldnames = tuple(reader.fieldnames or ())
            missing = [name for name in required if name not in fieldnames]
            if missing:
                fail('未筛选RTK轨迹CSV缺少字段：%s；要求含timestamp_sec,map_x,map_y的导出CSV' %
                     ', '.join(missing))
            for row_number, row in enumerate(reader, 2):
                values = {}
                for name in required:
                    try:
                        values[name] = float(row[name])
                    except (TypeError, ValueError):
                        fail('未筛选RTK轨迹CSV第%d行字段%s不是数值' % (row_number, name))
                    if not math.isfinite(values[name]):
                        fail('未筛选RTK轨迹CSV第%d行字段%s不是有限数' % (row_number, name))
                if previous_timestamp is not None and values['timestamp_sec'] <= previous_timestamp:
                    fail('未筛选RTK轨迹CSV时间戳必须严格递增；第%d行不满足' % row_number)
                previous_timestamp = values['timestamp_sec']
                points.append({'t': values['timestamp_sec'], 'x': values['map_x'], 'y': values['map_y']})
    except OSError as error:
        fail('未筛选RTK轨迹CSV无法读取：%s' % error)
    if len(points) < 2:
        fail('未筛选RTK轨迹至少需要2个有效XY点')
    return points, source


def raw_rtk_xy_overlay(path, localization_points, extent):
    """构造只用于平面可视对比的原始 RTK 蓝色图层，不产生精度评价。"""
    points, source = read_raw_rtk_xy_trajectory(path)
    in_extent = sum(1 for point in points if extent[0] <= point['x'] <= extent[2] and
                    extent[1] <= point['y'] <= extent[3])
    if not in_extent:
        fail('未筛选RTK轨迹与2.5D产品范围没有XY重叠；拒绝叠加')
    common_start = max(localization_points[0]['t'], points[0]['t'])
    common_end = min(localization_points[-1]['t'], points[-1]['t'])
    if common_end <= common_start:
        fail('红色定位轨迹与未筛选RTK轨迹没有共同时间段；拒绝伪造同步对比')
    localization_common_count = sum(1 for point in localization_points
                                    if common_start <= point['t'] <= common_end)
    overlay = {
        'kind': 'raw_rtk_map_xy_reference_trajectory',
        'label': '未筛选 RTK 参考轨迹（仅 XY，蓝）',
        'color': '#2878c8', 'pointCount': len(points),
        'startTimestampSec': points[0]['t'], 'endTimestampSec': points[-1]['t'],
        # points 仅有 t/x/y：不能让 Z 通过内嵌数据重新进入浏览器产品。
        'points': points,
    }
    contract = {
        'enabled': True,
        'layer_label': overlay['label'],
        'source_raw_rtk_csv': source,
        'coordinate_fields_used': ['timestamp_sec', 'map_x', 'map_y'],
        'z_handling': '仅读取map_x/map_y；altitude_m、east_m、north_m、up_m和map_z均不读取、不存储、不渲染。',
        'coordinate_frame': '调用者声明RTK导出已由WGS84/ENU转换到与红色定位轨迹及本产品相同的FAST-LIVO2局部map/camera_init坐标系；CSV本身不能反向证明地图身份。',
        'point_count': len(points), 'points_inside_product_extent': in_extent,
        'start_timestamp_sec': points[0]['t'], 'end_timestamp_sec': points[-1]['t'],
        'comparison_time_window': {
            'start_timestamp_sec': common_start, 'end_timestamp_sec': common_end,
            'localization_points_in_window': localization_common_count,
        },
        'rendering': '蓝色完整原始RTK XY线；时间滑条按红色定位时刻对RTK相邻XY点线性插值并显示蓝色同步点。',
        'limitations': '未筛选RTK可能含异常位置，且未施加GNSS天线到IMU水平杆臂修正；仅作XY平面参考对比，不是真值，不构成ATE/RMSE或绝对定位精度结论。',
    }
    return overlay, contract


def walk_coordinates(value, bounds):
    if isinstance(value, (list, tuple)) and value and isinstance(value[0], (int, float)):
        bounds[0] = min(bounds[0], float(value[0]))
        bounds[1] = min(bounds[1], float(value[1]))
        bounds[2] = max(bounds[2], float(value[0]))
        bounds[3] = max(bounds[3], float(value[1]))
    elif isinstance(value, (list, tuple)):
        for item in value:
            walk_coordinates(item, bounds)


def feature_bounds(features):
    bounds = [float('inf'), float('inf'), -float('inf'), -float('inf')]
    for item in features:
        walk_coordinates(item['geometry']['coordinates'], bounds)
    if not math.isfinite(bounds[0]):
        fail('没有可用于渲染的几何坐标')
    padding = 8.0
    return [bounds[0] - padding, bounds[1] - padding, bounds[2] + padding, bounds[3] + padding]


def keep_feature(item, fields):
    """Web产品只携带渲染与点击审计所需属性，避免复制全部阶段6审计负担。"""
    props = item['properties']
    result = {'id': item.get('id'), 'g': item['geometry'], 'p': {}}
    for field in fields:
        if field in props:
            result['p'][field] = props[field]
    return result


def midpoint(line):
    points = line['coordinates']
    first, last = points[0], points[-1]
    return ((float(first[0]) + float(last[0])) * 0.5, (float(first[1]) + float(last[1])) * 0.5)


def point_in_ring(point, ring):
    # 部分阶段6立面线保留了Z；平面包含关系只使用XY。
    x, y = float(point[0]), float(point[1])
    inside = False
    for first, second in zip(ring, ring[1:]):
        x1, y1 = float(first[0]), float(first[1])
        x2, y2 = float(second[0]), float(second[1])
        if (y1 > y) != (y2 > y):
            hit_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < hit_x:
                inside = not inside
    return inside


def point_in_geometry(point, geometry):
    polygons = [geometry['coordinates']] if geometry['type'] == 'Polygon' else geometry['coordinates']
    for polygon in polygons:
        if polygon and point_in_ring(point, polygon[0]) and not any(point_in_ring(point, hole) for hole in polygon[1:]):
            return True
    return False


def squared_distance(first, second):
    return ((float(first[0]) - float(second[0])) ** 2 +
            (float(first[1]) - float(second[1])) ** 2)


def point_to_segment_distance(point, first, second):
    """返回二维点到线段距离，供产品层排除贴道路的假建筑标识。"""
    dx = float(second[0]) - float(first[0])
    dy = float(second[1]) - float(first[1])
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return math.sqrt(squared_distance(point, first))
    ratio = ((float(point[0]) - float(first[0])) * dx +
             (float(point[1]) - float(first[1])) * dy) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    nearest = (float(first[0]) + ratio * dx, float(first[1]) + ratio * dy)
    return math.sqrt(squared_distance(point, nearest))


def orientation(first, second, third):
    return ((float(second[0]) - float(first[0])) * (float(third[1]) - float(first[1])) -
            (float(second[1]) - float(first[1])) * (float(third[0]) - float(first[0])))


def segments_intersect(first, second, third, fourth):
    """包含端点接触的二维线段相交判断。"""
    epsilon = 1e-10
    values = (orientation(first, second, third), orientation(first, second, fourth),
              orientation(third, fourth, first), orientation(third, fourth, second))
    if ((values[0] > epsilon and values[1] < -epsilon or values[0] < -epsilon and values[1] > epsilon) and
            (values[2] > epsilon and values[3] < -epsilon or values[2] < -epsilon and values[3] > epsilon)):
        return True
    return (abs(values[0]) <= epsilon and point_to_segment_distance(third, first, second) <= epsilon or
            abs(values[1]) <= epsilon and point_to_segment_distance(fourth, first, second) <= epsilon or
            abs(values[2]) <= epsilon and point_to_segment_distance(first, third, fourth) <= epsilon or
            abs(values[3]) <= epsilon and point_to_segment_distance(second, third, fourth) <= epsilon)


def segment_to_segment_distance(first, second, third, fourth):
    if segments_intersect(first, second, third, fourth):
        return 0.0
    return min(point_to_segment_distance(first, third, fourth), point_to_segment_distance(second, third, fourth),
               point_to_segment_distance(third, first, second), point_to_segment_distance(fourth, first, second))


def geometry_paths_for_distance(geometry):
    """把Polygon、MultiPolygon和LineString统一为连续折线集合。"""
    kind = geometry['type']
    if kind == 'Polygon':
        return geometry['coordinates']
    if kind == 'MultiPolygon':
        return [ring for polygon in geometry['coordinates'] for ring in polygon]
    if kind == 'LineString':
        return [geometry['coordinates']]
    return []


def line_to_features_distance(line, features):
    """计算立面线到道路、人行面或路沿的最短距离；面内和边界接触都视为零距离。"""
    best = float('inf')
    for item in features:
        geometry = item['geometry']
        if geometry['type'] in ('Polygon', 'MultiPolygon'):
            for point in line:
                if point_in_geometry(point, geometry):
                    return 0.0
        for target in geometry_paths_for_distance(geometry):
            for first, second in zip(line, line[1:]):
                for third, fourth in zip(target, target[1:]):
                    best = min(best, segment_to_segment_distance(first, second, third, fourth))
                    if best == 0.0:
                        return 0.0
    return best


def build_render_data(layers):
    roads = []
    suppressed_road_fragments = 0
    for item in layers['roads']:
        prop = item['properties']
        # 保留阶段6所有要素，但不让1--49格的噪声小岛污染导航产品画面。
        if int(prop.get('cell_count', 0)) < 50:
            suppressed_road_fragments += 1
            continue
        roads.append(keep_feature(item, ('road_candidate_primary_source', 'road_membership_confidence',
                                         'geometry_evidence_state', 'semantic_evidence_state',
                                         'evidence_state', 'render_policy', 'width_source')))
    sidewalks = [keep_feature(item, ('classification', 'geometry_evidence_state', 'semantic_evidence_state',
                                     'evidence_state', 'render_policy', 'cell_count')) for item in layers['sidewalks']]
    curbs = [keep_feature(item, ('boundary_band_id', 'semantic_class', 'classification_confidence',
                                 'geometry_evidence_state', 'semantic_evidence_state', 'evidence_state',
                                 'render_policy', 'continuous_length_m', 'local_height_delta_m')) for item in layers['curbs']]
    centerlines = [keep_feature(item, ('candidate_class', 'confidence', 'geometry_evidence_state',
                                       'semantic_evidence_state', 'evidence_state', 'render_policy',
                                       'axis_position_kind')) for item in layers['centerlines']]
    junctions = [keep_feature(item, ('candidate_class', 'geometry_evidence_state',
                                     'semantic_evidence_state', 'evidence_state', 'render_policy'))
                 for item in layers['junctions']]
    constraints = [keep_feature(item, ('boundary_band_id', 'constraint_confidence', 'geometry_evidence_state',
                                       'semantic_evidence_state', 'evidence_state', 'render_policy'))
                   for item in layers['constraints']]

    observed_road_geometries = [item['geometry'] for item in layers['roads']
                                if item['properties']['evidence_state'] == 'observed']
    # 用道路面、人行道/路肩面和路沿共同限定“不能是建筑”的导航走行带。
    navigation_surface_features = layers['roads'] + layers['sidewalks']
    navigation_edge_features = layers['curbs']
    building_clearance_m = 2.0
    suppressed_buildings = 0

    def objects(items, strict):
        result = []
        nonlocal_suppressed = 0
        for item in items:
            prop = item['properties']
            center = midpoint(item['geometry'])
            # 这是显示层保守收紧：只隐藏反证充分、短小或支持不足的建筑标识；不改变阶段5分类。
            if strict and (float(prop.get('visible_length_m', 0.0)) < 6.0 or
                           int(prop.get('source_point_count', 0)) < 200 or
                           float(prop.get('projection_coverage', 0.0)) < 0.85 or
                           float(prop.get('road_overlap_fraction', 0.0)) > 0.0 or
                           float(prop.get('road_above_fraction', 0.0)) > 0.0 or
                           any(point_in_geometry(center, geometry) for geometry in observed_road_geometries)):
                nonlocal_suppressed += 1
                continue
            line = item['geometry']['coordinates']
            surface_distance = line_to_features_distance(line, navigation_surface_features)
            edge_distance = line_to_features_distance(line, navigation_edge_features)
            navigation_clearance = min(surface_distance, edge_distance)
            if strict and navigation_clearance < building_clearance_m:
                nonlocal_suppressed += 1
                continue
            product_item = keep_feature(item, ('record_id', 'r3_navigation_class', 'geometry_evidence_state',
                                                'semantic_evidence_state', 'evidence_state', 'completeness',
                                                'base_z_m', 'top_z_m', 'visible_length_m', 'visible_height_m',
                                                'projection_coverage', 'source_point_count', 'render_policy',
                                                'decision_reason'))
            product_item['p']['product_navigation_clearance_m'] = round(navigation_clearance, 3)
            result.append(product_item)
        return result, nonlocal_suppressed

    building_strong, suppressed_buildings = objects(layers['building_strong'], True)
    building_possible, _ = objects(layers['building_possible'], True)

    # 上下文对象只保留中心点和类别，用导航符号表达，不复制原始立面线段以避免视觉噪声。
    context = []
    for item in layers['context']:
        prop = item['properties']
        x, y = midpoint(item['geometry'])
        context.append({'id': item.get('id'), 'x': x, 'y': y,
                        'class': prop.get('r3_navigation_class'), 'state': prop.get('evidence_state'),
                        'policy': prop.get('render_policy')})
    return {'roads': roads, 'sidewalks': sidewalks, 'curbs': curbs, 'centerlines': centerlines,
            'junctions': junctions, 'constraints': constraints,
            'buildingStrong': building_strong, 'buildingPossible': building_possible, 'context': context,
            'suppressedRoadFragments': suppressed_road_fragments,
            'suppressedBuildingMarkers': suppressed_buildings}


def geometry_paths(geometry, transform):
    """返回SVG path片段；原矢量顶点直接变换，不进行平滑或补点。"""
    kind = geometry['type']
    coordinates = geometry['coordinates']
    polygons = []
    if kind == 'Polygon':
        polygons = [coordinates]
    elif kind == 'MultiPolygon':
        polygons = coordinates
    else:
        return ''
    fragments = []
    for polygon in polygons:
        for ring in polygon:
            if not ring:
                continue
            first = transform(ring[0][0], ring[0][1])
            value = ['M %.2f %.2f' % first]
            for point in ring[1:]:
                x, y = transform(point[0], point[1])
                value.append('L %.2f %.2f' % (x, y))
            value.append('Z')
            fragments.append(' '.join(value))
    return ' '.join(fragments)


def svg_line(geometry, transform):
    coordinates = geometry['coordinates']
    if geometry['type'] == 'Point':
        x, y = transform(coordinates[0], coordinates[1])
        return 'M %.2f %.2f L %.2f %.2f' % (x - 0.1, y - 0.1, x + 0.1, y + 0.1)
    result = []
    for index, point in enumerate(coordinates):
        x, y = transform(point[0], point[1])
        result.append(('M' if index == 0 else 'L') + ' %.2f %.2f' % (x, y))
    return ' '.join(result)


def svg_localization_trajectory(points, transform):
    result = []
    for index, point in enumerate(points):
        x, y = transform(point['x'], point['y'])
        result.append(('M' if index == 0 else 'L') + ' %.2f %.2f' % (x, y))
    return ' '.join(result)


def svg_building(item, transform):
    """绘制可见立面锚定的导航体块；厚度只是符号，不能解释为完整建筑脚印。"""
    points = item['g']['coordinates']
    start, end = points[0], points[-1]
    x1, y1 = transform(start[0], start[1])
    x2, y2 = transform(end[0], end[1])
    dx, dy = x2 - x1, y2 - y1
    length = max(1.0, math.hypot(dx, dy))
    nx, ny = -dy / length, dx / length
    height = float(item['p'].get('visible_height_m', 3.0))
    depth = 7.0
    lift = min(30.0, max(9.0, height * 1.75))
    ox, oy = -lift * 0.46, -lift * 0.94
    a = (x1 + nx * depth * 0.5, y1 + ny * depth * 0.5)
    b = (x2 + nx * depth * 0.5, y2 + ny * depth * 0.5)
    c = (x2 - nx * depth * 0.5, y2 - ny * depth * 0.5)
    d = (x1 - nx * depth * 0.5, y1 - ny * depth * 0.5)
    top = [(a[0] + ox, a[1] + oy), (b[0] + ox, b[1] + oy), (c[0] + ox, c[1] + oy), (d[0] + ox, d[1] + oy)]
    def path(points):
        return 'M ' + ' L '.join('%.2f %.2f' % point for point in points) + ' Z'
    possible = item['p'].get('r3_navigation_class') != 'building_marker_candidate'
    front_color = '#c7b69e' if possible else '#527998'
    side_color = '#9b896f' if possible else '#31546f'
    top_color = '#dac9ad' if possible else '#6e93ad'
    return ('<path d="%s" fill="%s" opacity="%s"/><path d="%s" fill="%s" opacity="%s"/>'
            '<path d="%s" fill="%s" opacity="%s"/>' %
            (path([a, b, top[1], top[0]]), front_color, '0.58' if possible else '0.94',
             path([b, c, top[2], top[1]]), side_color, '0.52' if possible else '0.88',
             path(top), top_color, '0.66' if possible else '0.98'))


def render_svg(path, data, extent):
    width, height, margin = 1800, 1200, 72
    world_width, world_height = extent[2] - extent[0], extent[3] - extent[1]
    scale = min((width - margin * 2) / world_width, (height - margin * 2) / world_height)
    def transform(x, y):
        return (margin + (float(x) - extent[0]) * scale,
                height - margin - (float(y) - extent[1]) * scale)
    parts = ['''<svg xmlns="http://www.w3.org/2000/svg" width="1800" height="1200" viewBox="0 0 1800 1200">
<defs><pattern id="infer" width="12" height="12" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><rect width="12" height="12" fill="#aebac1" fill-opacity=".18"/><line x1="0" y1="0" x2="0" y2="12" stroke="#8e9da7" stroke-width="3" stroke-opacity=".35"/></pattern><filter id="shadow"><feGaussianBlur stdDeviation="2"/></filter></defs>
<rect width="1800" height="1200" fill="#e8eff1"/><text x="72" y="74" fill="#1d3041" font-size="30" font-family="Arial, sans-serif" font-weight="700">FAST-LIVO2 · 2.5D 导航地图</text><text x="72" y="106" fill="#627787" font-size="16" font-family="Arial, sans-serif">阶段 6 冻结矢量渲染 · 实测道路 / 可见场景标识</text>''']
    # 仅将有连续性桥接来源的推断道路放入静态总览；固定宽度回退候选仍保留在网页可选层。
    for item in data['roads']:
        if item['p']['road_candidate_primary_source'] == 'two_sided_continuity_bridge':
            parts.append('<path d="%s" fill="url(#infer)" fill-rule="evenodd" stroke="#aab7bd" stroke-width="1"/>' %
                         geometry_paths(item['g'], transform))
        elif item['p']['road_candidate_primary_source'] == 'fixed_width_fallback':
            parts.append('<path d="%s" fill="#f8f5ec" fill-rule="evenodd" stroke="#9aaab1" stroke-width="1.4" stroke-dasharray="5 4"/>' %
                         geometry_paths(item['g'], transform))
    for item in data['sidewalks']:
        parts.append('<path d="%s" fill="#d4dde1" fill-rule="evenodd" stroke="#b8c6cc" stroke-width="1"/>' %
                     geometry_paths(item['g'], transform))
    for item in data['roads']:
        if item['p']['evidence_state'] == 'observed':
            path_data = geometry_paths(item['g'], transform)
            parts.append('<path d="%s" fill="#b4c0c5" fill-rule="evenodd" transform="translate(0 3)" opacity=".55"/>' % path_data)
            parts.append('<path d="%s" fill="#f8f5ec" fill-rule="evenodd" stroke="#d8e0e1" stroke-width="1.5"/>' % path_data)
    for item in data['curbs']:
        color = '#8b9aa2' if item['p']['evidence_state'] == 'observed' else '#aeb8bd'
        parts.append('<path d="%s" fill="none" stroke="%s" stroke-width="2.4" stroke-linecap="round" opacity=".78"/>' %
                     (svg_line(item['g'], transform), color))
    for item in data['centerlines']:
        if item['p']['evidence_state'] == 'observed':
            parts.append('<path d="%s" fill="none" stroke="#e0a944" stroke-width="1.6" stroke-dasharray="8 7" stroke-linecap="round" opacity=".88"/>' % svg_line(item['g'], transform))
    for item in data['buildingStrong']:
        parts.append(svg_building(item, transform))
    raw_rtk = data.get('rawRtkTrajectory')
    if raw_rtk:
        rtk_path = svg_localization_trajectory(raw_rtk['points'], transform)
        # 蓝线先绘制，红色定位轨迹后绘制；重合处仍优先保留算法输出的可读性。
        parts.append('<path d="%s" fill="none" stroke="#f5f9ff" stroke-width="7" stroke-linecap="round" stroke-linejoin="round" opacity=".9"/>' % rtk_path)
        parts.append('<path d="%s" fill="none" stroke="#2878c8" stroke-width="3.2" stroke-linecap="round" stroke-linejoin="round"/>' % rtk_path)
        parts.append('<g transform="translate(910 990)" font-family="Arial, sans-serif" font-size="16" fill="#314a5b"><rect width="520" height="54" rx="12" fill="#f9fbfb" opacity=".92"/><line x1="18" y1="28" x2="58" y2="28" stroke="#2878c8" stroke-width="4" stroke-linecap="round"/><text x="72" y="34">未筛选 RTK 参考轨迹（蓝，仅 XY）</text></g>')
    overlay = data.get('localizationTrajectory')
    if overlay:
        trajectory_path = svg_localization_trajectory(overlay['points'], transform)
        first = transform(overlay['points'][0]['x'], overlay['points'][0]['y'])
        last = transform(overlay['points'][-1]['x'], overlay['points'][-1]['y'])
        # 定位点在静止或极低速时会出现毫米级往返。白色描边若沿用 SVG 默认的
        # miter join，会把这种极小的折角放大成虚假的长三角尖刺；圆角连接保留原始
        # 轨迹顶点，但不制造不存在的几何延伸。
        parts.append('<path d="%s" fill="none" stroke="#fff7f7" stroke-width="7" stroke-linecap="round" stroke-linejoin="round" opacity=".9"/>' % trajectory_path)
        parts.append('<path d="%s" fill="none" stroke="#e24848" stroke-width="3.5" stroke-linecap="round" stroke-linejoin="round"/>' % trajectory_path)
        parts.append('<circle cx="%.2f" cy="%.2f" r="6" fill="#ffffff" stroke="#e24848" stroke-width="3"/>' % first)
        parts.append('<circle cx="%.2f" cy="%.2f" r="6" fill="#e24848" stroke="#ffffff" stroke-width="2"/>' % last)
        parts.append('<g transform="translate(910 1070)" font-family="Arial, sans-serif" font-size="16" fill="#314a5b"><rect width="430" height="72" rx="12" fill="#f9fbfb" opacity=".92"/><line x1="18" y1="28" x2="58" y2="28" stroke="#e24848" stroke-width="4" stroke-linecap="round"/><text x="72" y="34">定位算法输出轨迹（红）</text></g>')
    parts.append('''<g transform="translate(72 1070)" font-family="Arial, sans-serif" font-size="16" fill="#314a5b"><rect width="810" height="72" rx="12" fill="#f9fbfb" opacity=".92"/><rect x="18" y="20" width="28" height="16" rx="4" fill="#f8f5ec" stroke="#d8e0e1"/><text x="55" y="34">双边实测道路</text><rect x="190" y="20" width="28" height="16" fill="url(#infer)"/><text x="227" y="34">推断道路候选</text><rect x="390" y="20" width="28" height="16" fill="#d4dde1"/><text x="427" y="34">确认人行道</text><rect x="565" y="20" width="28" height="16" fill="#38556f"/><text x="602" y="34">partial 建筑标识</text></g><text x="1728" y="1140" text-anchor="end" fill="#647985" font-size="14" font-family="Arial, sans-serif">不补造道路、边界或建筑轮廓</text></svg>''')
    with open(path, 'w') as handle:
        handle.write('\n'.join(parts))


HTML_TEMPLATE = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>FAST-LIVO2 · 2.5D 导航地图</title><style>
:root{--ink:#162b3a;--muted:#708391;--panel:#fbfcfc;--line:#d7e1e4;--accent:#2f6d86}*{box-sizing:border-box}body{margin:0;background:#e8eff1;color:var(--ink);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","Microsoft YaHei",sans-serif;overflow:hidden}#app{height:100vh;display:grid;grid-template-columns:320px 1fr}.side{background:rgba(251,252,252,.96);border-right:1px solid var(--line);padding:24px 20px;overflow:auto;z-index:2;box-shadow:8px 0 28px rgba(25,55,70,.08)}h1{font-size:19px;margin:0 0 5px;letter-spacing:.2px}.sub{font-size:12px;color:var(--muted);line-height:1.55;margin:0 0 20px}.badge{display:inline-block;background:#e5f0f2;color:#286077;border-radius:999px;font-size:11px;padding:4px 8px;margin-bottom:14px}.group{border-top:1px solid var(--line);padding:15px 0 4px}.group h2{font-size:12px;letter-spacing:.8px;text-transform:uppercase;color:#6d808d;margin:0 0 9px}.toggle{display:flex;align-items:center;gap:9px;padding:8px 0;font-size:13px;cursor:pointer}.toggle input{accent-color:#2d718b;width:15px;height:15px}.range{display:block;font-size:12px;color:#506875;padding:7px 0}.range input{display:block;width:100%;accent-color:#2d718b;margin:6px 0}.dot{width:11px;height:11px;border-radius:3px;display:inline-block}.hint{font-size:12px;color:#607584;line-height:1.55;margin:12px 0}.actions{display:flex;gap:8px;margin:18px 0 4px}button{appearance:none;border:1px solid #bdcdd3;background:#fff;border-radius:8px;padding:8px 10px;color:#24485a;font-weight:600;cursor:pointer;font-size:12px}button:hover{background:#edf5f6}.map{position:relative;min-width:0}.top{position:absolute;top:18px;left:22px;z-index:1;background:rgba(248,251,251,.9);border:1px solid #d3e0e3;border-radius:11px;padding:10px 13px;box-shadow:0 8px 20px rgba(35,63,77,.1);font-size:12px;color:#536b78}.top b{color:#1d475b}.legend{position:absolute;right:20px;bottom:20px;z-index:1;background:rgba(251,252,252,.94);border:1px solid #d5e1e3;border-radius:11px;padding:11px 14px;box-shadow:0 8px 20px rgba(35,63,77,.1);font-size:12px;line-height:1.8;color:#546b78}.legend i{display:inline-block;width:13px;height:9px;border-radius:2px;margin-right:6px}.tip{position:absolute;display:none;max-width:300px;background:#173042;color:#eff7f8;border-radius:10px;padding:11px 13px;font-size:12px;line-height:1.55;pointer-events:none;box-shadow:0 10px 24px rgba(0,0,0,.25);z-index:3}canvas{width:100%;height:100%;display:block;cursor:grab}.footer{font-size:11px;color:#80929b;margin-top:15px;line-height:1.5}@media(max-width:850px){body{overflow:auto}#app{height:auto;min-height:100vh;grid-template-columns:1fr;grid-template-rows:auto 72vh}.side{border-right:0;border-bottom:1px solid var(--line)}}
</style></head><body><div id="app"><aside class="side"><div class="badge">阶段 6 冻结矢量 · 阶段 7 产品视图</div><h1>2.5D 导航地图</h1><p class="sub">视觉样式仅服务阅读。所有道路、边界和对象位置仍来自冻结矢量，不以渲染补造事实。</p><div class="group"><h2>道路与边界</h2><label class="toggle"><input id="roadObserved" type="checkbox" checked><i class="dot" style="background:#f8f5ec;border:1px solid #d8e0e1"></i>双边实测道路</label><label class="toggle"><input id="bridgeRoad" type="checkbox" checked><i class="dot" style="background:#b9c7ca"></i>桥接连续道路（推断）</label><label class="toggle"><input id="fallbackRoad" type="checkbox"><i class="dot" style="background:#d4dcde"></i>固定宽度连续候选</label><label class="toggle"><input id="centerObserved" type="checkbox" checked><i class="dot" style="background:#e0a944"></i>实测道路中轴</label><label class="toggle"><input id="centerInferred" type="checkbox"><i class="dot" style="background:#e9c76e"></i>桥接道路中轴</label><label class="toggle"><input id="sidewalks" type="checkbox" checked><i class="dot" style="background:#d4dde1"></i>确认人行道 / 路肩</label><label class="toggle"><input id="curbs" type="checkbox" checked><i class="dot" style="background:#8b9aa2"></i>路沿候选</label><label class="toggle"><input id="junctions" type="checkbox" checked><i class="dot" style="background:#d889aa"></i>待确认开放区 / 路口</label></div><div class="group"><h2>场景标识</h2><label class="toggle"><input id="buildStrong" type="checkbox" checked><i class="dot" style="background:#456987"></i>优选可见立面体块</label><label class="toggle"><input id="buildPossible" type="checkbox"><i class="dot" style="background:#b9a38b"></i>可能立面体块</label><label class="toggle"><input id="context" type="checkbox"><i class="dot" style="background:#74a46f"></i>植被 / 杆体上下文</label><label class="toggle"><input id="constraints" type="checkbox"><i class="dot" style="background:#a16bd1"></i>推断道路边约束（复核）</label></div><div class="actions"><button id="reset">重置默认斜俯视</button><button id="png">导出 PNG</button></div><p class="hint">拖拽旋转 · 滚轮缩放 · 双击重置；按住 Shift 拖拽可平移。点击建筑体块可查看来源状态。小型道路碎片和低支持建筑仅从产品视图隐藏，不改变阶段 6 事实。</p><p class="footer">建筑体块严格锚定可见立面线段；厚度是导航符号，完整性仍为 partial，不代表真实完整建筑轮廓。</p></aside><main class="map"><div class="top"><b id="title">场景总览</b><span id="status"></span></div><canvas id="map"></canvas><div class="legend"><div><i style="background:#f8f5ec;border:1px solid #d8e0e1"></i>实测道路</div><div><i style="background:#b9c7ca"></i>桥接连续候选</div><div><i style="background:#456987"></i>partial 可见立面体块</div></div><div class="tip" id="tip"></div></main></div><script>const DATA=__MAP_DATA__;
const canvas=document.getElementById('map'),ctx=canvas.getContext('2d'),tip=document.getElementById('tip');let dpr=1,view={s:1,cx:0,cy:0,px:0,py:0,rot:-.56,cameraPitch:.79,pitch:Math.sin(.79)},drag=null;const state={},E=DATA.extent;
function syncToggle(input){state[input.id]=input.checked;draw()}
document.querySelectorAll('input[type=checkbox]').forEach(x=>{state[x.id]=x.checked;x.addEventListener('change',()=>syncToggle(x));x.addEventListener('input',()=>syncToggle(x));});
// 固定宽度候选默认展示，但始终以虚线候选边界与实测道路区分。
document.getElementById('fallbackRoad').checked=true;state.fallbackRoad=true;
function reset(){view.rot=-.56;view.cameraPitch=.79;view.pitch=Math.sin(view.cameraPitch);fit()}
function resize(){dpr=window.devicePixelRatio||1;const r=canvas.getBoundingClientRect();canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);fit()}
function fit(){const r=canvas.getBoundingClientRect(),pad=46,w=E[2]-E[0],h=E[3]-E[1];view.s=Math.min((r.width-pad*2)/w,(r.height-pad*2)/(h*view.pitch));view.cx=(E[0]+E[2])/2;view.cy=(E[1]+E[3])/2;view.px=r.width/2;view.py=r.height/2;draw()}
function xy(p){const dx=p[0]-view.cx,dy=p[1]-view.cy,c=Math.cos(view.rot),s=Math.sin(view.rot),rx=dx*c-dy*s,ry=dx*s+dy*c;return [rx*view.s+view.px,-ry*view.s*view.pitch+view.py]}
function path(g){const polys=g.type==='Polygon'?[g.coordinates]:g.coordinates;ctx.beginPath();for(const polygon of polys){for(const ring of polygon){for(let i=0;i<ring.length;i++){const q=xy(ring[i]);if(i){ctx.lineTo(q[0],q[1])}else{ctx.moveTo(q[0],q[1])}}}}}
function poly(g,fill,stroke,w,shift){const fallback=fill==='rgba(212,220,222,.24)',junction=fill==='rgba(216,137,170,.13)';if(fallback){fill='#f8f5ec';stroke='#9aaab1';w=1.35}if(junction){fill='rgba(248,245,236,.82)';stroke='#bdc8ca';w=1.5}ctx.save();if(shift)ctx.translate(shift[0],shift[1]);path(g);ctx.fillStyle=fill;ctx.fill('evenodd');if(stroke){ctx.strokeStyle=stroke;ctx.lineWidth=w;if(fallback||junction)ctx.setLineDash(fallback?[5,4]:[8,5]);ctx.stroke()}ctx.restore()}
function line(g,color,w,dash,alpha){const c=g.coordinates;ctx.save();ctx.beginPath();c.forEach((p,i)=>{const q=xy(p);i?ctx.lineTo(q[0],q[1]):ctx.moveTo(q[0],q[1])});ctx.strokeStyle=color;ctx.lineWidth=w;ctx.lineCap='round';ctx.lineJoin='round';ctx.globalAlpha=alpha===undefined?1:alpha;ctx.setLineDash(dash||[]);ctx.stroke();ctx.restore()}
function building(o,possible){const c=o.g.coordinates,a=xy(c[0]),b=xy(c[c.length-1]),dx=b[0]-a[0],dy=b[1]-a[1],L=Math.max(1,Math.hypot(dx,dy)),nx=-dy/L,ny=dx/L,depth=Math.max(5,view.s*1.55),h=Math.min(32,Math.max(9,(o.p.visible_height_m||3)*view.s*.48)),ox=-h*.46,oy=-h*.94;const A=[a[0]+nx*depth*.5,a[1]+ny*depth*.5],B=[b[0]+nx*depth*.5,b[1]+ny*depth*.5],C=[b[0]-nx*depth*.5,b[1]-ny*depth*.5],D=[a[0]-nx*depth*.5,a[1]-ny*depth*.5],T=[[A[0]+ox,A[1]+oy],[B[0]+ox,B[1]+oy],[C[0]+ox,C[1]+oy],[D[0]+ox,D[1]+oy]];function shape(ps,col,alpha){ctx.save();ctx.beginPath();ps.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle=col;ctx.globalAlpha=alpha;ctx.fill();ctx.restore()}const front=possible?'#c7b69e':'#527998',side=possible?'#9b896f':'#31546f',roof=possible?'#dac9ad':'#6e93ad';shape([A,B,T[1],T[0]],front,possible ? .58 : .94);shape([B,C,T[2],T[1]],side,possible ? .52 : .88);shape(T,roof,possible ? .66 : .98);return {x:(a[0]+b[0])/2,y:(a[1]+b[1])/2,o:o}}
let hits=[];function draw(){const r=canvas.getBoundingClientRect();ctx.clearRect(0,0,r.width,r.height);ctx.fillStyle='#e8eff1';ctx.fillRect(0,0,r.width,r.height);if(state.bridgeRoad)DATA.roads.filter(x=>x.p.road_candidate_primary_source==='two_sided_continuity_bridge').forEach(x=>poly(x.g,'rgba(185,199,202,.52)','#9eafb7',1));if(state.fallbackRoad)DATA.roads.filter(x=>x.p.road_candidate_primary_source==='fixed_width_fallback').forEach(x=>poly(x.g,'rgba(212,220,222,.24)','#b7c4c8',1));if(state.sidewalks)DATA.sidewalks.forEach(x=>poly(x.g,'#d4dde1','#b8c6cc',1));if(state.roadObserved)DATA.roads.filter(x=>x.p.evidence_state==='observed').forEach(x=>{poly(x.g,'rgba(135,154,163,.35)',null,0,[0,3]);poly(x.g,'#f8f5ec','#d8e0e1',1.2)});if(state.junctions)DATA.junctions.forEach(x=>poly(x.g,'rgba(216,137,170,.13)','#c7749a',1));if(state.curbs)DATA.curbs.forEach(x=>line(x.g,x.p.evidence_state==='observed'?'#8798a1':'#acb7bc',2.1,[],.78));if(state.centerObserved)DATA.centerlines.filter(x=>x.p.evidence_state==='observed').forEach(x=>line(x.g,'#e0a944',1.5,[8,7],.9));if(state.centerInferred)DATA.centerlines.filter(x=>x.p.evidence_state==='inferred').forEach(x=>line(x.g,'#e5c668',1.2,[4,6],.7));if(state.constraints)DATA.constraints.forEach(x=>line(x.g,'#9e67d1',1.4,[3,5],.75));if(state.context){ctx.fillStyle='rgba(84,148,91,.48)';DATA.context.forEach(x=>{const p=xy([x.x,x.y]);ctx.beginPath();ctx.arc(p[0],p[1],Math.max(2,view.s*.55),0,Math.PI*2);ctx.fill()})}hits=[];if(state.buildStrong)DATA.buildingStrong.forEach(x=>hits.push(building(x,false)));if(state.buildPossible)DATA.buildingPossible.forEach(x=>hits.push(building(x,true)));document.getElementById('status').textContent=' · 实测道路 '+DATA.counts.observedRoad+' · 优选建筑 '+DATA.counts.buildingStrong}
function eventPoint(e){const r=canvas.getBoundingClientRect();return [e.clientX-r.left,e.clientY-r.top]}
canvas.addEventListener('wheel',e=>{e.preventDefault();view.s=Math.max(.4,Math.min(22,view.s*(e.deltaY<0?1.18:.85)));draw()},{passive:false});canvas.addEventListener('pointerdown',e=>{if(e.button!==0)return;drag={p:eventPoint(e),rot:view.rot,cameraPitch:view.cameraPitch,px:view.px,py:view.py,pan:e.shiftKey};canvas.setPointerCapture(e.pointerId);canvas.style.cursor='grabbing'});canvas.addEventListener('pointermove',e=>{const p=eventPoint(e);if(drag){const dx=p[0]-drag.p[0],dy=p[1]-drag.p[1];if(drag.pan){view.px=drag.px+dx;view.py=drag.py+dy}else{view.rot=drag.rot+dx*.008;view.cameraPitch=Math.max(.22,Math.min(1.42,drag.cameraPitch-dy*.008));view.pitch=Math.sin(view.cameraPitch)}draw();return}let best=null,bd=18;hits.forEach(h=>{const z=Math.hypot(p[0]-h.x,p[1]-h.y);if(z<bd){bd=z;best=h}});if(best){const q=best.o.p;tip.style.display='block';tip.style.left=(p[0]+16)+'px';tip.style.top=(p[1]+16)+'px';tip.innerHTML='<b>'+q.r3_navigation_class+'</b><br>语义：'+q.semantic_evidence_state+' · 完整性：'+q.completeness+'<br>可见高度：'+Number(q.visible_height_m||0).toFixed(1)+' m<br>来源对象：'+q.record_id}else tip.style.display='none'});canvas.addEventListener('pointerup',()=>{drag=null;canvas.style.cursor='grab'});canvas.addEventListener('pointercancel',()=>{drag=null;canvas.style.cursor='grab'});canvas.addEventListener('dblclick',reset);document.getElementById('reset').onclick=reset;document.getElementById('png').onclick=()=>{const a=document.createElement('a');a.download='stage07_navigation_2p5d.png';a.href=canvas.toDataURL('image/png');a.click()};window.addEventListener('resize',resize);resize();</script></body></html>'''


LOCALIZATION_OVERLAY_TEMPLATE = r'''<script>
(() => {
  const trajectory = DATA.localizationTrajectory;
  const rawRtk = DATA.rawRtkTrajectory;
  if (!trajectory || !Array.isArray(trajectory.points) || trajectory.points.length < 2) return;
  const hasRawRtk = rawRtk && Array.isArray(rawRtk.points) && rawRtk.points.length >= 2;
  let firstIndex = 0, lastIndex = trajectory.points.length - 1;
  if (hasRawRtk) {
    while (firstIndex < trajectory.points.length && trajectory.points[firstIndex].t < rawRtk.points[0].t) firstIndex += 1;
    while (lastIndex >= 0 && trajectory.points[lastIndex].t > rawRtk.points[rawRtk.points.length - 1].t) lastIndex -= 1;
    if (firstIndex > lastIndex) return;
  }
  const actions = document.querySelector('.actions');
  const rawToggle = hasRawRtk ? '<label class="toggle"><input id="rawRtkTrack" type="checkbox" checked><i class="dot" style="background:#2878c8"></i>未筛选 RTK 参考（蓝，仅 XY）</label>' : '';
  const comparisonHint = hasRawRtk ? '蓝线仅读取 map_x/map_y，与红线按共同时间段同步；未筛选 RTK 仅作平面参考，不是真值或精度结论。' : '红线是全局初始化成功后记录的地图系 IMU 定位轨迹；三角标记按 CSV 四元数显示当前航向。';
  actions.insertAdjacentHTML('beforebegin', '<div class="group"><h2>定位效果展示</h2><label class="toggle"><input id="localizationTrack" type="checkbox" checked><i class="dot" style="background:#e24848"></i>FAST-LOCALIZATION 定位算法输出（红）</label>' + rawToggle + '<label class="range">定位时刻 <input id="localizationTime" type="range" min="' + firstIndex + '" max="' + lastIndex + '" value="' + firstIndex + '" step="1"><span id="localizationStamp"></span></label><div class="actions"><button id="localizationPlay">播放</button></div><p class="hint">' + comparisonHint + '</p></div>');
  document.querySelector('.legend').insertAdjacentHTML('beforeend', '<div><i style="background:#e24848"></i>定位算法输出轨迹</div>' + (hasRawRtk ? '<div><i style="background:#2878c8"></i>未筛选 RTK 参考（仅 XY）</div>' : ''));
  const checkbox = document.getElementById('localizationTrack');
  const rawCheckbox = document.getElementById('rawRtkTrack');
  const slider = document.getElementById('localizationTime');
  const stamp = document.getElementById('localizationStamp');
  const play = document.getElementById('localizationPlay');
  let index = firstIndex, timer = null;
  state.localizationTrack = true;
  state.rawRtkTrack = hasRawRtk;
  function updateStamp() { const point = trajectory.points[index]; slider.value = String(index); stamp.textContent = '第 ' + (index + 1) + ' / ' + trajectory.points.length + ' 点 · ' + point.t.toFixed(3) + (hasRawRtk ? ' · 红蓝共同时间段' : ''); }
  function drawMarker(point) {
    const origin = xy([point.x, point.y]);
    const yaw = Math.atan2(2 * (point.qw * point.qz + point.qx * point.qy), 1 - 2 * (point.qy * point.qy + point.qz * point.qz));
    const forward = xy([point.x + Math.cos(yaw), point.y + Math.sin(yaw)]);
    const angle = Math.atan2(forward[1] - origin[1], forward[0] - origin[0]);
    ctx.save(); ctx.translate(origin[0], origin[1]); ctx.rotate(angle); ctx.fillStyle = '#e24848'; ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(11, 0); ctx.lineTo(-8, 7); ctx.lineTo(-4, 0); ctx.lineTo(-8, -7); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
  }
  function rawRtkAt(timestamp) {
    if (!hasRawRtk || timestamp < rawRtk.points[0].t || timestamp > rawRtk.points[rawRtk.points.length - 1].t) return null;
    let low = 0, high = rawRtk.points.length - 1;
    while (low < high) { const middle = Math.floor((low + high) / 2); if (rawRtk.points[middle].t < timestamp) low = middle + 1; else high = middle; }
    const after = rawRtk.points[low];
    if (after.t === timestamp || low === 0) return [after.x, after.y];
    const before = rawRtk.points[low - 1], ratio = (timestamp - before.t) / (after.t - before.t);
    return [before.x + ratio * (after.x - before.x), before.y + ratio * (after.y - before.y)];
  }
  function drawRawRtk() {
    if (!hasRawRtk || !state.rawRtkTrack) return;
    const coordinates = rawRtk.points.map(point => [point.x, point.y]);
    line({coordinates: coordinates}, '#f5f9ff', 7, [], .92);
    line({coordinates: coordinates}, '#2878c8', 3.1, [], .96);
    const point = rawRtkAt(trajectory.points[index].t);
    if (point) { const projected = xy(point); ctx.save(); ctx.beginPath(); ctx.arc(projected[0], projected[1], 6, 0, Math.PI * 2); ctx.fillStyle = '#2878c8'; ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2; ctx.fill(); ctx.stroke(); ctx.restore(); }
  }
  function drawTrajectory() {
    drawRawRtk();
    if (!state.localizationTrack) return;
    const coordinates = trajectory.points.map(point => [point.x, point.y]);
    line({coordinates: coordinates}, '#fff7f7', 7, [], .92);
    line({coordinates: coordinates}, '#e24848', 3.2, [], .96);
    drawMarker(trajectory.points[index]);
  }
  const baseDraw = draw;
  draw = function() { baseDraw(); drawTrajectory(); };
  checkbox.addEventListener('change', () => { state.localizationTrack = checkbox.checked; draw(); });
  if (rawCheckbox) rawCheckbox.addEventListener('change', () => { state.rawRtkTrack = rawCheckbox.checked; draw(); });
  slider.addEventListener('input', () => { index = Number(slider.value); updateStamp(); draw(); });
  play.addEventListener('click', () => {
    if (timer) { clearInterval(timer); timer = null; play.textContent = '播放'; return; }
    play.textContent = '暂停'; timer = setInterval(() => { index = index >= lastIndex ? firstIndex : index + 1; updateStamp(); draw(); }, 120);
  });
  updateStamp(); draw();
})();
</script>'''


def render_html(path, data):
    encoded = json.dumps(data, ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/')
    html = HTML_TEMPLATE.replace('__MAP_DATA__', encoded)
    # 产品页保持地图操作与图层控制；删除不参与操作的状态摘要和说明性文案。
    for fragment in (
            '<p class="sub">视觉样式仅服务阅读。所有道路、边界和对象位置仍来自冻结矢量，不以渲染补造事实。</p>',
            '<p class="hint">拖拽旋转 · 滚轮缩放 · 双击重置；按住 Shift 拖拽可平移。点击建筑体块可查看来源状态。小型道路碎片和低支持建筑仅从产品视图隐藏，不改变阶段 6 事实。</p>',
            '<p class="footer">建筑体块严格锚定可见立面线段；厚度是导航符号，完整性仍为 partial，不代表真实完整建筑轮廓。</p>',
            '<div class="top"><b id="title">场景总览</b><span id="status"></span></div>',
    ):
        if fragment not in html:
            fail('阶段7页面模板缺少预期的可移除内容')
        html = html.replace(fragment, '')
    old_pointerdown = "canvas.addEventListener('pointerdown',e=>{if(e.button!==0)return;drag={p:eventPoint(e),rot:view.rot,cameraPitch:view.cameraPitch,px:view.px,py:view.py,pan:e.shiftKey};canvas.setPointerCapture(e.pointerId);canvas.style.cursor='grabbing'});"
    new_pointerdown = "canvas.addEventListener('contextmenu',e=>e.preventDefault());canvas.addEventListener('pointerdown',e=>{if(e.button!==0&&e.button!==2)return;e.preventDefault();drag={p:eventPoint(e),rot:view.rot,cameraPitch:view.cameraPitch,px:view.px,py:view.py,pan:e.button===2};canvas.setPointerCapture(e.pointerId);canvas.style.cursor='grabbing'});"
    if old_pointerdown not in html:
        fail('阶段7页面模板缺少预期的平移交互')
    html = html.replace(old_pointerdown, new_pointerdown)
    old_pointermove = "canvas.addEventListener('pointermove',e=>{const p=eventPoint(e);if(drag){const dx=p[0]-drag.p[0],dy=p[1]-drag.p[1];"
    new_pointermove = "canvas.addEventListener('pointermove',e=>{const p=eventPoint(e);if(drag){e.preventDefault();const dx=p[0]-drag.p[0],dy=p[1]-drag.p[1];"
    if old_pointermove not in html:
        fail('阶段7页面模板缺少预期的拖动交互')
    html = html.replace(old_pointermove, new_pointermove)
    canvas_style = 'canvas{width:100%;height:100%;display:block;cursor:grab}'
    protected_canvas_style = 'canvas{width:100%;height:100%;display:block;cursor:grab;touch-action:none;user-select:none;-webkit-user-select:none}'
    if canvas_style not in html:
        fail('阶段7页面模板缺少预期的画布样式')
    html = html.replace(canvas_style, protected_canvas_style)
    status_update = ";document.getElementById('status').textContent=' · 实测道路 '+DATA.counts.observedRoad+' · 优选建筑 '+DATA.counts.buildingStrong"
    if status_update not in html:
        fail('阶段7页面模板缺少预期的状态摘要')
    html = html.replace(status_update, '')
    if data.get('localizationTrajectory'):
        html = html.replace('</body>', LOCALIZATION_OVERLAY_TEMPLATE + '</body>')
    with open(path, 'w') as handle:
        handle.write(html)


def main():
    parser = argparse.ArgumentParser(description='阶段7：阶段6冻结矢量到2.5D导航风格产品')
    parser.add_argument('--run-dir', required=True, help='例如 Log/scene_evidence_20260814_01')
    parser.add_argument('--output', default=None, help='默认 RUN_DIR/scene_pipeline_v4/stage07_navigation_2p5d_v4')
    parser.add_argument('--localization-trajectory', default=None,
                        help='FAST-LOCALIZATION自动导出的红色定位CSV；提供后生成可播放定位覆盖层')
    parser.add_argument('--raw-rtk-trajectory', default=None,
                        help='未筛选RTK导出的蓝色参考CSV；只读取timestamp_sec,map_x,map_y，必须与红色定位轨迹配对')
    parser.add_argument('--trajectory-map-dir', default=None,
                        help='该定位运行实际使用的FAST-LIVO2地图目录；与--localization-trajectory配对必填')
    args = parser.parse_args()
    if bool(args.localization_trajectory) != bool(args.trajectory_map_dir):
        fail('--localization-trajectory与--trajectory-map-dir必须同时提供，避免混用不同地图')
    if args.raw_rtk_trajectory and not args.localization_trajectory:
        fail('--raw-rtk-trajectory只能与--localization-trajectory共同使用，避免单独展示而暗示RTK为地图真值')
    run_dir = os.path.abspath(args.run_dir)
    v4 = os.path.join(run_dir, 'scene_pipeline_v4')
    output = os.path.abspath(args.output or os.path.join(v4, 'stage07_navigation_2p5d_v4'))
    if os.path.exists(output):
        fail('输出目录已存在，为保护既有产品拒绝覆盖：' + output)
    handoff = os.path.join(v4, 'stage06_final_handoff')
    manifest = read_json(os.path.join(handoff, 'stage06_freeze_manifest.json'), '阶段6冻结清单')
    if manifest.get('status') != 'frozen' or manifest.get('stage') != '06_vectorization_and_regularization':
        fail('阶段6未冻结，阶段7拒绝渲染')
    stage06 = manifest.get('accepted_baseline', {}).get('stage06_v1')
    if not stage06 or not os.path.isdir(stage06):
        fail('阶段6冻结清单没有有效的V1矢量基线')
    vectors = os.path.join(stage06, 'vectors')
    paths = {
        'roads': 'road_surface_areas.geojson', 'centerlines': 'road_centerline_segments.geojson',
        'curbs': 'curb_candidates.geojson', 'sidewalks': 'sidewalk_confirmed_areas.geojson',
        'junctions': 'junction_or_open_area_support.geojson',
        'constraints': 'inferred_road_edge_constraints.geojson',
        'building_strong': 'building_marker_candidates.geojson',
        'building_possible': 'building_marker_possible.geojson', 'context': 'scene_object_context.geojson',
    }
    layers = dict((name, read_feature_collection(os.path.join(vectors, filename), name))
                  for name, filename in paths.items())
    # 最重要的防线：阶段7拒绝被错误标为可正常道路的推断面。
    for item in layers['roads']:
        prop = item['properties']
        if prop['evidence_state'] == 'inferred' and prop['render_policy'] == 'eligible':
            fail('推断道路被阶段6错误标为eligible，拒绝渲染：' + item.get('id', ''))
    for item in layers['building_strong'] + layers['building_possible']:
        prop = item['properties']
        if prop.get('completeness') != 'partial' or prop.get('semantic_evidence_state') != 'inferred':
            fail('建筑标识状态不符合冻结契约：' + item.get('id', ''))

    data = build_render_data(layers)
    extent = feature_bounds(layers['roads'] + layers['sidewalks'] + layers['building_strong'])
    trajectory_contract = {'enabled': False,
                           'reason': '本次未提供FAST-LOCALIZATION定位轨迹；仅渲染阶段6冻结矢量。'}
    raw_rtk_contract = {'enabled': False,
                        'reason': '本次未提供未筛选RTK轨迹；产品不包含RTK参考图层。'}
    trajectory_data = None
    if args.localization_trajectory:
        trajectory_data, trajectory_contract = localization_overlay(
            args.localization_trajectory, args.trajectory_map_dir, extent)
        data['localizationTrajectory'] = trajectory_data
    if args.raw_rtk_trajectory:
        raw_rtk_data, raw_rtk_contract = raw_rtk_xy_overlay(
            args.raw_rtk_trajectory, trajectory_data['points'], extent)
        data['rawRtkTrajectory'] = raw_rtk_data
    data['extent'] = extent
    data['counts'] = {
        'observedRoad': sum(1 for item in data['roads'] if item['p']['evidence_state'] == 'observed'),
        'inferredRoad': sum(1 for item in data['roads'] if item['p']['evidence_state'] == 'inferred'),
        'buildingStrong': len(data['buildingStrong']), 'buildingPossible': len(data['buildingPossible']),
    }
    os.makedirs(os.path.join(output, 'web'))
    os.makedirs(os.path.join(output, 'overview'))
    os.makedirs(os.path.join(output, 'validation'))
    render_html(os.path.join(output, 'web', 'index.html'), data)
    render_svg(os.path.join(output, 'overview', 'navigation_2p5d_overview.svg'), data, extent)
    contract = {
        'schema': SCHEMA, 'stage': '07_2p5d_product_rendering', 'source_stage06_handoff': handoff,
        'source_stage06_v1': stage06, 'coordinates': 'FAST-LIVO2局部map坐标；产品不声明WGS84地理坐标。',
        'rendering_rules': {
            'road_surface_observed': '正常道路面、阴影和中心虚线。',
            'road_search_surface_bridge': '有two_sided_continuity_bridge来源的推断连续道路默认显示为低饱和候选，不得与实测道路同色。',
            'road_search_surface_fixed_width': 'fixed_width_fallback默认显示为导航道路底色，但必须保留虚线候选边界，不能与实测道路混同。',
            'road_fragment_suppression': 'cell_count小于50的道路小碎片不进入产品视图，但保留在阶段6冻结矢量中。',
            'sidewalk_confirmed': '正常显示为确认人行道/路肩。',
            'curb_candidate': '显示为细边线；inferred路沿降低不透明度。',
            'building_marker': '只显示长度不少于6m、点数不少于200、覆盖率不少于0.85、无道路反证且距道路/人行道/路肩/路沿不少于2m的partial可见立面；以可见立面锚定的导航体块显示。',
            'building_possible': '默认关闭；仍应用同一反证和2m导航走行带净距过滤，打开时用浅色弱化。',
            'unknown_or_context': '未知边界、轨迹参考和排除边不进入产品图；路口/开放区、约束与植被仅为可选复核层。',
        },
        'prohibited_actions': manifest['rendering_contract']['prohibited_interpretations'],
        'localization_trajectory_overlay': trajectory_contract,
        'raw_rtk_xy_overlay': raw_rtk_contract,
        'outputs': {'web': 'web/index.html', 'overview_svg': 'overview/navigation_2p5d_overview.svg'},
    }
    with open(os.path.join(output, 'render_contract.json'), 'w') as handle:
        json.dump(contract, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    report = {'schema': SCHEMA, 'status': 'complete', 'input_feature_counts':
              dict((key, len(value)) for key, value in layers.items()),
              'display_feature_counts': {'road_surface_after_fragment_suppression': len(data['roads']),
                                         'strong_building_after_product_filter': len(data['buildingStrong']),
                                         'possible_building_after_product_filter': len(data['buildingPossible']),
                                         'suppressed_road_fragments': data['suppressedRoadFragments'],
                                         'suppressed_strong_building_markers': data['suppressedBuildingMarkers']},
              'display_defaults': {'observed_road': True, 'bridge_road_candidate': True,
                                   'fixed_width_road_candidate': True, 'confirmed_sidewalk': True,
                                   'curb_candidate': True, 'junction_or_open_area_candidate': True,
                                   'strong_building_marker': True, 'possible_building_marker': False,
                                   'context': False,
                                   'localization_trajectory': bool(args.localization_trajectory),
                                   'raw_rtk_xy_trajectory': bool(args.raw_rtk_trajectory)},
              'safety_checks': ['阶段6冻结清单为frozen。', '推断道路未升级为eligible道路。',
                                '所有建筑标识保持partial/inferred。', '建筑产品层已剔除道路反证、短小或支持不足候选。',
                                '未读取pcd_review或阶段4/5原始PCD。',
                                '红色定位轨迹只接受FAST-LOCALIZATION地图系CSV。',
                                '若启用蓝色RTK，仅使用原始CSV的map_x/map_y；不读取Z，不标为真值，不计算ATE/RMSE。']}
    with open(os.path.join(output, 'validation', 'stage07_render_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'stage07_complete.json'), 'w') as handle:
        json.dump({'schema': SCHEMA, 'stage': '07_2p5d_product_rendering', 'status': 'complete',
                   'source_stage06_handoff': handoff, 'render_contract': 'render_contract.json'}, handle,
                  ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'README.md'), 'w') as handle:
        handle.write('# 阶段 7 2.5D 导航产品\n\n')
        handle.write('打开 `web/index.html` 即可查看；无需HTTP服务，页面内嵌阶段6矢量的必要渲染数据。\n\n')
        handle.write('可拖动、缩放、旋转、调节俯视角、开关图层并通过“导出 PNG”保存当前视图。\n')
        if args.localization_trajectory:
            handle.write('本产品包含可播放的 FAST-LOCALIZATION 红色地图系定位轨迹；来源与地图指纹见 `render_contract.json`。\n')
        if args.raw_rtk_trajectory:
            handle.write('本产品另含未筛选 RTK 蓝色参考轨迹；仅使用 map_x/map_y 与红线同步展示，不是真值或精度评估。\n')
        handle.write('`overview/navigation_2p5d_overview.svg` 是静态总览。完整限制见 `render_contract.json`。\n')
    print('阶段7完成：输出=%s；Web产品=web/index.html' % output)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段7失败：%s' % error, file=sys.stderr)
        sys.exit(1)
