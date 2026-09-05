#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证阶段6输出契约是否完整且没有把推断对象升级成事实。"""

from __future__ import print_function

import argparse
import csv
import json
import os
import sys


REQUIRED_GEOJSON = (
    'road_centerline_segments.geojson', 'road_surface_areas.geojson', 'curb_candidates.geojson',
    'unknown_boundaries.geojson', 'sidewalk_confirmed_areas.geojson', 'inferred_road_edge_constraints.geojson',
    'road_edge_context_review.geojson', 'junction_or_open_area_candidates.geojson',
    'junction_or_open_area_support.geojson', 'building_marker_candidates.geojson',
    'building_marker_possible.geojson', 'scene_object_context.geojson')

REQUIRED_PCD_REVIEW = (
    '01_road_surface_vector_review.pcd', '02_road_centerline_vector_review.pcd',
    '02b_trajectory_reference_context.pcd', '02c_road_axis_with_trajectory_context_review.pcd',
    '03_boundary_vector_review.pcd', '04_sidewalk_confirmed_vector_review.pcd',
    '05_inferred_road_edge_constraint_vector_review.pcd', '06_road_edge_context_review.pcd',
    '07_junction_open_area_vector_review.pcd', '08_building_marker_candidate_vector_review.pcd',
    '09_building_marker_possible_vector_review.pcd', '10_scene_object_context_vector_review.pcd',
    'stage06_vector_review_all.pcd', 'stage06_pcd_review_contract.json')


def fail(message):
    raise RuntimeError(message)


def read_geojson(path):
    with open(path, 'r') as handle:
        value = json.load(handle)
    if value.get('type') != 'FeatureCollection' or not isinstance(value.get('features'), list):
        fail('不是FeatureCollection：' + path)
    for item in value['features']:
        prop = item.get('properties', {})
        if 'geometry_evidence_state' not in prop or 'semantic_evidence_state' not in prop:
            fail('缺少双证据状态：%s 中的 %s' % (path, item.get('id')))
    return value


def main():
    parser = argparse.ArgumentParser(description='验证阶段6版本化矢量输出')
    parser.add_argument('--output', required=True, help='stage06_vector_map_v1目录')
    args = parser.parse_args()
    root = os.path.abspath(args.output)
    with open(os.path.join(root, 'stage06_complete.json'), 'r') as handle:
        complete = json.load(handle)
    if complete.get('status') != 'complete' or complete.get('stage') != '06_vectorization_and_regularization':
        fail('阶段6完成标记不兼容')
    with open(os.path.join(root, 'audit', 'stage06_input_validation.json'), 'r') as handle:
        inputs = json.load(handle)
    if not inputs.get('stage04_freeze_verified') or not inputs.get('stage05_freeze_verified'):
        fail('未记录阶段4/5冻结包校验成功')
    vectors = os.path.join(root, 'vectors')
    layers = {}
    for name in REQUIRED_GEOJSON:
        layers[name] = read_geojson(os.path.join(vectors, name))
    pcd_review = os.path.join(root, 'pcd_review')
    for name in REQUIRED_PCD_REVIEW:
        path = os.path.join(pcd_review, name)
        if not os.path.isfile(path) or os.path.getsize(path) == 0:
            fail('缺少或为空的CloudCompare复核层：' + path)
    with open(os.path.join(pcd_review, 'stage06_pcd_review_contract.json'), 'r') as handle:
        pcd_contract = json.load(handle)
    if pcd_contract.get('purpose') != 'CloudCompare人工复核的阶段6矢量显示副本，不是新的观测证据。':
        fail('PCD复核层契约不兼容')
    # 关键防线：固定宽度和桥接道路、道路约束、建筑标识不能变成observed语义。
    for item in layers['road_surface_areas.geojson']['features']:
        prop = item['properties']
        if prop['road_candidate_primary_source'] != 'two_sided_measured' and prop['semantic_evidence_state'] == 'observed':
            fail('推断道路搜索面被升级为observed：' + item['id'])
    for item in layers['road_centerline_segments.geojson']['features']:
        prop = item['properties']
        two_sided = prop['candidate_class'] in ('two_sided_measured_road_core', 'two_sided_continuity_bridge')
        if two_sided and prop.get('axis_position_kind') != 'two_sided_boundary_midpoint':
            fail('双边道路中轴没有使用左右边界中点：' + item['id'])
        if not two_sided and prop.get('layer') == 'road_centerline_observed':
            fail('单侧/开放区轨迹被错误标为实测道路中轴：' + item['id'])
    for item in layers['inferred_road_edge_constraints.geojson']['features']:
        prop = item['properties']
        if prop['evidence_state'] != 'inferred' or prop['constraint_confidence'] > 0.49:
            fail('道路边界约束状态或置信度不符合阶段4.2冻结契约：' + item['id'])
    for name in ('building_marker_candidates.geojson', 'building_marker_possible.geojson'):
        for item in layers[name]['features']:
            prop = item['properties']
            if prop['completeness'] != 'partial' or prop['semantic_evidence_state'] != 'inferred':
                fail('建筑标识被升级为完整/观测语义：' + item['id'])
            if item['geometry']['type'] != 'LineString':
                fail('建筑标识不应在阶段6成为建筑面：' + item['id'])
    with open(os.path.join(vectors, 'road_width_profiles.csv'), 'r') as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames or 'width_evidence_state' not in reader.fieldnames:
            fail('道路宽度剖面缺少证据状态')
        for row in reader:
            if row['width_evidence_state'] == 'observed' and row['candidate_class'] != 'two_sided_measured_road_core':
                fail('非双边实测宽度被标为observed：样本' + row['sample_id'])
    print('阶段6矢量输出校验通过：GeoJSON图层=%d，PCD复核层=%d' % (len(layers), len(REQUIRED_PCD_REVIEW)))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段6校验失败：%s' % error, file=sys.stderr)
        sys.exit(1)
