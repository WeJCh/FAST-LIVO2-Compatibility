#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读校验阶段5可见场景对象输出的结构、统计和PCD完整性。"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys
from collections import Counter


OBJECT_TYPES = (
    'visible_facade_candidate',
    'wall_candidate',
    'pole_or_trunk_candidate',
    'vegetation_or_nonplanar',
    'uncertain_vertical_structure',
)

PCD_FILES = (
    'observed_vertical_voxel_support.pcd',
    'visible_facade_candidate_support.pcd',
    'wall_candidate_support.pcd',
    'pole_or_trunk_candidate_support.pcd',
    'vegetation_or_nonplanar_support.pcd',
    'uncertain_vertical_structure_support.pcd',
)


def fail(message):
    raise RuntimeError(message)


def pcd_point_count(path):
    """只读PCD头并核对二进制负载长度，不把百万点可视层全部载入内存。"""
    with open(path, 'rb') as handle:
        count = None
        while True:
            line = handle.readline()
            if not line:
                fail('PCD头不完整：' + path)
            text = line.decode('ascii').strip()
            if text.startswith('POINTS '):
                count = int(text.split()[1])
            if text == 'DATA binary':
                data_offset = handle.tell()
                break
    if count is None or count <= 0:
        fail('PCD没有有效点数：' + path)
    actual = os.path.getsize(path) - data_offset
    if actual != count * 16:
        fail('PCD负载长度不匹配：{0}，期望{1}，实际{2}'.format(path, count * 16, actual))
    return count


def main():
    parser = argparse.ArgumentParser(description='校验阶段5可见场景对象输出')
    parser.add_argument('--stage05', required=True, help='stage05_visible_scene_objects目录')
    arguments = parser.parse_args()
    stage05 = os.path.abspath(arguments.stage05)

    complete_path = os.path.join(stage05, 'stage05_complete.json')
    report_path = os.path.join(stage05, 'validation', 'stage05_visible_scene_objects_report.json')
    records_path = os.path.join(stage05, 'evidence', 'visible_scene_object_records.csv')
    contract_path = os.path.join(stage05, 'stage05_input_contract.json')
    for path in (complete_path, report_path, records_path, contract_path):
        if not os.path.isfile(path):
            fail('缺少阶段5必要文件：' + path)
    with open(complete_path, 'r') as handle:
        complete = json.load(handle)
    with open(report_path, 'r') as handle:
        report = json.load(handle)
    if complete.get('status') != 'complete' or report.get('status') != 'complete':
        fail('阶段5完成状态不是 complete')
    if report.get('schema') != 'fast_livo_scene_pipeline_stage05_visible_scene_objects/v1':
        fail('阶段5报告schema不兼容')
    if report.get('source_points_read') != report.get('valid_transformed_points'):
        fail('阶段5存在未处理的无效变换点')
    if report.get('parameters', {}).get('voxel_m') != 0.25:
        fail('阶段5体素参数不是已验收的0.25 m')

    with open(records_path, 'r') as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        fail('阶段5对象审计表为空')
    counts = Counter()
    for row in rows:
        if row['object_type'] not in OBJECT_TYPES:
            fail('未知对象类别：' + row['object_type'])
        if row['geometry_evidence_state'] != 'observed':
            fail('对象几何证据状态不是 observed：' + row['object_id'])
        if row['completeness'] != 'partial':
            fail('阶段5不应出现非 partial 对象：' + row['object_id'])
        if int(row['source_stable_voxel_count']) <= 0:
            fail('对象没有来源稳定体素：' + row['object_id'])
        confidence = float(row['confidence'])
        if not math.isfinite(confidence) or confidence < 0.0 or confidence > 1.0:
            fail('对象置信度无效：' + row['object_id'])
        counts[row['object_type']] += 1
    if dict(counts) != report.get('object_type_counts'):
        fail('对象类别统计与报告不一致')
    if len(rows) != complete.get('object_records'):
        fail('对象审计表行数与完成标记不一致')

    point_counts = {}
    for name in PCD_FILES:
        path = os.path.join(stage05, 'evidence', name)
        if not os.path.isfile(path):
            fail('缺少阶段5可视证据层：' + path)
        point_counts[name] = pcd_point_count(path)
    if point_counts['observed_vertical_voxel_support.pcd'] != report.get('classified_observed_vertical_voxels'):
        fail('真实稳定体素PCD点数与报告不一致')

    print('阶段5输出校验通过：对象={0}，分类体素={1}，PCD层={2}'.format(
        len(rows), report['classified_observed_vertical_voxels'], len(point_counts)))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段5输出校验失败：{0}'.format(error), file=sys.stderr)
        sys.exit(1)
