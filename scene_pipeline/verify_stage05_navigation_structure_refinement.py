#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读校验阶段5 R2导航结构精炼输出。"""

from __future__ import print_function

import argparse
import csv
import json
import os
import sys
from collections import Counter


VALID_CLASSES = (
    'building_marker_candidate', 'building_marker_possible',
    'vegetation_like_rejected', 'vegetation_or_nonplanar',
    'pole_or_trunk_context', 'vertical_structure_context',
)


def fail(message):
    raise RuntimeError(message)


def pcd_point_count(path):
    """仅读取PCD头并检查二进制长度，避免全量加载可视层。"""
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
        fail('PCD点数无效：' + path)
    if os.path.getsize(path) - data_offset != count * 16:
        fail('PCD二进制负载长度不匹配：' + path)
    return count


def main():
    parser = argparse.ArgumentParser(description='校验阶段5 R2导航结构精炼输出')
    parser.add_argument('--stage05-r2', required=True, help='stage05_navigation_structure_refinement_r2目录')
    arguments = parser.parse_args()
    root = os.path.abspath(arguments.stage05_r2)
    complete_path = os.path.join(root, 'stage05_r2_complete.json')
    report_path = os.path.join(root, 'validation', 'stage05_navigation_structure_refinement_report.json')
    records_path = os.path.join(root, 'evidence', 'navigation_structure_records.csv')
    for path in (complete_path, report_path, records_path):
        if not os.path.isfile(path):
            fail('缺少R2必要文件：' + path)
    with open(complete_path, 'r') as handle:
        complete = json.load(handle)
    with open(report_path, 'r') as handle:
        report = json.load(handle)
    if complete.get('status') != 'complete' or report.get('status') != 'complete':
        fail('R2完成状态不是 complete')
    if report.get('schema') != 'fast_livo_scene_pipeline_stage05_navigation_structure_refinement/v1':
        fail('R2报告schema不兼容')

    with open(records_path, 'r') as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) != report.get('r1_object_count'):
        fail('R2记录数未覆盖全部R1对象')
    counts = Counter()
    for row in rows:
        category = row['r2_navigation_class']
        if category not in VALID_CLASSES:
            fail('未知R2类别：' + category)
        if row['geometry_evidence_state'] != 'observed':
            fail('R2对象几何证据状态不是 observed：' + row['r1_object_id'])
        if row['completeness'] != 'partial':
            fail('R2不应出现非 partial 对象：' + row['r1_object_id'])
        counts[category] += 1
    if dict(counts) != report.get('r2_class_counts'):
        fail('R2类别统计与报告不一致')

    evidence_dir = os.path.join(root, 'evidence')
    support_counts = report.get('r2_support_point_counts', {})
    for category, expected_count in support_counts.items():
        path = os.path.join(evidence_dir, category + '_support.pcd')
        if not os.path.isfile(path):
            fail('缺少R2支持层：' + path)
        actual_count = pcd_point_count(path)
        if actual_count != expected_count:
            fail('R2支持层点数与报告不一致：' + category)
    if not counts['building_marker_candidate']:
        fail('没有可供人工复核的强建筑标识候选')
    print('阶段5 R2输出校验通过：对象={0}，强建筑标识={1}，可能标识={2}，植被样拒绝={3}'.format(
        len(rows), counts['building_marker_candidate'], counts['building_marker_possible'],
        counts['vegetation_like_rejected']))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段5 R2输出校验失败：{0}'.format(error), file=sys.stderr)
        sys.exit(1)
