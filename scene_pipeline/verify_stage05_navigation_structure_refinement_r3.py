#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读校验阶段5 R3道路反证与绿色层局部平面回收输出。"""

from __future__ import print_function

import argparse
import csv
import json
import os
import sys
from collections import Counter


def fail(message):
    raise RuntimeError(message)


def pcd_count(path):
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
                offset = handle.tell()
                break
    if count is None or count <= 0 or os.path.getsize(path) - offset != count * 16:
        fail('PCD点数或二进制负载无效：' + path)
    return count


def main():
    parser = argparse.ArgumentParser(description='校验阶段5 R3导航结构输出')
    parser.add_argument('--stage05-r3', required=True)
    args = parser.parse_args()
    root = os.path.abspath(args.stage05_r3)
    complete_path = os.path.join(root, 'stage05_r3_complete.json')
    report_path = os.path.join(root, 'validation', 'stage05_navigation_structure_r3_report.json')
    records_path = os.path.join(root, 'evidence', 'navigation_structure_r3_records.csv')
    for path in (complete_path, report_path, records_path):
        if not os.path.isfile(path):
            fail('缺少R3必要文件：' + path)
    with open(complete_path, 'r') as handle:
        complete = json.load(handle)
    with open(report_path, 'r') as handle:
        report = json.load(handle)
    if complete.get('status') != 'complete' or report.get('status') != 'complete':
        fail('R3完成状态不是 complete')
    if report.get('schema') != 'fast_livo_scene_pipeline_stage05_navigation_structure_refinement/v3':
        fail('R3报告schema不兼容')
    with open(records_path, 'r') as handle:
        rows = list(csv.DictReader(handle))
    counts = Counter(row['r3_navigation_class'] for row in rows)
    if dict(counts) != report.get('r3_class_counts'):
        fail('R3类别统计与报告不一致')
    if len(rows) != report.get('r1_object_count', 0) + counts['building_marker_possible_recovered']:
        fail('R3未覆盖全部R1对象或绿色层回收记录数不一致')
    for row in rows:
        if row['geometry_evidence_state'] != 'observed' or row['completeness'] != 'partial':
            fail('R3出现非观测或非partial对象：' + row['record_id'])
    evidence = os.path.join(root, 'evidence')
    for category, expected in report.get('r3_support_point_counts', {}).items():
        path = os.path.join(evidence, category + '_support.pcd')
        if pcd_count(path) != expected:
            fail('R3支持层点数与报告不一致：' + category)
    if report.get('green_recovered_point_count', 0) > report.get('green_source_point_count', 0):
        fail('R3绿色层回收点数超过来源点数')
    print('阶段5 R3输出校验通过：强建筑=%d，可能建筑=%d，道路上方拒绝=%d，绿色层回收=%d' % (
        counts['building_marker_candidate'], counts['building_marker_possible'] + counts['building_marker_possible_recovered'],
        counts['road_overhead_or_dynamic_rejected'], counts['building_marker_possible_recovered']))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段5 R3输出校验失败：%s' % error, file=sys.stderr)
        sys.exit(1)
