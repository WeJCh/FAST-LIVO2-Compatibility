#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段5冻结交接：封存R1→R2→R3可见场景对象候选工作流。"""

from __future__ import print_function

import argparse
import hashlib
import json
import os
import sys
from datetime import datetime


def fail(message):
    raise RuntimeError(message)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        while True:
            block = handle.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def file_entry(path, label):
    if not os.path.isfile(path):
        fail('缺少%s：%s' % (label, path))
    return {
        'label': label, 'path': os.path.abspath(path),
        'size_bytes': os.path.getsize(path), 'sha256': sha256_file(path),
    }


def tree_entry(root, label):
    if not os.path.isdir(root):
        fail('缺少%s目录：%s' % (label, root))
    entries = []
    digest = hashlib.sha256()
    for current, _, files in os.walk(root):
        for filename in sorted(files):
            path = os.path.join(current, filename)
            relative = os.path.relpath(path, root)
            value = sha256_file(path)
            entries.append({'relative_path': relative, 'size_bytes': os.path.getsize(path), 'sha256': value})
            digest.update(relative.encode('utf-8'))
            digest.update(b'\0')
            digest.update(value.encode('ascii'))
            digest.update(b'\0')
    return {
        'label': label, 'path': os.path.abspath(root), 'file_count': len(entries),
        'total_size_bytes': sum(item['size_bytes'] for item in entries),
        'tree_sha256': digest.hexdigest(), 'files': entries,
    }


def read_json(path, label):
    try:
        with open(path, 'r') as handle:
            return json.load(handle)
    except Exception as error:
        fail('%s无法读取：%s' % (label, error))


def main():
    parser = argparse.ArgumentParser(description='阶段5冻结交接包')
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--stage01', required=True)
    parser.add_argument('--stage02', required=True)
    parser.add_argument('--stage03', required=True)
    parser.add_argument('--stage04-handoff', required=True)
    parser.add_argument('--stage05-r1', required=True)
    parser.add_argument('--stage05-r2', required=True)
    parser.add_argument('--stage05-r3', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    project_root = os.path.abspath(args.project_root)
    stage01 = os.path.abspath(args.stage01)
    stage02 = os.path.abspath(args.stage02)
    stage03 = os.path.abspath(args.stage03)
    stage04 = os.path.abspath(args.stage04_handoff)
    r1, r2, r3 = map(os.path.abspath, (args.stage05_r1, args.stage05_r2, args.stage05_r3))
    output = os.path.abspath(args.output)
    if os.path.exists(output):
        fail('阶段5冻结目录已存在，拒绝覆盖：' + output)

    r1_complete = read_json(os.path.join(r1, 'stage05_complete.json'), 'R1完成标记')
    r2_complete = read_json(os.path.join(r2, 'stage05_r2_complete.json'), 'R2完成标记')
    r3_complete = read_json(os.path.join(r3, 'stage05_r3_complete.json'), 'R3完成标记')
    if any(item.get('status') != 'complete' for item in (r1_complete, r2_complete, r3_complete)):
        fail('阶段5 R1/R2/R3存在未完成输出，拒绝冻结')

    input_specs = (
        (os.path.join(stage01, 'pose_correction', 'frame_map_corrections.csv'), '阶段1逐帧回环校正'),
        (os.path.join(stage01, 'input_inventory', 'input_manifest.json'), '阶段1输入指纹清单'),
        (os.path.join(stage02, 'evidence', 'geometric_observation_grid.csv'), '阶段2几何观测网格'),
        (os.path.join(stage03, 'evidence', 'road_surface_candidate_complete_cells.csv'), '阶段3完整道路候选格'),
        (os.path.join(stage04, 'stage04_freeze_manifest.json'), '阶段4冻结清单'),
    )
    inputs = [file_entry(path, label) for path, label in input_specs]
    source_specs = (
        ('scene_pipeline/stage05_visible_scene_objects.cpp', '阶段5 R1提取代码'),
        ('scene_pipeline/run_stage05_visible_scene_objects.sh', '阶段5 R1运行入口'),
        ('scene_pipeline/verify_stage05_visible_scene_objects.py', '阶段5 R1校验代码'),
        ('scene_pipeline/run_verify_stage05_visible_scene_objects.sh', '阶段5 R1校验入口'),
        ('scene_pipeline/stage05_navigation_structure_refinement.py', '阶段5 R2精炼代码'),
        ('scene_pipeline/run_stage05_navigation_structure_refinement.sh', '阶段5 R2运行入口'),
        ('scene_pipeline/verify_stage05_navigation_structure_refinement.py', '阶段5 R2校验代码'),
        ('scene_pipeline/run_verify_stage05_navigation_structure_refinement.sh', '阶段5 R2校验入口'),
        ('scene_pipeline/stage05_navigation_structure_refinement_r3.py', '阶段5 R3精炼代码'),
        ('scene_pipeline/run_stage05_navigation_structure_refinement_r3.sh', '阶段5 R3运行入口'),
        ('scene_pipeline/verify_stage05_navigation_structure_refinement_r3.py', '阶段5 R3校验代码'),
        ('scene_pipeline/run_verify_stage05_navigation_structure_refinement_r3.sh', '阶段5 R3校验入口'),
        ('scene_pipeline/stage05_freeze_handoff.py', '阶段5冻结代码'),
        ('scene_pipeline/verify_stage05_freeze.py', '阶段5冻结校验代码'),
        ('scene_pipeline/run_verify_stage05_freeze.sh', '阶段5冻结校验入口'),
    )
    source_files = [file_entry(os.path.join(project_root, path), label) for path, label in source_specs]
    artifacts = [
        tree_entry(r1, '阶段5 R1多帧对象证据输出'),
        tree_entry(r2, '阶段5 R2导航结构精炼输出'),
        tree_entry(r3, '阶段5 R3道路反证与绿色层回收输出'),
    ]

    os.makedirs(os.path.join(output, 'validation'))
    manifest = {
        'schema': 'fast_livo_scene_pipeline_stage05_freeze_handoff/v1',
        'status': 'frozen', 'stage': '05_visible_scene_objects',
        'frozen_at_utc': datetime.utcnow().replace(microsecond=0).isoformat() + 'Z',
        'accepted_baseline': {
            'stage05_r1': r1, 'stage05_r2': r2, 'stage05_r3': r3,
            'consumption_policy': 'R3是阶段6/7唯一场景对象候选基线；R1/R2保留为R3可审计来源，不得跳过或混入pre_*迭代。',
        },
        'upstream_frozen_inputs': inputs,
        'implementation_files': source_files,
        'accepted_output_trees': artifacts,
        'rendering_contract': {
            'optional_page_layers': [
                'building_marker_candidate_support.pcd',
                'building_marker_possible_support.pcd',
                'building_marker_possible_recovered_support.pcd',
            ],
            'required_attributes': ['geometry_evidence_state=observed', 'semantic_evidence_state=inferred', 'completeness=partial'],
            'prohibited_interpretations': [
                '不得把可选橙色/浅橙层解释为完整建筑轮廓、建筑深度、屋顶或背面。',
                '不得把绿色、灰色或红色层渲染为建筑。',
                '页面必须允许分别开关橙色强候选、浅橙R1可能候选和浅橙绿色层回收候选。',
            ],
        },
        'known_limits': [
            '道路旁平整绿篱与可见墙面在现有单方向点云中存在几何证据重叠；建筑候选保持inferred。',
            '对象审计表保存来源稳定体素数、最大帧支持数和聚合帧支持量；未保存每个对象的精确帧ID列表。',
            '绿色层局部平面回收仅是possible候选，默认不应作为强建筑渲染。',
            '所有对象默认partial；阶段5不输出完整building_block。',
        ],
        'supersession_policy': 'stage05_navigation_structure_refinement_r2_pre_vegetation_fix与stage05_navigation_structure_refinement_r3_pre_csv_contract_fix已归档，仅供追溯，不得作为阶段6/7输入。',
    }
    manifest_path = os.path.join(output, 'stage05_freeze_manifest.json')
    with open(manifest_path, 'w') as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    report = {
        'status': 'complete', 'stage': '05_freeze_handoff',
        'manifest_sha256': sha256_file(manifest_path),
        'accepted_stage05_r1': r1, 'accepted_stage05_r2': r2, 'accepted_stage05_r3': r3,
        'checks': [
            'R1/R2/R3完成状态均为complete。',
            '阶段5实际读取的阶段1/2/3/4输入已记录SHA-256。',
            'R1/R2/R3输出树和所有实现/校验入口已记录SHA-256。',
            '冻结包不复制或改写阶段0--5既有大体积PCD。',
        ],
    }
    with open(os.path.join(output, 'validation', 'stage05_freeze_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'stage05_complete.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '05_freeze_handoff', 'manifest': manifest_path,
                   'accepted_stage05_r3': r3}, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'README.md'), 'w') as handle:
        handle.write('# 阶段 5 冻结交接包\n\n')
        handle.write('唯一可作为阶段 6/7 输入的场景对象候选基线：\n\n')
        handle.write('- R1 多帧对象证据：`%s`\n' % r1)
        handle.write('- R2 导航结构精炼：`%s`\n' % r2)
        handle.write('- R3 道路反证与绿色层回收：`%s`\n\n' % r3)
        handle.write('橙色与两类浅橙层可以作为页面**可选**图层；它们均是 `partial / inferred`，不得画成完整建筑块。\n')
    print('阶段5冻结交接完成：' + output)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段5冻结失败：%s' % error, file=sys.stderr)
        sys.exit(1)
