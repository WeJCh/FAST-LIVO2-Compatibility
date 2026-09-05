#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段4冻结交接：封存已验收的R7与阶段4.2 R2的来源和完整性清单。

本脚本只读取阶段0--4既有产物，在一个新的 stage04_final_handoff 目录写入冻结清单。
它不复制大体积PCD、不重跑提取，也不修改任一已验收输出；后续阶段必须以此清单指向的
R7/R2作为唯一阶段4基线。
"""

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
        'label': label,
        'path': os.path.abspath(path),
        'size_bytes': os.path.getsize(path),
        'sha256': sha256_file(path),
    }


def tree_entries(root, label):
    if not os.path.isdir(root):
        fail('缺少%s目录：%s' % (label, root))
    entries = []
    tree_digest = hashlib.sha256()
    for current, _, files in os.walk(root):
        for filename in sorted(files):
            path = os.path.join(current, filename)
            relative = os.path.relpath(path, root)
            digest = sha256_file(path)
            size = os.path.getsize(path)
            entries.append({'relative_path': relative, 'size_bytes': size, 'sha256': digest})
            tree_digest.update(relative.encode('utf-8'))
            tree_digest.update(b'\0')
            tree_digest.update(digest.encode('ascii'))
            tree_digest.update(b'\0')
    return {
        'label': label,
        'path': os.path.abspath(root),
        'file_count': len(entries),
        'total_size_bytes': sum(item['size_bytes'] for item in entries),
        'tree_sha256': tree_digest.hexdigest(),
        'files': entries,
    }


def read_json(path, label):
    try:
        with open(path, 'r') as handle:
            return json.load(handle)
    except Exception as error:
        fail('%s无法读取：%s' % (label, error))


def main():
    parser = argparse.ArgumentParser(description='阶段4冻结交接包')
    parser.add_argument('--project-root', required=True, help='FAST-LIVO2工程根目录')
    parser.add_argument('--stage02', required=True, help='冻结阶段2目录')
    parser.add_argument('--stage03a', required=True, help='冻结阶段3A目录')
    parser.add_argument('--stage03', required=True, help='冻结阶段3最终交接目录')
    parser.add_argument('--stage04', required=True, help='阶段4 R7最终目录')
    parser.add_argument('--stage042', required=True, help='阶段4.2 R2最终目录')
    parser.add_argument('--output', required=True, help='新的阶段4最终交接目录；必须不存在')
    arguments = parser.parse_args()
    project_root = os.path.abspath(arguments.project_root)
    stage02, stage03a, stage03, stage04, stage042 = map(
        os.path.abspath, (arguments.stage02, arguments.stage03a, arguments.stage03,
                          arguments.stage04, arguments.stage042))
    output = os.path.abspath(arguments.output)
    if os.path.exists(output):
        fail('阶段4冻结交接目录已存在，拒绝覆盖：' + output)

    # 阶段4实际读取的冻结上游证据，不以目录名替代文件内容校验。
    input_specs = (
        (os.path.join(stage02, 'evidence', 'geometric_observation_grid.csv'), '阶段2几何观测网格'),
        (os.path.join(stage02, 'evidence', 'continuous_hard_height_edge_support.pcd'), '阶段2连续硬高差'),
        (os.path.join(stage03a, 'evidence', 'merged_boundary_bands.csv'), '阶段3A合并边界带'),
        (os.path.join(stage03a, 'evidence', 'trajectory_boundary_hierarchy.csv'), '阶段3A边界层级'),
        (os.path.join(stage03, 'evidence', 'road_cross_section_confidence.csv'), '阶段3横断面置信度'),
        (os.path.join(stage03, 'evidence', 'road_surface_candidate_complete_cells.csv'), '阶段3完整道路候选'),
        (os.path.join(stage03, 'evidence', 'junction_turn_node_candidate_support.pcd'), '阶段3转弯路口候选'),
    )
    inputs = [file_entry(path, label) for path, label in input_specs]
    stage04_complete = read_json(os.path.join(stage04, 'stage04_complete.json'), '阶段4完成标记')
    stage04_report = read_json(os.path.join(stage04, 'validation', 'stage04_curb_sidewalk_report.json'), '阶段4报告')
    stage042_report = read_json(os.path.join(stage042, 'validation', 'stage042_inferred_road_edge_constraints_report.json'),
                                '阶段4.2报告')
    if stage04_complete.get('status') != 'complete' or stage042_report.get('status') != 'complete':
        fail('阶段4或阶段4.2未完成，拒绝冻结')

    source_paths = (
        ('scene_pipeline/stage04_curb_sidewalk_extraction.py', '阶段4提取代码'),
        ('scene_pipeline/run_stage04_curb_sidewalk_extraction.sh', '阶段4运行入口'),
        ('scene_pipeline/stage04_local_hard_edge_diagnosis.py', '阶段4.1诊断代码'),
        ('scene_pipeline/stage042_inferred_road_edge_constraints.py', '阶段4.2约束代码'),
        ('scene_pipeline/run_stage042_inferred_road_edge_constraints.sh', '阶段4.2运行入口'),
        ('scene_pipeline/stage04_freeze_handoff.py', '阶段4冻结代码'),
        ('scene_pipeline/verify_stage04_freeze.py', '阶段4冻结校验代码'),
        ('scene_pipeline/run_verify_stage04_freeze.sh', '阶段4冻结校验入口'),
    )
    source_files = [file_entry(os.path.join(project_root, relative), label) for relative, label in source_paths]
    artifacts = [tree_entries(stage04, '阶段4 R7最终输出'), tree_entries(stage042, '阶段4.2 R2最终输出')]

    os.makedirs(os.path.join(output, 'validation'))
    manifest = {
        'schema': 'fast_livo_scene_pipeline_stage04_freeze_handoff/v1',
        'status': 'frozen',
        'stage': '04_curb_sidewalk_extraction',
        'frozen_at_utc': datetime.utcnow().replace(microsecond=0).isoformat() + 'Z',
        'accepted_baseline': {
            'stage04_r7': stage04,
            'stage042_r2': stage042,
            'stage04_policy': 'R7是阶段4唯一语义输出基线；R4.2 R2只提供阶段6可审计的inferred道路边界约束。',
        },
        'upstream_frozen_inputs': inputs,
        'implementation_files': source_files,
        'accepted_output_trees': artifacts,
        'stage04_summary': {
            'boundary_record_count': stage04_report.get('boundary_record_count'),
            'boundary_class_counts': stage04_report.get('boundary_class_counts'),
            'boundary_evidence_state_counts': stage04_report.get('boundary_evidence_state_counts'),
            'sidewalk_surface_counts': stage04_report.get('sidewalk_surface_counts'),
            'confirmed_sidewalk_cells': stage04_report.get('confirmed_sidewalk_cells'),
            'low_road_evidence_hard_edge_review_records': stage04_report.get('low_road_evidence_hard_edge_review_records'),
            'stage042_status_counts': stage042_report.get('status_counts'),
        },
        'known_limits': [
            '墙根与绿化在当前冻结输入下不能可靠细分时保留 unknown 或组合候选；不得借由渲染补全。',
            '低道路证据硬高差和 inferred_road_edge_constraint 不是 observed 路沿或 observed 道路。',
            '阶段6只能消费阶段4.2 R2的 inferred 约束并保留来源、置信度和 inferred 状态。',
            '阶段7不得直接消费阶段4 PCD；必须消费阶段6版本化矢量及其证据状态。',
        ],
        'supersession_policy': 'R7/R2之外的阶段4迭代已移至 archive/stage04_pre_freeze_iterations；仅供追溯，不得作为后续阶段输入。',
    }
    manifest_path = os.path.join(output, 'stage04_freeze_manifest.json')
    with open(manifest_path, 'w') as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    report = {
        'status': 'complete',
        'stage': '04_freeze_handoff',
        'accepted_stage04_r7': stage04,
        'accepted_stage042_r2': stage042,
        'manifest_sha256': sha256_file(manifest_path),
        'checks': [
            '阶段4与阶段4.2完成标记均为 complete。',
            '阶段4实际消费的7个上游证据文件已记录 SHA-256。',
            'R7与R4.2 R2完整输出树和实现代码已记录 SHA-256。',
            '冻结包不复制或改写任何阶段0--4既有PCD/CSV。',
        ],
    }
    with open(os.path.join(output, 'validation', 'stage04_freeze_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'stage04_complete.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '04_freeze_handoff',
                   'manifest': manifest_path,
                   'accepted_stage04_r7': stage04, 'accepted_stage042_r2': stage042},
                  handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'README.md'), 'w') as handle:
        handle.write('# 阶段 4 冻结交接包\n\n')
        handle.write('唯一可作为后续输入的阶段 4 基线：\n\n')
        handle.write('- 阶段 4 语义输出：`%s`\n' % stage04)
        handle.write('- 阶段 4.2 推断道路边界约束：`%s`\n\n' % stage042)
        handle.write('请先验证 `stage04_freeze_manifest.json` 中的 SHA-256，再进入阶段 5/6/7。')
    print('阶段4冻结交接完成：' + output)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段4冻结失败：%s' % error, file=sys.stderr)
        sys.exit(1)
