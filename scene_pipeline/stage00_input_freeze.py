#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 0：将已有输入指纹整理为独立、可审计的冻结交接包。

阶段 1 在首次校正时已生成 ``input_inventory``，其中包含运行目录内关键输入的 SHA-256，
以及原始 bag 和标定文件的外部指纹。本程序不重新建图、不复制数据，也不重算 16 GB
原始 bag；它只校验已有清单的结构和运行目录内文件的当前指纹，再冻结本次 0--3 阶段
使用的代码文件哈希。
"""

from __future__ import print_function

import argparse
import hashlib
import json
import os
import sys


CANONICAL_CODE_FILES = (
    'stage00_input_freeze.py',
    'stage01_pose_correction.py',
    'stage02_geometric_evidence_v2.cpp',
    'stage03a_global_boundary_primitives.py',
    'stage032_boundary_constrained_corridors.py',
    'stage03_final_consolidation.py',
    'run_stage00_input_freeze.sh',
    'run_stage01_pose_correction.sh',
    'run_stage02_geometric_evidence_v2.sh',
    'run_stage03a_global_boundary_primitives.sh',
    'run_stage032_boundary_constrained_corridors.sh',
    'run_stage03_final_consolidation.sh',
    'common/__init__.py',
    'common/geometry.py',
    'common/boundary_geometry.py',
)


def fail(message):
    raise RuntimeError(message)


def sha256(path):
    hasher = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            hasher.update(block)
    return hasher.hexdigest()


def load_json(path, label):
    try:
        with open(path, 'r') as handle:
            return json.load(handle)
    except (IOError, ValueError) as error:
        fail(label + ' 不可读取或格式无效：{0} ({1})'.format(path, error))


def yaml_quote(value):
    return json.dumps(str(value), ensure_ascii=False)


def write_manifest(path, run_dir, stage01, input_manifest, external_sources, code_hashes):
    """只写入受控字符串和列表；JSON 字符串也是合法 YAML 标量。"""
    lines = [
        'schema: fast_livo_scene_pipeline_run_manifest/v1',
        'status: frozen',
        'purpose: 阶段0冻结输入、外部采集来源与阶段0至阶段3最终代码版本。',
        'run_dir: ' + yaml_quote(run_dir),
        'stage01_inventory: ' + yaml_quote(stage01),
        'input_inventory_schema: ' + yaml_quote(input_manifest.get('schema', 'unknown')),
        'external_source_schema: ' + yaml_quote(external_sources.get('schema', 'unknown')),
        'final_dense_map_points: ' + str(input_manifest.get('final_dense_map_points', 0)),
        'frame_cache:',
        '  frame_count: ' + str(input_manifest.get('frame_cache', {}).get('frame_count', 0)),
        '  total_points: ' + str(input_manifest.get('frame_cache', {}).get('total_points', 0)),
        'frozen_run_inputs:',
    ]
    for name, item in sorted(input_manifest.get('fingerprints', {}).items()):
        lines.extend((
            '  - name: ' + yaml_quote(name),
            '    path_relative_to_run: ' + yaml_quote(item['path_relative_to_run']),
            '    bytes: ' + str(item['bytes']),
            '    sha256: ' + yaml_quote(item['sha256']),
        ))
    lines.append('frozen_external_sources:')
    for name, item in sorted(external_sources.get('sources', {}).items()):
        lines.extend((
            '  - name: ' + yaml_quote(name),
            '    path_at_inventory_time: ' + yaml_quote(item['absolute_path_at_inventory_time']),
            '    bytes: ' + str(item['bytes']),
            '    sha256: ' + yaml_quote(item['sha256']),
        ))
    lines.append('canonical_code:')
    for name, digest in sorted(code_hashes.items()):
        lines.extend((
            '  - path: ' + yaml_quote(name),
            '    sha256: ' + yaml_quote(digest),
        ))
    lines.extend((
        'verification_policy:',
        '  run_inputs: 当前执行时重新计算并验证 SHA-256。',
        '  external_sources: 复用阶段1已保存的采集时 SHA-256；本次不读取或修改原始 bag。',
        '  code: 当前最终入口及公共模块按 SHA-256 冻结。',
    ))
    with open(path, 'w') as handle:
        handle.write('\n'.join(lines) + '\n')


def main():
    parser = argparse.ArgumentParser(description='阶段 0：冻结并校验输入、外部来源和最终代码版本')
    parser.add_argument('--run-dir', required=True)
    parser.add_argument('--stage01', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    run_dir = os.path.abspath(args.run_dir)
    stage01 = os.path.abspath(args.stage01)
    output = os.path.abspath(args.output)
    project_root = os.path.abspath(args.project_root)
    if not os.path.isdir(run_dir):
        fail('运行目录不存在：' + run_dir)
    inventory_dir = os.path.join(stage01, 'input_inventory')
    input_manifest = load_json(os.path.join(inventory_dir, 'input_manifest.json'), '阶段1运行目录输入清单')
    external_sources = load_json(os.path.join(inventory_dir, 'external_sources.json'), '阶段1外部来源清单')
    if input_manifest.get('source_policy') != 'Inputs are read-only. Cached PCD paths are normalized to paths relative to run_dir.':
        fail('阶段1输入清单的只读策略不匹配，拒绝冻结')
    if not external_sources.get('sources'):
        fail('阶段1外部来源清单为空，拒绝冻结')
    checks = []
    for name, item in sorted(input_manifest.get('fingerprints', {}).items()):
        relative = item.get('path_relative_to_run')
        path = os.path.join(run_dir, relative or '')
        if not relative or not os.path.isfile(path):
            fail('冻结输入缺失：{0} ({1})'.format(name, path))
        actual_bytes = os.path.getsize(path)
        actual_sha256 = sha256(path)
        match = actual_bytes == item.get('bytes') and actual_sha256 == item.get('sha256')
        checks.append({'name': name, 'path_relative_to_run': relative, 'bytes_expected': item.get('bytes'),
                       'bytes_actual': actual_bytes, 'sha256_expected': item.get('sha256'),
                       'sha256_actual': actual_sha256, 'match': match})
        if not match:
            fail('冻结输入指纹不一致：' + relative)
    code_hashes = {}
    for relative in CANONICAL_CODE_FILES:
        path = os.path.join(project_root, 'scene_pipeline', relative)
        if not os.path.isfile(path):
            fail('最终代码文件不存在：' + path)
        code_hashes[relative] = sha256(path)
    complete = os.path.join(output, 'stage00_complete.json')
    if os.path.exists(complete) and not args.overwrite:
        fail('阶段0交接包已存在，拒绝覆盖：' + output)
    if not os.path.isdir(output):
        os.makedirs(output)
    validation = os.path.join(output, 'validation')
    if not os.path.isdir(validation):
        os.makedirs(validation)
    write_manifest(os.path.join(output, 'run_manifest.yaml'), run_dir, stage01, input_manifest,
                   external_sources, code_hashes)
    report = {
        'schema': 'fast_livo_scene_pipeline_input_integrity_report/v1',
        'run_dir': run_dir,
        'stage01_inventory': stage01,
        'run_input_checks': checks,
        'all_run_input_checks_passed': True,
        'external_source_policy': '使用阶段1已记录的 SHA-256；本次未重算或修改外部原始 bag/标定文件。',
        'external_source_count': len(external_sources['sources']),
        'canonical_code_sha256': code_hashes,
    }
    with open(os.path.join(validation, 'input_integrity_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(complete, 'w') as handle:
        json.dump({'status': 'complete', 'stage': '0_input_freeze', 'run_dir': run_dir,
                   'stage01_inventory': stage01, 'policy': '只读校验输入；不修改原始采集、标定和建图输出。'},
                  handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    print('[scene-stage00] 完成：校验运行目录输入={0}，冻结代码={1}，输出={2}'.format(
        len(checks), len(code_hashes), output))


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as error:
        print('[scene-stage00] ' + str(error), file=sys.stderr)
        sys.exit(2)
