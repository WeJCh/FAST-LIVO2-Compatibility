#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段6冻结交接：封存可审计矢量地图与CloudCompare复核副本。"""

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
    return {'label': label, 'path': os.path.abspath(path), 'size_bytes': os.path.getsize(path),
            'sha256': sha256_file(path)}


def tree_entry(root, label):
    if not os.path.isdir(root):
        fail('缺少%s目录：%s' % (label, root))
    entries = []
    digest = hashlib.sha256()
    for current, _, names in os.walk(root):
        for name in sorted(names):
            path = os.path.join(current, name)
            relative = os.path.relpath(path, root)
            value = sha256_file(path)
            entries.append({'relative_path': relative, 'size_bytes': os.path.getsize(path), 'sha256': value})
            digest.update(relative.encode('utf-8'))
            digest.update(b'\0')
            digest.update(value.encode('ascii'))
            digest.update(b'\0')
    return {'label': label, 'path': os.path.abspath(root), 'file_count': len(entries),
            'total_size_bytes': sum(item['size_bytes'] for item in entries),
            'tree_sha256': digest.hexdigest(), 'files': entries}


def read_json(path, label):
    try:
        with open(path, 'r') as handle:
            return json.load(handle)
    except Exception as error:
        fail('%s无法读取：%s' % (label, error))


def main():
    parser = argparse.ArgumentParser(description='阶段6冻结交接包')
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--stage03', required=True)
    parser.add_argument('--stage04-handoff', required=True)
    parser.add_argument('--stage05-handoff', required=True)
    parser.add_argument('--stage06', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    project_root = os.path.abspath(args.project_root)
    stage03 = os.path.abspath(args.stage03)
    stage04 = os.path.abspath(args.stage04_handoff)
    stage05 = os.path.abspath(args.stage05_handoff)
    stage06 = os.path.abspath(args.stage06)
    output = os.path.abspath(args.output)
    if os.path.exists(output):
        fail('阶段6冻结目录已存在，拒绝覆盖：' + output)

    complete = read_json(os.path.join(stage06, 'stage06_complete.json'), '阶段6完成标记')
    if complete.get('status') != 'complete' or complete.get('stage') != '06_vectorization_and_regularization':
        fail('阶段6输出未完成或阶段标识不兼容，拒绝冻结')
    input_validation = read_json(os.path.join(stage06, 'audit', 'stage06_input_validation.json'),
                                 '阶段6输入校验记录')
    if not input_validation.get('stage04_freeze_verified') or not input_validation.get('stage05_freeze_verified'):
        fail('阶段6未记录阶段4/5冻结校验通过，拒绝冻结')

    input_specs = (
        (os.path.join(stage03, 'stage03_complete.json'), '阶段3最终交接完成标记'),
        (os.path.join(stage03, 'evidence', 'road_surface_candidate_complete_cells.csv'), '阶段3道路栅格契约'),
        (os.path.join(stage03, 'evidence', 'road_cross_section_confidence.csv'), '阶段3横断面契约'),
        (os.path.join(stage04, 'stage04_freeze_manifest.json'), '阶段4冻结清单'),
        (os.path.join(stage05, 'stage05_freeze_manifest.json'), '阶段5冻结清单'),
    )
    inputs = [file_entry(path, label) for path, label in input_specs]
    source_specs = (
        ('scene_pipeline/common/geometry.py', '阶段6共用PCD与栅格工具'),
        ('scene_pipeline/stage06_vectorization.py', '阶段6矢量化代码'),
        ('scene_pipeline/run_stage06_vectorization.sh', '阶段6运行入口'),
        ('scene_pipeline/verify_stage06_vectorization.py', '阶段6矢量校验代码'),
        ('scene_pipeline/run_verify_stage06_vectorization.sh', '阶段6矢量校验入口'),
        ('scene_pipeline/stage06_freeze_handoff.py', '阶段6冻结代码'),
        ('scene_pipeline/verify_stage06_freeze.py', '阶段6冻结校验代码'),
        ('scene_pipeline/run_verify_stage06_freeze.sh', '阶段6冻结校验入口'),
    )
    source_files = [file_entry(os.path.join(project_root, path), label) for path, label in source_specs]
    artifact = tree_entry(stage06, '阶段6 V1版本化矢量与CloudCompare复核输出')

    os.makedirs(os.path.join(output, 'validation'))
    manifest = {
        'schema': 'fast_livo_scene_pipeline_stage06_freeze_handoff/v1',
        'status': 'frozen', 'stage': '06_vectorization_and_regularization',
        'frozen_at_utc': datetime.utcnow().replace(microsecond=0).isoformat() + 'Z',
        'accepted_baseline': {
            'stage06_v1': stage06,
            'consumption_policy': '阶段7只能经本冻结包消费stage06_vector_map_v1的版本化GeoJSON/CSV；PCD复核副本仅供人工检查。',
        },
        'upstream_frozen_inputs': inputs,
        'implementation_files': source_files,
        'accepted_output_trees': [artifact],
        'rendering_contract': {
            'required_vector_layers': [
                'vectors/road_surface_areas.geojson', 'vectors/road_centerline_segments.geojson',
                'vectors/road_width_profiles.csv', 'vectors/curb_candidates.geojson',
                'vectors/sidewalk_confirmed_areas.geojson', 'vectors/junction_or_open_area_candidates.geojson',
                'vectors/building_marker_candidates.geojson', 'vectors/building_marker_possible.geojson',
            ],
            'required_attributes': ['source_stage', 'geometry_evidence_state', 'semantic_evidence_state',
                                    'evidence_state', 'render_policy'],
            'prohibited_interpretations': [
                '不得将road_search_surface_continuity或road_search_surface_fixed_width渲染为observed道路。',
                '不得将unknown_boundaries、road_edge_context_review或junction_or_open_area候选画成确定路沿或普通道路。',
                'inferred_road_edge_constraints只可作为低置信度对齐约束，不能直接显示为路沿。',
                'building_marker候选只能显示为partial/inferred导航标识，不能补成真实完整建筑轮廓。',
                '不得将pcd_review目录作为阶段7几何输入；它是CloudCompare显示副本。',
            ],
        },
        'known_limits': [
            '阶段6仅做同证据类别内的中心线分段与原始0.2m栅格并集，没有平行化、正交化、圆弧拟合或固定宽度补面。',
            '道路中轴仅在双边道路横断面中由左右边界中点计算；单侧/开放区轨迹保留为语境，不是道路中轴。',
            '确认人行道审计CSV没有每个栅格到surface_id的一对一映射，矢量面回溯至确认支持PCD与记录集合。',
            '建筑对象仅有可见拟合段、可见高度和partial状态；没有证据恢复完整建筑深度、背面或屋顶。',
        ],
        'supersession_policy': '后续阶段6改进必须新建版本化目录并重新冻结；不得覆盖stage06_vector_map_v1或本冻结包。',
    }
    manifest_path = os.path.join(output, 'stage06_freeze_manifest.json')
    with open(manifest_path, 'w') as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    report = {
        'status': 'complete', 'stage': '06_freeze_handoff', 'manifest_sha256': sha256_file(manifest_path),
        'accepted_stage06_v1': stage06,
        'checks': [
            '阶段4和阶段5冻结包已由阶段6运行和冻结入口只读校验。',
            '阶段6矢量输出与CloudCompare复核层已通过独立契约校验。',
            '阶段6代码、入口、校验器和完整输出树均已记录SHA-256。',
            '冻结包不复制或改写阶段0--6既有大体积输出。',
        ],
    }
    with open(os.path.join(output, 'validation', 'stage06_freeze_report.json'), 'w') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'stage06_complete.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '06_freeze_handoff', 'manifest': manifest_path,
                   'accepted_stage06_v1': stage06}, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'README.md'), 'w') as handle:
        handle.write('# 阶段 6 冻结交接包\n\n')
        handle.write('阶段 7 的唯一阶段 6 输入：`%s`。\n\n' % stage06)
        handle.write('只消费 `vectors/` 的版本化 GeoJSON/CSV 与证据状态；`pcd_review/` 仅供 CloudCompare 人工复核。\n')
    print('阶段6冻结交接完成：' + output)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段6冻结失败：%s' % error, file=sys.stderr)
        sys.exit(1)
