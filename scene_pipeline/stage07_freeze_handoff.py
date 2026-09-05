#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段7冻结交接：封存当前接受的2.5D产品与其可复现实现。"""

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
    files = []
    digest = hashlib.sha256()
    for current, _, names in os.walk(root):
        for name in sorted(names):
            path = os.path.join(current, name)
            relative = os.path.relpath(path, root)
            value = sha256_file(path)
            files.append({'relative_path': relative, 'size_bytes': os.path.getsize(path), 'sha256': value})
            digest.update(relative.encode('utf-8'))
            digest.update(b'\0')
            digest.update(value.encode('ascii'))
            digest.update(b'\0')
    return {'label': label, 'path': os.path.abspath(root), 'file_count': len(files),
            'total_size_bytes': sum(item['size_bytes'] for item in files),
            'tree_sha256': digest.hexdigest(), 'files': files}


def read_json(path, label):
    try:
        with open(path, 'r') as handle:
            return json.load(handle)
    except Exception as error:
        fail('%s无法读取：%s' % (label, error))


def main():
    parser = argparse.ArgumentParser(description='阶段7冻结交接包')
    parser.add_argument('--project-root', required=True)
    parser.add_argument('--stage06-handoff', required=True)
    parser.add_argument('--stage07', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    project_root = os.path.abspath(args.project_root)
    stage06 = os.path.abspath(args.stage06_handoff)
    stage07 = os.path.abspath(args.stage07)
    output = os.path.abspath(args.output)
    if os.path.exists(output):
        fail('阶段7冻结目录已存在，拒绝覆盖：' + output)

    stage06_manifest = read_json(os.path.join(stage06, 'stage06_freeze_manifest.json'), '阶段6冻结清单')
    if stage06_manifest.get('status') != 'frozen':
        fail('阶段6不是冻结状态，拒绝冻结阶段7')
    complete = read_json(os.path.join(stage07, 'stage07_complete.json'), '阶段7完成标记')
    if complete.get('status') != 'complete' or complete.get('stage') != '07_2p5d_product_rendering':
        fail('阶段7输出未完成或阶段标识不兼容，拒绝冻结')
    contract = read_json(os.path.join(stage07, 'render_contract.json'), '阶段7渲染契约')
    report = read_json(os.path.join(stage07, 'validation', 'stage07_render_report.json'), '阶段7渲染报告')
    if report.get('status') != 'complete':
        fail('阶段7渲染报告不是完成状态')

    upstream = [file_entry(os.path.join(stage06, 'stage06_freeze_manifest.json'), '阶段6冻结清单')]
    source_specs = (
        ('scene_pipeline/run_2p5d_workflow.sh', '端到端2.5D工作流入口'),
        ('scene_pipeline/stage07_navigation_2p5d.py', '阶段7渲染实现'),
        ('scene_pipeline/run_stage07_navigation_2p5d.sh', '阶段7渲染入口'),
        ('scene_pipeline/verify_stage07_navigation_2p5d.py', '阶段7产品校验器'),
        ('scene_pipeline/run_verify_stage07_navigation_2p5d.sh', '阶段7产品校验入口'),
        ('scene_pipeline/stage07_freeze_handoff.py', '阶段7冻结实现'),
        ('scene_pipeline/verify_stage07_freeze.py', '阶段7冻结校验器'),
        ('scene_pipeline/run_verify_stage07_freeze.sh', '阶段7冻结校验入口'),
    )
    implementation = [file_entry(os.path.join(project_root, path), label) for path, label in source_specs]
    artifact = tree_entry(stage07, '阶段7 V4自包含Web产品与静态总览')

    os.makedirs(os.path.join(output, 'validation'))
    manifest = {
        'schema': 'fast_livo_scene_pipeline_stage07_freeze_handoff/v1',
        'status': 'frozen',
        'stage': '07_2p5d_product_rendering',
        'frozen_at_utc': datetime.utcnow().replace(microsecond=0).isoformat() + 'Z',
        'accepted_baseline': {
            'stage07_v4': stage07,
            'consumption_policy': '阶段8只能以本冻结包指定的V4产品、渲染契约和审计报告进行质量评估；不得回退至V1--V3。',
        },
        'upstream_frozen_inputs': upstream,
        'implementation_files': implementation,
        'accepted_output_trees': [artifact],
        'rendering_contract_sha256': sha256_file(os.path.join(stage07, 'render_contract.json')),
        'product_limits': contract.get('prohibited_actions', []),
        'known_limits': [
            '道路实测、连续桥接和固定宽度候选保持不同证据语义；固定宽度候选虽默认显示，仍以虚线边界标识。',
            '路口/开放区仅是未知但有观测支持的候选面，不能解释为已确认路口。',
            '建筑体块锚定partial可见立面；厚度为产品符号，不能恢复建筑背面、深度或屋顶。',
            '道路、人行道/路肩和路沿2m以内的建筑候选由产品层隐藏，不回写阶段5/6对象分类。',
        ],
        'supersession_policy': 'V1--V3已移至archive/stage07_pre_freeze_iterations，仅供追溯；后续改进必须新建Stage7版本并重新冻结，不得覆盖V4或本冻结包。',
    }
    manifest_path = os.path.join(output, 'stage07_freeze_manifest.json')
    with open(manifest_path, 'w') as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'validation', 'stage07_freeze_report.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '07_freeze_handoff',
                   'manifest_sha256': sha256_file(manifest_path), 'accepted_stage07_v4': stage07,
                   'checks': ['阶段6冻结包已由阶段7入口只读校验。', '阶段7 Web与SVG产品已通过独立校验。',
                              '阶段7实现、入口、校验器和完整输出树均已记录SHA-256。',
                              '冻结包不复制或改写阶段0--7既有大体积输出。']},
                  handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'stage07_complete.json'), 'w') as handle:
        json.dump({'status': 'complete', 'stage': '07_freeze_handoff', 'manifest': manifest_path,
                   'accepted_stage07_v4': stage07}, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write('\n')
    with open(os.path.join(output, 'README.md'), 'w') as handle:
        handle.write('# 阶段 7 冻结交接包\n\n')
        handle.write('阶段 8 的唯一阶段 7 产品输入：`%s`。\n\n' % stage07)
        handle.write('先验证冻结清单SHA-256，再进行质量评估；V1--V3只在归档目录保留追溯用途。\n')
    print('阶段7冻结交接完成：' + output)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段7冻结失败：%s' % error, file=sys.stderr)
        sys.exit(1)
