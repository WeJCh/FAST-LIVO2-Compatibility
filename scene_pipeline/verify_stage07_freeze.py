#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读验证阶段7冻结交接包，并递归验证阶段6冻结依赖。"""

from __future__ import print_function

import argparse
import hashlib
import json
import os
import subprocess
import sys


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        while True:
            block = handle.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def verify_file(entry):
    if not os.path.isfile(entry['path']):
        return False, '%s缺失：%s' % (entry.get('label', '文件'), entry['path'])
    if sha256_file(entry['path']) != entry['sha256']:
        return False, '%s SHA-256不匹配：%s' % (entry.get('label', '文件'), entry['path'])
    return True, ''


def verify_tree(tree):
    root = tree['path']
    if not os.path.isdir(root):
        return False, '%s目录缺失：%s' % (tree['label'], root)
    expected = dict((item['relative_path'], item) for item in tree['files'])
    actual = set()
    digest = hashlib.sha256()
    for current, _, names in os.walk(root):
        for name in sorted(names):
            path = os.path.join(current, name)
            relative = os.path.relpath(path, root)
            actual.add(relative)
            item = expected.get(relative)
            if item is None:
                return False, '%s出现未冻结文件：%s' % (tree['label'], relative)
            value = sha256_file(path)
            if value != item['sha256']:
                return False, '%s文件SHA-256不匹配：%s' % (tree['label'], relative)
            digest.update(relative.encode('utf-8'))
            digest.update(b'\0')
            digest.update(value.encode('ascii'))
            digest.update(b'\0')
    if actual != set(expected):
        return False, '%s缺少冻结文件' % tree['label']
    if digest.hexdigest() != tree['tree_sha256']:
        return False, '%s输出树SHA-256不匹配' % tree['label']
    return True, ''


def main():
    parser = argparse.ArgumentParser(description='验证阶段7冻结交接包')
    parser.add_argument('--handoff', required=True)
    parser.add_argument('--project-root', help='FAST-LIVO2工程根目录；由shell入口传入')
    args = parser.parse_args()
    handoff = os.path.abspath(args.handoff)
    with open(os.path.join(handoff, 'stage07_freeze_manifest.json'), 'r') as handle:
        manifest = json.load(handle)
    if manifest.get('status') != 'frozen' or manifest.get('stage') != '07_2p5d_product_rendering':
        raise RuntimeError('阶段7冻结状态或阶段标识不兼容')
    failures = []
    for entry in manifest['upstream_frozen_inputs'] + manifest['implementation_files']:
        valid, message = verify_file(entry)
        if not valid:
            failures.append(message)
    for tree in manifest['accepted_output_trees']:
        valid, message = verify_tree(tree)
        if not valid:
            failures.append(message)
    if failures:
        for message in failures:
            print('阶段7冻结校验失败：' + message, file=sys.stderr)
        raise RuntimeError('阶段7冻结完整性校验失败，共%d项' % len(failures))
    v4 = os.path.dirname(handoff)
    # RUN_DIR 可以位于任意磁盘位置，不能假设它总是 PROJECT_ROOT/Log/*。
    # 兼容直接调用旧 Python 入口时保留旧布局的推断；正式 shell 入口始终显式传入。
    project_root = (os.path.abspath(args.project_root) if args.project_root
                    else os.path.abspath(os.path.join(v4, '..', '..', '..')))
    if not os.path.isfile(os.path.join(project_root, 'scene_pipeline', 'run_verify_stage06_freeze.sh')):
        raise RuntimeError('FAST-LIVO2工程根目录无效；请通过 run_verify_stage07_freeze.sh 调用或传入 --project-root')
    run_dir = os.path.dirname(v4)
    subprocess.check_call([os.path.join(project_root, 'scene_pipeline', 'run_verify_stage06_freeze.sh'), run_dir])
    print('阶段7冻结完整性校验通过：输入=%d，代码=%d，输出树=%d' % (
        len(manifest['upstream_frozen_inputs']), len(manifest['implementation_files']),
        len(manifest['accepted_output_trees'])))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段7冻结校验失败：%s' % error, file=sys.stderr)
        sys.exit(1)
