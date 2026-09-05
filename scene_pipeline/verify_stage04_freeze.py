#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""只读验证阶段4冻结交接包的输入、代码和最终输出树完整性。"""

from __future__ import print_function

import argparse
import hashlib
import json
import os
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
    path = entry['path']
    if not os.path.isfile(path):
        return False, '%s 缺失：%s' % (entry.get('label', '文件'), path)
    actual = sha256_file(path)
    if actual != entry['sha256']:
        return False, '%s SHA-256不匹配：%s' % (entry.get('label', '文件'), path)
    return True, ''


def verify_tree(tree):
    root = tree['path']
    if not os.path.isdir(root):
        return False, '%s 目录缺失：%s' % (tree['label'], root)
    expected = dict((item['relative_path'], item) for item in tree['files'])
    actual = set()
    digest = hashlib.sha256()
    for current, _, files in os.walk(root):
        for filename in sorted(files):
            path = os.path.join(current, filename)
            relative = os.path.relpath(path, root)
            actual.add(relative)
            item = expected.get(relative)
            if item is None:
                return False, '%s 出现未冻结文件：%s' % (tree['label'], relative)
            value = sha256_file(path)
            if value != item['sha256']:
                return False, '%s 文件SHA-256不匹配：%s' % (tree['label'], relative)
            digest.update(relative.encode('utf-8'))
            digest.update(b'\0')
            digest.update(value.encode('ascii'))
            digest.update(b'\0')
    if actual != set(expected):
        missing = sorted(set(expected).difference(actual))
        return False, '%s 缺少冻结文件：%s' % (tree['label'], ', '.join(missing))
    if digest.hexdigest() != tree['tree_sha256']:
        return False, '%s 输出树SHA-256不匹配' % tree['label']
    return True, ''


def main():
    parser = argparse.ArgumentParser(description='验证阶段4冻结交接包（只读）')
    parser.add_argument('--handoff', required=True, help='stage04_final_handoff目录')
    arguments = parser.parse_args()
    handoff = os.path.abspath(arguments.handoff)
    manifest_path = os.path.join(handoff, 'stage04_freeze_manifest.json')
    if not os.path.isfile(manifest_path):
        raise RuntimeError('缺少冻结清单：' + manifest_path)
    with open(manifest_path, 'r') as handle:
        manifest = json.load(handle)
    if manifest.get('status') != 'frozen':
        raise RuntimeError('冻结清单状态不是 frozen')
    failures = []
    for item in manifest['upstream_frozen_inputs'] + manifest['implementation_files']:
        ok, message = verify_file(item)
        if not ok:
            failures.append(message)
    for tree in manifest['accepted_output_trees']:
        ok, message = verify_tree(tree)
        if not ok:
            failures.append(message)
    if failures:
        for message in failures:
            print('冻结校验失败：' + message, file=sys.stderr)
        raise RuntimeError('阶段4冻结完整性校验失败，共%d项' % len(failures))
    print('阶段4冻结完整性校验通过：输入=%d，代码=%d，输出树=%d' % (
        len(manifest['upstream_frozen_inputs']), len(manifest['implementation_files']),
        len(manifest['accepted_output_trees'])))


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段4冻结校验失败：%s' % error, file=sys.stderr)
        sys.exit(1)
