#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 0--3 共用的 PCD、栅格、轨迹和稳定地面读写工具。"""

from __future__ import print_function

import csv
import math
import os
import struct


GRID_M = 0.20


def fail(message):
    raise RuntimeError(message)


def grid_key(x, y):
    return (int(math.floor(x / GRID_M)), int(math.floor(y / GRID_M)))


def cell_center(cell):
    return ((cell[0] + 0.5) * GRID_M, (cell[1] + 0.5) * GRID_M)


def create_directory(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def packed_rgb(red, green, blue):
    return (red << 16) | (green << 8) | blue


def read_binary_pcd(path):
    with open(path, 'rb') as handle:
        point_count = None
        while True:
            line = handle.readline()
            if not line:
                fail('PCD 头不完整：' + path)
            text = line.decode('ascii').strip()
            if text.startswith('POINTS '):
                point_count = int(text.split()[1])
            if text == 'DATA binary':
                break
        if point_count is None:
            fail('PCD 缺少 POINTS：' + path)
        payload = handle.read()
    if len(payload) != point_count * 16:
        fail('PCD 长度不匹配：' + path)
    return [struct.unpack_from('fffI', payload, offset)
            for offset in range(0, len(payload), 16)]


def write_pcd(path, points):
    with open(path, 'wb') as handle:
        header = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n'
                  'FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n'
                  'WIDTH {0}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {0}\nDATA binary\n').format(len(points))
        handle.write(header.encode('ascii'))
        for x, y, z, rgb in points:
            handle.write(struct.pack('fffI', float(x), float(y), float(z), int(rgb)))


def read_ground(path):
    result = {}
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        need = ('cell_ix', 'cell_iy', 'ground_mean_z', 'stable_ground',
                'ground_frame_count', 'free_frame_count', 'observation_confidence')
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in need):
            fail('阶段 2 网格字段不兼容：' + path)
        for row in reader:
            if row['stable_ground'] != '1' or int(row['free_frame_count']) < 1:
                continue
            result[(int(row['cell_ix']), int(row['cell_iy']))] = {
                'z': float(row['ground_mean_z']),
                'ground_frames': int(row['ground_frame_count']),
                'free_frames': int(row['free_frame_count']),
                'confidence': float(row['observation_confidence']),
            }
    if not result:
        fail('没有可用稳定地面')
    return result


def read_trajectory(path):
    result = []
    with open(path, 'r') as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            result.append((float(row['map_imu_tx']), float(row['map_imu_ty'])))
    if len(result) < 3:
        fail('轨迹点不足')
    return result


def resample_trajectory(trajectory, spacing_m):
    """按路径距离降采样，并用较大时间窗估计切向，降低姿态瞬时抖动影响。"""
    samples = []
    accumulated = 0.0
    previous = trajectory[0]
    for index, point in enumerate(trajectory):
        if index:
            accumulated += math.hypot(point[0] - previous[0], point[1] - previous[1])
            previous = point
        if index not in (0, len(trajectory) - 1) and accumulated < spacing_m:
            continue
        accumulated = 0.0
        left_index = max(0, index - 12)
        right_index = min(len(trajectory) - 1, index + 12)
        tangent_x = trajectory[right_index][0] - trajectory[left_index][0]
        tangent_y = trajectory[right_index][1] - trajectory[left_index][1]
        norm = math.hypot(tangent_x, tangent_y)
        if norm < 0.05:
            continue
        tangent_x /= norm
        tangent_y /= norm
        samples.append({'x': point[0], 'y': point[1], 'tx': tangent_x, 'ty': tangent_y,
                        'nx': -tangent_y, 'ny': tangent_x})
    if len(samples) < 3:
        fail('有效轨迹切向不足')
    return samples


def closest_ground_z(ground, x, y):
    base = grid_key(x, y)
    best = None
    for dx in range(-2, 3):
        for dy in range(-2, 3):
            cell = (base[0] + dx, base[1] + dy)
            if cell not in ground:
                continue
            center_x, center_y = cell_center(cell)
            distance = math.hypot(center_x - x, center_y - y)
            if best is None or distance < best[0]:
                best = (distance, ground[cell]['z'])
    return None if best is None else best[1]
