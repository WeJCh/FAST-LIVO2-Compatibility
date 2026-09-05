#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""阶段 3A 使用的局部边界线性支持工具，不包含道路语义或阈值策略。"""

from __future__ import print_function

import math
from collections import defaultdict


def cell_key(x, y, cell_m):
    return (int(math.floor(x / cell_m)), int(math.floor(y / cell_m)))


def spatial_index(points, cell_m):
    index = defaultdict(list)
    for point_id, point in enumerate(points):
        index[cell_key(point[0], point[1], cell_m)].append(point_id)
    return index


def nearby(index, points, x, y, cell_m, radius_m):
    base_cell = cell_key(x, y, cell_m)
    span = int(math.ceil(radius_m / cell_m))
    radius_squared = radius_m * radius_m
    for dx in range(-span, span + 1):
        for dy in range(-span, span + 1):
            for point_id in index.get((base_cell[0] + dx, base_cell[1] + dy), ()):
                point = points[point_id]
                if (point[0] - x) ** 2 + (point[1] - y) ** 2 <= radius_squared:
                    yield point_id


def local_line_support(points, index, radius_m):
    """返回具有足够线性邻域的点及其二维切向。"""
    supported = {}
    for point_id, point in enumerate(points):
        neighbors = list(nearby(index, points, point[0], point[1], 0.40, radius_m))
        if len(neighbors) < 5:
            continue
        mean_x = sum(points[item][0] for item in neighbors) / len(neighbors)
        mean_y = sum(points[item][1] for item in neighbors) / len(neighbors)
        xx = sum((points[item][0] - mean_x) ** 2 for item in neighbors)
        yy = sum((points[item][1] - mean_y) ** 2 for item in neighbors)
        xy = sum((points[item][0] - mean_x) * (points[item][1] - mean_y) for item in neighbors)
        trace = xx + yy
        discriminant = max(0.0, (xx - yy) ** 2 + 4.0 * xy * xy)
        largest = (trace + math.sqrt(discriminant)) * 0.5
        smallest = (trace - math.sqrt(discriminant)) * 0.5
        if largest < 0.12 or (largest - smallest) / largest < 0.72:
            continue
        tangent_x, tangent_y = xy, largest - xx
        norm = math.hypot(tangent_x, tangent_y)
        if norm < 1e-9:
            tangent_x, tangent_y = 1.0, 0.0
        else:
            tangent_x /= norm
            tangent_y /= norm
        supported[point_id] = (tangent_x, tangent_y)
    return supported
