#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证阶段7产品是否仍遵守阶段6冻结的证据状态。"""
from __future__ import print_function
import argparse
import json
import os
import re
import sys

def fail(message):
    raise RuntimeError(message)


def verify_script_delimiters(html):
    """对内嵌脚本做轻量结构检查，弥补无浏览器环境下的语法防线。"""
    match = re.search(r'<script>(.*)</script>', html, re.DOTALL)
    if match is None:
        fail('产品页面缺少内嵌脚本')
    script = match.group(1)
    matching = {')': '(', ']': '[', '}': '{'}
    stack = []
    quote = None
    escaped = False
    for index, char in enumerate(script):
        if quote is not None:
            if escaped:
                escaped = False
            elif char == '\\':
                escaped = True
            elif char == quote:
                quote = None
            continue
        if char in ('\'', '\"', '`'):
            quote = char
        elif char in '([{':
            stack.append((char, index))
        elif char in matching:
            if not stack or stack[-1][0] != matching[char]:
                fail('产品脚本括号不匹配，位置%d附近出现%s' % (index, char))
            stack.pop()
    if quote is not None:
        fail('产品脚本存在未闭合字符串')
    if stack:
        fail('产品脚本存在未闭合括号，位置%d附近' % stack[-1][1])


def embedded_map_data(html):
    """读取页面内嵌的渲染数据，以验证蓝色RTK没有携带Z。"""
    marker = 'const DATA='
    start = html.find(marker)
    if start < 0:
        fail('产品页面缺少内嵌地图数据')
    start += len(marker)
    end = html.find(';\nconst canvas=', start)
    if end < 0:
        fail('产品页面内嵌地图数据边界不完整')
    try:
        return json.loads(html[start:end])
    except (TypeError, ValueError) as error:
        fail('产品页面内嵌地图数据不是有效JSON：%s' % error)


def main():
    parser = argparse.ArgumentParser(description='验证阶段7 2.5D导航产品')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = os.path.abspath(args.output)
    with open(os.path.join(root, 'stage07_complete.json'), 'r') as handle:
        complete = json.load(handle)
    if complete.get('status') != 'complete' or complete.get('stage') != '07_2p5d_product_rendering':
        fail('阶段7完成标记不兼容')
    with open(os.path.join(root, 'render_contract.json'), 'r') as handle:
        contract = json.load(handle)
    rules = contract.get('rendering_rules', {})
    # 连续性桥接可以默认以候选样式显示，但固定宽度回退必须默认关闭，二者均不得冒充实测道路。
    bridge_rule = rules.get('road_search_surface_bridge', '')
    fallback_rule = rules.get('road_search_surface_fixed_width', '')
    if '低饱和' not in bridge_rule or '不得与实测道路同色' not in bridge_rule:
        fail('连续性桥接道路没有弱化候选策略')
    if '默认显示' not in fallback_rule or '虚线候选边界' not in fallback_rule or '不能与实测道路混同' not in fallback_rule:
        fail('固定宽度道路没有可辨识的默认候选策略')
    if 'partial' not in rules.get('building_marker', ''):
        fail('建筑标识没有partial约束')
    if '不进入产品图' not in rules.get('unknown_or_context', ''):
        fail('未知对象没有排除策略')
    with open(os.path.join(root, 'validation', 'stage07_render_report.json'), 'r') as handle:
        report = json.load(handle)
    defaults = report.get('display_defaults', {})
    if defaults.get('bridge_road_candidate') is not True or defaults.get('fixed_width_road_candidate') is not True:
        fail('道路候选默认开关不符合产品契约')
    counts = report.get('display_feature_counts', {})
    if counts.get('suppressed_road_fragments', 0) <= 0 or counts.get('suppressed_strong_building_markers', 0) <= 0:
        fail('产品层碎片/建筑收紧统计缺失')
    trajectory = contract.get('localization_trajectory_overlay')
    # V4 早期冻结产品没有该可选显示层；将其视为关闭，避免升级校验器后反向破坏历史只读验收。
    if trajectory is None:
        trajectory = {'enabled': False}
    if not isinstance(trajectory, dict) or 'enabled' not in trajectory:
        fail('定位轨迹覆盖层契约格式错误')
    if bool(defaults.get('localization_trajectory')) != bool(trajectory.get('enabled')):
        fail('定位轨迹覆盖层默认状态与渲染契约不一致')
    raw_rtk = contract.get('raw_rtk_xy_overlay')
    # 早期产品没有蓝色原始RTK展示层，仍视为关闭。
    if raw_rtk is None:
        raw_rtk = {'enabled': False}
    if not isinstance(raw_rtk, dict) or 'enabled' not in raw_rtk:
        fail('原始RTK XY覆盖层契约格式错误')
    if bool(defaults.get('raw_rtk_xy_trajectory')) != bool(raw_rtk.get('enabled')):
        fail('原始RTK XY覆盖层默认状态与渲染契约不一致')
    if raw_rtk.get('enabled') and not trajectory.get('enabled'):
        fail('原始RTK XY图层必须与红色定位轨迹配对，不能单独显示')
    web = os.path.join(root, 'web', 'index.html')
    overview = os.path.join(root, 'overview', 'navigation_2p5d_overview.svg')
    for path in (web, overview):
        if not os.path.isfile(path) or os.path.getsize(path) < 1024:
            fail('缺少或异常小的产品文件：' + path)
    with open(web, 'r') as handle:
        html = handle.read()
    if 'pcd_review' in html or 'stage04_' in html or 'stage05_' in html:
        fail('产品页面不应直接消费PCD复核层或阶段4/5输出')
    # 当前产品使用内嵌JS；没有浏览器运行环境时，先阻止本次实际故障类型：三元表达式
    # 缺少空格被解析为 optional-chaining 数字字面量，整段脚本会在启动前失败。
    if re.search(r'\?\.[0-9]', html):
        fail('产品页面含非法optional-chaining数字字面量，Canvas脚本无法启动')
    if trajectory.get('enabled'):
        for field in ('source_trajectory_csv', 'map_identity', 'point_count',
                      'points_inside_product_extent', 'excluded_data'):
            if field not in trajectory:
                fail('定位轨迹覆盖层缺少审计字段：' + field)
        if trajectory['point_count'] < 2 or trajectory['points_inside_product_extent'] < 1:
            fail('定位轨迹覆盖层没有足够的有效地图范围重叠')
        source = trajectory['source_trajectory_csv']
        identity = trajectory['map_identity']
        if not source.get('sha256') or not identity.get('metadata', {}).get('sha256') or \
                not identity.get('imu_pose_file', {}).get('sha256'):
            fail('定位轨迹或地图身份指纹缺失')
        if '定位算法输出（红）' not in html or 'localizationTrajectory' not in html or \
                '定位算法输出轨迹（红）' not in open(overview, 'r').read():
            fail('定位轨迹没有同时进入Web和静态总览')
        if not raw_rtk.get('enabled') and ('RTK' in html or 'rtk_' in html.lower()):
            fail('定位展示产品不得混入RTK图层或真值暗示')
    if raw_rtk.get('enabled'):
        for field in ('source_raw_rtk_csv', 'coordinate_fields_used', 'z_handling',
                      'point_count', 'points_inside_product_extent', 'comparison_time_window',
                      'limitations'):
            if field not in raw_rtk:
                fail('原始RTK XY覆盖层缺少审计字段：' + field)
        if raw_rtk['point_count'] < 2 or raw_rtk['points_inside_product_extent'] < 1:
            fail('原始RTK XY覆盖层没有足够的有效范围重叠')
        source = raw_rtk['source_raw_rtk_csv']
        if not isinstance(source, dict) or not source.get('sha256'):
            fail('原始RTK CSV缺少来源SHA-256')
        if raw_rtk['coordinate_fields_used'] != ['timestamp_sec', 'map_x', 'map_y']:
            fail('原始RTK图层没有严格限定为timestamp_sec,map_x,map_y')
        if '不读取' not in raw_rtk['z_handling'] or 'map_z' not in raw_rtk['z_handling']:
            fail('原始RTK图层没有明确排除Z')
        limitations = raw_rtk['limitations']
        if '不是真值' not in limitations or 'ATE/RMSE' not in limitations:
            fail('原始RTK图层缺少非真值/非精度评估限制')
        window = raw_rtk['comparison_time_window']
        if window.get('end_timestamp_sec', 0) <= window.get('start_timestamp_sec', 0) or \
                window.get('localization_points_in_window', 0) < 2:
            fail('红色定位与原始RTK没有有效共同时间段')
        if 'rawRtkTrajectory' not in html or '未筛选 RTK 参考（蓝，仅 XY）' not in html or \
                '未筛选 RTK 参考轨迹（蓝，仅 XY）' not in open(overview, 'r').read():
            fail('原始RTK XY轨迹没有同时以正确标签进入Web和静态总览')
        if re.search(r'RTK\s*真值|真值\s*RTK|\bATE\b|\bRMSE\b', html, re.IGNORECASE):
            fail('产品页面不得将原始RTK写为真值或展示ATE/RMSE')
        data = embedded_map_data(html)
        points = data.get('rawRtkTrajectory', {}).get('points')
        if not isinstance(points, list) or len(points) != raw_rtk['point_count']:
            fail('网页内嵌原始RTK XY点数与渲染契约不一致')
        for index, point in enumerate(points):
            if set(point.keys()) != set(('t', 'x', 'y')):
                fail('网页内嵌原始RTK第%d点含有非XY字段' % index)
            if not all(isinstance(point[name], (int, float)) for name in ('t', 'x', 'y')):
                fail('网页内嵌原始RTK第%d点含有非数值XY字段' % index)
    verify_script_delimiters(html)
    print('阶段7产品校验通过：Web和SVG均存在，证据状态策略完整')

if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('阶段7校验失败：%s' % error, file=sys.stderr)
        sys.exit(1)
