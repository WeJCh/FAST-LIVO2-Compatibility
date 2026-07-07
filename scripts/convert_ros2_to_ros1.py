#!/usr/bin/env python3
"""
convert_ros2_to_ros1.py ── ROS2 db3 → ROS1 bag 转换脚本

将机器人狗采集的 ROS2 db3 rosbag 转换为 FAST-LIVO2 兼容的 ROS1 rosbag。

功能:
  /front_lidar                    (sensor_msgs/msg/PointCloud2)    → /livox/lidar    (livox_ros_driver/CustomMsg)
  /front_lidar/imu                (sensor_msgs/msg/Imu)            → /livox/imu      (sensor_msgs/Imu)
  /front_camera/image_compressed  (sensor_msgs/msg/CompressedImage)→ /left_camera/image (sensor_msgs/Image, JPEG→BGR8)

时间戳策略:
  消息 header.stamp 保留传感器原始时间，bag 记录时间保留 ROS2 bag 中的接收时间。
  两者不能混用：Livox 点云 header 是扫描起始时间，而 bag 记录时间通常晚约 0.1 秒。

用法:
  python3 scripts/convert_ros2_to_ros1.py \
    RobotDogDataset/robotdog_submit_20260618_190534/raw_data/sync_20260618_190534/rosbag2_sensor_sync_20260618_190534 \
    output_fast_livo2.bag
"""

import sys
import struct
import argparse
import logging
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# rosbags 导入
# ---------------------------------------------------------------------------
from rosbags.rosbag1 import Writer as Rosbag1Writer
from rosbags.rosbag2 import Reader as Rosbag2Reader
from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.msg import get_types_from_msg


# ===========================================================================
# 核心设计: 双 typestore
#
#   ros2_store (Stores.LATEST) : 用于读取 / 反序列化 ROS2 消息
#   ros1_store (Stores.ROS1_NOETIC) : 用于构造对象 / 序列化为 ROS1 格式
#
# 原因: ROS1 的 std_msgs/Header 包含 uint32 seq 字段, 而 ROS2 不含。
#       若用 ROS2 store 做 ROS1 序列化, 字节偏移会少 4 字节, 导致 ROS1
#       端反序列化报 "Buffer Overrun"。
# ===========================================================================


def build_ros2_store():
    """构建 ROS2 typestore (用于反序列化 ROS2 CDR 消息)."""
    ts = get_typestore(Stores.LATEST)
    return ts


def build_ros1_store():
    """构建 ROS1 NOETIC typestore, 并注册 livox 自定义类型 (用于 ROS1 序列化)."""
    ts = get_typestore(Stores.ROS1_NOETIC)

    # ---------- 注册 livox CustomPoint ----------
    CUSTOMPOINT_DEF = """\
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""
    ts.register(get_types_from_msg(CUSTOMPOINT_DEF, 'livox_ros_driver/CustomPoint'))

    # ---------- 注册 livox CustomMsg ----------
    CUSTOMMSG_DEF = """\
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver/CustomPoint[] points
"""
    ts.register(get_types_from_msg(CUSTOMMSG_DEF, 'livox_ros_driver/CustomMsg'))

    return ts


# ===========================================================================
# 辅助: 从 ROS2 时间戳构造 ROS1 std_msgs/Header
#
# ROS1 Header 字段: uint32 seq, time stamp, string frame_id
# ROS1 time 字段:   uint32 secs, uint32 nsecs
# ===========================================================================


def stamp_to_ns(stamp) -> int:
    """将 ROS2 builtin_interfaces/Time 转为纳秒整数。"""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def make_ros1_header(ros1_store, stamp_ns: int, frame_id: str = "livox_frame"):
    """
    构造 ROS1 std_msgs/Header 对象.
    ros1_store: Stores.ROS1_NOETIC 构建的 typestore
    stamp_ns:   纳秒时间戳

    ROS1 NOETIC 的 std_msgs/Header.stamp 类型是 builtin_interfaces/msg/Time,
    其字段为 sec (int) / nanosec (int), 而非 std_msgs/msg/Time.
    """
    secs = int(stamp_ns // 1_000_000_000)
    nsecs = int(stamp_ns % 1_000_000_000)

    BuiltinTime = ros1_store.types['builtin_interfaces/msg/Time']
    HeaderCls = ros1_store.types['std_msgs/msg/Header']

    stamp = BuiltinTime(sec=secs, nanosec=nsecs)
    return HeaderCls(seq=0, stamp=stamp, frame_id=frame_id)


# ===========================================================================
# 转换函数
# ===========================================================================

def convert_pointcloud2_to_custommsg(ros1_store, pc2_msg):
    """
    将 ROS2 sensor_msgs/PointCloud2 转换为 ROS1 livox_ros_driver/CustomMsg.

    pc2_msg: 已反序列化的 sensor_msgs/msg/PointCloud2 Python 对象 (来自 ROS2 store)
    """
    data_types = {1: "b", 2: "B", 3: "h", 4: "H", 5: "i", 6: "I", 7: "f", 8: "d"}
    size_map = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}

    fields_meta = {}
    for f in pc2_msg.fields:
        fmt = data_types.get(f.datatype)
        sz = size_map.get(f.datatype)
        if fmt is None:
            raise ValueError(f"不支持的数据类型: datatype={f.datatype}")
        fields_meta[f.name] = {"offset": f.offset, "fmt": fmt, "size": sz, "count": f.count}

    required = ["x", "y", "z", "intensity", "timestamp", "tag", "line"]
    missing = [r for r in required if r not in fields_meta]
    if missing:
        raise KeyError(f"PointCloud2 缺少必要字段: {missing}")

    data = memoryview(pc2_msg.data)
    point_step = max(1, pc2_msg.point_step)
    num_points = len(data) // point_step
    if num_points == 0:
        return None

    raw_points = []

    for i in range(num_points):
        off = i * point_step

        def _read(name):
            fi = fields_meta[name]
            return struct.unpack_from(fi["fmt"], data, off + fi["offset"])[0]

        x = float(_read("x"))
        y = float(_read("y"))
        z = float(_read("z"))
        intensity = int(_read("intensity"))
        ts = int(_read("timestamp"))
        tag = int(_read("tag"))
        line = int(_read("line"))

        raw_points.append({
            "x": x, "y": y, "z": z,
            "reflectivity": max(0, min(255, intensity)),
            "timestamp": ts, "tag": tag, "line": line,
        })

    # 本机器狗 PointCloud2 的 timestamp 是相对 header.stamp 的帧内纳秒时间
    # （约 0~100 ms）。CustomMsg 的 timebase 应使用扫描起始绝对时间，不能
    # 使用晚约 0.1 秒的 rosbag 记录时间，否则 LiDAR 会相对 IMU 整体错位。
    lidar_stamp_ns = stamp_to_ns(pc2_msg.header.stamp)
    timebase = lidar_stamp_ns

    # 使用 ROS1 store 的类型类
    CustomPointCls = ros1_store.types['livox_ros_driver/msg/CustomPoint']
    CustomMsgCls = ros1_store.types['livox_ros_driver/msg/CustomMsg']

    custom_points = []
    for p in raw_points:
        # timestamp 已经是相对扫描起点的纳秒偏移，直接写入 offset_time。
        # uint32 可覆盖约 4.29 秒，远大于当前约 0.1 秒的单帧扫描时长。
        offset_time = max(0, min(0xFFFFFFFF, p["timestamp"]))
        cp = CustomPointCls(
            offset_time=np.uint32(offset_time),
            x=float(p["x"]), y=float(p["y"]), z=float(p["z"]),
            reflectivity=np.uint8(p["reflectivity"]),
            tag=np.uint8(p["tag"]), line=np.uint8(p["line"]),
        )
        custom_points.append(cp)

    header = make_ros1_header(
        ros1_store,
        lidar_stamp_ns,
        frame_id=pc2_msg.header.frame_id or "livox_frame",
    )
    msg = CustomMsgCls(
        header=header,
        timebase=timebase,
        point_num=len(custom_points),
        lidar_id=1,
        rsvd=np.zeros(3, dtype=np.uint8),
        points=custom_points,
    )
    return msg


def decompress_image(ros1_store, compressed_msg):
    """
    将 ROS2 sensor_msgs/CompressedImage 解压为 ROS1 sensor_msgs/Image (BGR8).
    compressed_msg: 已反序列化的 ROS2 CompressedImage 对象
    """
    if isinstance(compressed_msg.data, np.ndarray):
        raw_bytes = compressed_msg.data.tobytes()
    elif hasattr(compressed_msg.data, 'tobytes'):
        raw_bytes = bytes(compressed_msg.data)
    else:
        raw_bytes = bytes(compressed_msg.data)

    np_arr = np.frombuffer(raw_bytes, np.uint8)
    bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if bgr is None:
        logging.warning("图像解压失败，跳过该帧")
        return None

    h, w = bgr.shape[:2]
    channels = 3

    img_stamp_ns = int(
        compressed_msg.header.stamp.sec * 1_000_000_000
        + compressed_msg.header.stamp.nanosec
    )
    header = make_ros1_header(ros1_store, img_stamp_ns,
                              frame_id=compressed_msg.header.frame_id or "camera")

    ImageCls = ros1_store.types['sensor_msgs/msg/Image']
    return ImageCls(
        header=header,
        height=h, width=w,
        encoding="bgr8",
        is_bigendian=0,
        step=w * channels,
        data=np.frombuffer(bgr.tobytes(), dtype=np.uint8),
    )


def convert_imu(ros1_store, ros2_imu_msg):
    """
    ROS2 sensor_msgs/Imu → ROS1 sensor_msgs/Imu.

    ros2_imu_msg: 已反序列化的 ROS2 Imu 对象
    注意: 必须从 ROS2 对象提取原始数值, 再用 ROS1 store 构造新对象。
          rosbags 的 Geometry 子对象跨 store 不兼容。
    """
    # IMU 同样保留传感器 header，而不是使用 rosbag 接收时间。当前两者虽仅
    # 相差约 0.5 ms，但统一保留源时间可避免引入额外同步误差。
    imu_stamp_ns = stamp_to_ns(ros2_imu_msg.header.stamp)
    header = make_ros1_header(
        ros1_store,
        imu_stamp_ns,
        frame_id=ros2_imu_msg.header.frame_id or "livox_frame",
    )

    ImuCls = ros1_store.types['sensor_msgs/msg/Imu']
    QuatCls = ros1_store.types['geometry_msgs/msg/Quaternion']
    Vec3Cls = ros1_store.types['geometry_msgs/msg/Vector3']

    # 从 ROS2 对象提取原始值
    ori = ros2_imu_msg.orientation
    ang = ros2_imu_msg.angular_velocity
    lin = ros2_imu_msg.linear_acceleration

    return ImuCls(
        header=header,
        orientation=QuatCls(x=float(ori.x), y=float(ori.y), z=float(ori.z), w=float(ori.w)),
        orientation_covariance=np.array(
            [float(v) for v in ros2_imu_msg.orientation_covariance],
            dtype=np.float64,
        ),
        angular_velocity=Vec3Cls(x=float(ang.x), y=float(ang.y), z=float(ang.z)),
        angular_velocity_covariance=np.array(
            [float(v) for v in ros2_imu_msg.angular_velocity_covariance],
            dtype=np.float64,
        ),
        linear_acceleration=Vec3Cls(x=float(lin.x), y=float(lin.y), z=float(lin.z)),
        linear_acceleration_covariance=np.array(
            [float(v) for v in ros2_imu_msg.linear_acceleration_covariance],
            dtype=np.float64,
        ),
    )


# ===========================================================================
# 主逻辑
# ===========================================================================

TOPIC_MAP = {
    "/front_lidar":                   ("/livox/lidar",       "livox_ros_driver/msg/CustomMsg"),
    "/front_lidar/imu":               ("/livox/imu",          "sensor_msgs/msg/Imu"),
    "/front_camera/image_compressed": ("/left_camera/image",  "sensor_msgs/msg/Image"),
}


def main():
    parser = argparse.ArgumentParser(
        description="ROS2 db3 → ROS1 bag 转换工具 (FAST-LIVO2 适配)"
    )
    parser.add_argument(
        "input_dir",
        help="ROS2 db3 rosbag 目录路径 (包含 metadata.yaml 和 .db3 的文件夹)",
    )
    parser.add_argument("output_bag", help="输出 ROS1 .bag 文件路径")
    parser.add_argument(
        "--skip-images", action="store_true",
        help="跳过图像解压转换（仅转换 LiDAR + IMU）",
    )
    parser.add_argument("--quiet", action="store_true", help="减少日志输出")
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_bag = Path(args.output_bag)

    if not input_dir.is_dir():
        print(f"错误: 输入目录不存在: {input_dir}")
        sys.exit(1)
    if not (input_dir / "metadata.yaml").exists():
        print(f"错误: 输入目录不包含 metadata.yaml，请确认路径正确")
        sys.exit(1)

    if output_bag.exists():
        output_bag.unlink()
        print(f"已删除旧的输出文件: {output_bag}")

    # ------------------------------------------------------------------
    # 构建两个 typestore
    #   ros2_store: 用于读取 ROS2 消息 (deserialize_cdr)
    #   ros1_store: 用于构造 & 序列化 ROS1 消息 (serialize_ros1)
    # ------------------------------------------------------------------
    print("构建 ROS2 typestore (用于读取) ...")
    ros2_store = build_ros2_store()
    print("构建 ROS1 typestore (用于写入) ...")
    ros1_store = build_ros1_store()
    print("typestore 构建完成。\n")

    stats = {"lidar": 0, "imu": 0, "image": 0, "skipped": 0}

    print("=" * 60)
    print("ROS2 db3 → ROS1 bag 转换")
    print("=" * 60)
    print(f"输入: {input_dir}")
    print(f"输出: {output_bag}")
    print(f"跳过图像: {'是' if args.skip_images else '否'}")
    print()
    print("Topic 映射:")
    for old, (new, mtype) in TOPIC_MAP.items():
        note = " (跳过)" if (args.skip_images and "camera" in old) else ""
        print(f"  {old:40s} → {new}  [{mtype}]{note}")
    print("=" * 60)

    with Rosbag2Reader(input_dir) as reader, Rosbag1Writer(output_bag) as writer:
        # 预注册输出连接 (使用 ROS1 typestore 生成正确的 msgdef 和 MD5)
        output_conns = {}
        for orig_topic, (new_topic, msgtype) in TOPIC_MAP.items():
            if args.skip_images and "camera" in orig_topic:
                continue
            if not any(c.topic == orig_topic for c in reader.connections):
                continue
            conn = writer.add_connection(new_topic, msgtype, typestore=ros1_store)
            output_conns[orig_topic] = conn

        total_messages = sum(c.msgcount for c in reader.connections)
        processed = 0

        for connection, bag_timestamp_ns, raw_cdr_bytes in reader.messages():
            processed += 1
            orig_topic = connection.topic

            if orig_topic not in TOPIC_MAP:
                stats["skipped"] += 1
                continue

            if args.skip_images and "camera" in orig_topic:
                stats["skipped"] += 1
                continue

            try:
                # --- 用 ROS2 store 反序列化 ---
                ros2_msg = ros2_store.deserialize_cdr(raw_cdr_bytes, connection.msgtype)

                # --- 转换 (用 ROS1 store 构造新对象) ---
                if orig_topic == "/front_lidar":
                    converted = convert_pointcloud2_to_custommsg(
                        ros1_store, ros2_msg
                    )
                    if converted is None:
                        stats["skipped"] += 1
                        continue

                elif orig_topic == "/front_lidar/imu":
                    converted = convert_imu(ros1_store, ros2_msg)

                elif orig_topic == "/front_camera/image_compressed":
                    converted = decompress_image(ros1_store, ros2_msg)
                    if converted is None:
                        stats["skipped"] += 1
                        continue
                else:
                    stats["skipped"] += 1
                    continue

                # --- 用 ROS1 store 序列化为 ROS1 格式 ---
                ros1_bytes = ros1_store.serialize_ros1(
                    converted, output_conns[orig_topic].msgtype
                )

                # ROS1 bag 的记录时间继续沿用 ROS2 bag 接收时间；消息内部的
                # header.stamp 已分别保留传感器时间。这样回放顺序忠于原始采集，
                # 同时 FAST-LIVO2 能按正确传感器时间完成同步和点云去畸变。
                writer.write(
                    output_conns[orig_topic],
                    bag_timestamp_ns,
                    ros1_bytes,
                )

                if orig_topic == "/front_lidar":
                    stats["lidar"] += 1
                elif orig_topic == "/front_lidar/imu":
                    stats["imu"] += 1
                elif orig_topic == "/front_camera/image_compressed":
                    stats["image"] += 1

                if not args.quiet and processed % 1000 == 0:
                    print(
                        f"  [{processed}/{total_messages}] "
                        f"LiDAR: {stats['lidar']}, IMU: {stats['imu']}, "
                        f"Image: {stats['image']}"
                    )

            except Exception as exc:
                logging.warning(
                    "处理消息失败 topic=%s ts=%d: %s",
                    orig_topic, bag_timestamp_ns, exc,
                )
                stats["skipped"] += 1

    print()
    print("=" * 60)
    print("转换完成!")
    print("=" * 60)
    print(f"  LiDAR 帧 (→ /livox/lidar):         {stats['lidar']:>6}")
    print(f"  IMU 帧  (→ /livox/imu):            {stats['imu']:>6}")
    print(f"  图像帧  (→ /left_camera/image):    {stats['image']:>6}")
    print(f"  跳过:                              {stats['skipped']:>6}")
    print(f"  总计:                              {sum(stats.values()):>6}")
    print()
    print(f"输出文件: {output_bag.absolute()}")
    print(f"文件大小: {output_bag.stat().st_size / (1024**3):.2f} GiB")
    print()
    print("下一步: rosbag play <output_bag> 然后 roslaunch fast_livo2 mapping_avia.launch")


if __name__ == "__main__":
    main()
