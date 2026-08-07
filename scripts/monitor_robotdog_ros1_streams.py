#!/usr/bin/env python3
"""无落盘地核验机器狗 ROS1 回放输入及 FAST-LIVO2 里程计覆盖范围。

该脚本只订阅消息头，不保存点云数据。用于区分 ROS2->ROS1 播放器没有
发布完整 IMU，和 FAST-LIVO2 虽收到完整输入但停止发布里程计这两种情况。
"""

import argparse
import threading

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


class StreamStats:
    """记录一个话题的消息数量与 header.stamp 覆盖范围。"""

    def __init__(self, name):
        self.name = name
        self.count = 0
        self.first_stamp = None
        self.last_stamp = None
        self.non_monotonic = 0

    def add(self, stamp):
        value = stamp.to_sec()
        if self.first_stamp is None:
            self.first_stamp = value
        elif value < self.last_stamp:
            self.non_monotonic += 1
        self.last_stamp = value
        self.count += 1

    def summary(self):
        if self.first_stamp is None:
            return "%s: count=0" % self.name
        return (
            "%s: count=%d, start=%.3f, end=%.3f, span=%.3fs, non_monotonic=%d"
            % (self.name, self.count, self.first_stamp, self.last_stamp,
               self.last_stamp - self.first_stamp, self.non_monotonic)
        )


class Monitor:
    """ROS 回调只做常数时间统计，避免监控器本身造成消息堆积。"""

    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.lidar = StreamStats("LiDAR")
        self.imu = StreamStats("IMU")
        self.odom = StreamStats("/aft_mapped_to_init")
        self.lidar_target_reached_at = None

        # 大队列仅用于离线回放，防止监控器成为验证过程中的瓶颈。
        rospy.Subscriber(args.lidar_topic, PointCloud2, self.on_lidar,
                         queue_size=2000, tcp_nodelay=True)
        rospy.Subscriber(args.imu_topic, Imu, self.on_imu,
                         queue_size=20000, tcp_nodelay=True)
        rospy.Subscriber(args.odom_topic, Odometry, self.on_odom,
                         queue_size=2000, tcp_nodelay=True)

    def on_lidar(self, msg):
        with self.lock:
            self.lidar.add(msg.header.stamp)
            # 以起止时间筛选 ROS2 bag 时，首末 LiDAR 帧通常不会恰好落在
            # duration 的两个边界上；留 0.25 秒容差避免 60 秒片段停在 59.9 秒。
            if (self.args.source_duration > 0 and
                    self.lidar.first_stamp is not None and
                    self.lidar.last_stamp - self.lidar.first_stamp >= self.args.source_duration - 0.25 and
                    self.lidar_target_reached_at is None):
                self.lidar_target_reached_at = rospy.Time.now()

    def on_imu(self, msg):
        with self.lock:
            self.imu.add(msg.header.stamp)

    def on_odom(self, msg):
        with self.lock:
            self.odom.add(msg.header.stamp)

    def print_summary(self):
        with self.lock:
            rospy.loginfo("[stream-monitor] %s", self.lidar.summary())
            rospy.loginfo("[stream-monitor] %s", self.imu.summary())
            rospy.loginfo("[stream-monitor] %s", self.odom.summary())

    def ready_to_finish(self):
        with self.lock:
            if self.lidar_target_reached_at is None:
                return False
            return (rospy.Time.now() - self.lidar_target_reached_at).to_sec() >= self.args.settle_wall_sec


def parse_args():
    parser = argparse.ArgumentParser(description="无落盘机器狗 ROS1 输入/里程计覆盖监控")
    parser.add_argument("--lidar-topic", default="/front_lidar")
    parser.add_argument("--imu-topic", default="/front_lidar/imu")
    parser.add_argument("--odom-topic", default="/aft_mapped_to_init")
    parser.add_argument("--source-duration", type=float, default=60.0,
                        help="达到此 LiDAR header 时间跨度后，等待 settle-wall-sec 并自动输出结果")
    parser.add_argument("--settle-wall-sec", type=float, default=5.0,
                        help="LiDAR 到达目标跨度后继续等待里程计输出的实际时间")
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("robotdog_ros1_stream_monitor", anonymous=True)
    monitor = Monitor(args)
    rospy.loginfo("[stream-monitor] monitoring LiDAR, IMU and odometry without recording data")

    rate = rospy.Rate(2.0)
    try:
        while not rospy.is_shutdown():
            if monitor.ready_to_finish():
                break
            rate.sleep()
    finally:
        monitor.print_summary()


if __name__ == "__main__":
    main()
