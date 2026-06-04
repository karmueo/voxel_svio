#!/usr/bin/env python3
"""发布与 Voxel-SVIO 估计轨迹对齐的 EuRoC 真值轨迹。

该节点读取 EuRoC/OpenVINS 格式真值文件，订阅估计轨迹首帧后执行 pos+yaw
对齐，并按估计轨迹当前时间逐步发布 RViz2 可显示的 `nav_msgs/msg/Path`
真值轨迹。
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from evaluate_euroc_trajectory import (
    PoseSample,
    planar_distance,
    planar_yaw_between,
    read_pose_file,
    rotate_yaw,
)


def stamp_to_seconds(stamp) -> float:
    """将 ROS2 时间戳转换为秒。

    Args:
        stamp: ROS2 消息时间戳。

    Returns:
        秒级浮点时间。
    """
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def nearest_groundtruth_pose(groundtruth: List[PoseSample], timestamp: float) -> PoseSample:
    """查找距离指定时间戳最近的真值位姿。

    Args:
        groundtruth: 真值位姿列表。
        timestamp: 估计位姿时间戳。

    Returns:
        最近的真值位姿。

    Raises:
        ValueError: 真值列表为空。
    """
    if not groundtruth:
        raise ValueError("真值轨迹为空")
    return min(groundtruth, key=lambda sample: abs(sample.timestamp - timestamp))


def align_groundtruth_to_estimate(
    groundtruth: List[PoseSample],
    estimate_anchor: PoseSample,
    truth_anchor: PoseSample,
    estimate_direction: PoseSample,
    truth_direction: PoseSample,
) -> List[PoseSample]:
    """将真值轨迹按起点和起步段方向对齐到估计轨迹坐标系。

    Args:
        groundtruth: 原始真值轨迹。
        estimate_anchor: 用于对齐的估计位姿。
        truth_anchor: 与估计位姿时间最近的真值位姿。
        estimate_direction: 用于估计 VIO 起步方向的位姿。
        truth_direction: 用于估计真值起步方向的位姿。

    Returns:
        对齐后的真值轨迹。
    """
    yaw_delta = planar_yaw_between(estimate_anchor, estimate_direction) - planar_yaw_between(
        truth_anchor, truth_direction
    )
    rotated_anchor = rotate_yaw(truth_anchor.position, yaw_delta)
    translation = tuple(
        estimate_anchor.position[index] - rotated_anchor[index] for index in range(3)
    )

    aligned: List[PoseSample] = []
    for sample in groundtruth:
        rotated = rotate_yaw(sample.position, yaw_delta)
        position = tuple(rotated[index] + translation[index] for index in range(3))
        aligned.append(PoseSample(sample.timestamp, position, sample.quaternion))
    return aligned


def truncate_groundtruth_by_time(
    groundtruth: List[PoseSample],
    latest_timestamp: float,
) -> List[PoseSample]:
    """按估计轨迹当前时间截取真值轨迹前缀。

    Args:
        groundtruth: 已经对齐到估计坐标系的真值轨迹。
        latest_timestamp: 估计轨迹当前最新时间戳。

    Returns:
        时间不晚于估计轨迹最新时间的真值前缀。若估计时间早于首个真值时间，
        返回首个真值点，避免 RViz2 Path 长时间为空。
    """
    if not groundtruth:
        return []

    visible_groundtruth: List[PoseSample] = []
    for sample in groundtruth:
        if sample.timestamp > latest_timestamp:
            break
        visible_groundtruth.append(sample)

    if not visible_groundtruth:
        return [groundtruth[0]]
    return visible_groundtruth


class GroundtruthPathPublisher(Node):
    """发布对齐真值轨迹的 ROS2 节点。"""

    def __init__(self) -> None:
        """初始化参数、订阅者和发布者。"""
        super().__init__("groundtruth_path_publisher")
        self.declare_parameter("groundtruth_path", "")
        self.declare_parameter("estimate_path_topic", "/vio/path")
        self.declare_parameter("output_topic", "/vio/groundtruth_path")
        self.declare_parameter("frame_id", "camera_init")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("alignment_min_distance", 0.05)

        groundtruth_path = Path(
            self.get_parameter("groundtruth_path").get_parameter_value().string_value
        )
        if not groundtruth_path.is_file():
            raise FileNotFoundError(f"真值文件不存在: {groundtruth_path}")

        self.groundtruth = read_pose_file(groundtruth_path)
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        estimate_path_topic = (
            self.get_parameter("estimate_path_topic").get_parameter_value().string_value
        )
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.alignment_min_distance = (
            self.get_parameter("alignment_min_distance").get_parameter_value().double_value
        )

        self.aligned_groundtruth: List[PoseSample] = []
        self.latest_estimate_timestamp: float | None = None
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(PathMsg, output_topic, path_qos)
        self.subscription = self.create_subscription(
            PathMsg, estimate_path_topic, self.estimate_path_callback, 10
        )
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 0.1), self.publish_path)

    def estimate_path_callback(self, msg: PathMsg) -> None:
        """接收估计轨迹并在首次有效输入时完成真值对齐。

        Args:
            msg: 估计轨迹消息。
        """
        if not msg.poses:
            return

        latest_pose = msg.poses[-1]
        self.latest_estimate_timestamp = stamp_to_seconds(latest_pose.header.stamp)

        if self.aligned_groundtruth:
            self.publish_path()
            return

        if len(msg.poses) < 2:
            return

        anchor_pose = msg.poses[0]
        direction_pose = msg.poses[-1]
        estimate_anchor = PoseSample(
            timestamp=stamp_to_seconds(anchor_pose.header.stamp),
            position=(
                anchor_pose.pose.position.x,
                anchor_pose.pose.position.y,
                anchor_pose.pose.position.z,
            ),
            quaternion=(
                anchor_pose.pose.orientation.x,
                anchor_pose.pose.orientation.y,
                anchor_pose.pose.orientation.z,
                anchor_pose.pose.orientation.w,
            ),
        )
        estimate_direction = PoseSample(
            timestamp=stamp_to_seconds(direction_pose.header.stamp),
            position=(
                direction_pose.pose.position.x,
                direction_pose.pose.position.y,
                direction_pose.pose.position.z,
            ),
            quaternion=(
                direction_pose.pose.orientation.x,
                direction_pose.pose.orientation.y,
                direction_pose.pose.orientation.z,
                direction_pose.pose.orientation.w,
            ),
        )
        truth_anchor = nearest_groundtruth_pose(self.groundtruth, estimate_anchor.timestamp)
        truth_direction = nearest_groundtruth_pose(self.groundtruth, estimate_direction.timestamp)
        if (
            planar_distance(estimate_anchor, estimate_direction) < self.alignment_min_distance
            or planar_distance(truth_anchor, truth_direction) < self.alignment_min_distance
        ):
            return

        self.aligned_groundtruth = align_groundtruth_to_estimate(
            self.groundtruth,
            estimate_anchor,
            truth_anchor,
            estimate_direction,
            truth_direction,
        )
        self.get_logger().info(
            f"groundtruth aligned with timestamp dt="
            f"{abs(truth_anchor.timestamp - estimate_anchor.timestamp):.6f}s"
        )
        self.publish_path()

    def publish_path(self) -> None:
        """发布对齐后的真值轨迹。"""
        if not self.aligned_groundtruth:
            return

        if self.latest_estimate_timestamp is None:
            return

        visible_groundtruth = truncate_groundtruth_by_time(
            self.aligned_groundtruth, self.latest_estimate_timestamp
        )

        path_msg = PathMsg()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        path_msg.poses = [self.to_pose_stamped(sample) for sample in visible_groundtruth]
        self.publisher.publish(path_msg)

    def to_pose_stamped(self, sample: PoseSample) -> PoseStamped:
        """将位姿采样转换为 PoseStamped。

        Args:
            sample: 对齐后的真值位姿采样。

        Returns:
            ROS2 位姿消息。
        """
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.header.stamp.sec = int(sample.timestamp)
        pose_msg.header.stamp.nanosec = int((sample.timestamp - int(sample.timestamp)) * 1e9)
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = (
            sample.position
        )
        (
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ) = sample.quaternion
        return pose_msg

def main() -> None:
    """启动真值轨迹发布节点。"""
    rclpy.init()
    node = GroundtruthPathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
