#!/usr/bin/env python3
"""提供 OpenVINS 风格的单次轨迹误差评估命令。

该脚本读取 `timestamp px py pz qx qy qz qw` 轨迹文件，按 0.02 秒最近邻
匹配估计轨迹和真值轨迹，并输出 OpenVINS `error_singlerun` 风格的 ATE、
按距离分段 RPE 和 NEES 统计。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
Matrix3 = Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
Transform = Tuple[Matrix3, Vector3]


@dataclass(frozen=True)
class PoseSample:
    """单个位姿采样。

    Args:
        timestamp: 位姿时间戳，单位为秒。
        position: 世界系下的位置 `(x, y, z)`。
        quaternion: 姿态四元数 `(qx, qy, qz, qw)`。
        covariance_orientation: 可选姿态协方差矩阵。
        covariance_position: 可选位置协方差矩阵。
    """

    timestamp: float
    position: Vector3
    quaternion: Quaternion
    covariance_orientation: Optional[Matrix3] = None
    covariance_position: Optional[Matrix3] = None


@dataclass(frozen=True)
class Statistics:
    """标量误差序列的统计量。"""

    count: int = 0
    rmse: float = 0.0
    mean: float = 0.0
    median: float = 0.0
    minimum: float = 0.0
    maximum: float = 0.0
    std: float = 0.0

    @property
    def min(self) -> float:
        """返回最小值，兼容 OpenVINS 输出命名。"""
        return self.minimum

    @property
    def max(self) -> float:
        """返回最大值，兼容 OpenVINS 输出命名。"""
        return self.maximum


MatchedPair = Tuple[PoseSample, PoseSample]
AlignedPair = Tuple[PoseSample, PoseSample]


def read_pose_file(path: Path) -> List[PoseSample]:
    """读取 OpenVINS 空格分隔轨迹文件。

    Args:
        path: 轨迹文件路径，支持 8 列位姿或 20 列位姿加协方差格式。

    Returns:
        轨迹采样列表。

    Raises:
        FileNotFoundError: 输入文件不存在。
        ValueError: 非注释行列数不足或数值解析失败。
    """
    samples: List[PoseSample] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, raw_line in enumerate(handle, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            fields = line.split()
            if len(fields) < 8:
                raise ValueError(f"{path}:{line_number} 至少需要 8 列")
            values = [float(value) for value in fields[:20]]
            covariance_orientation: Optional[Matrix3] = None
            covariance_position: Optional[Matrix3] = None
            if len(values) >= 20:
                covariance_orientation = symmetric_matrix_from_upper(
                    values[8], values[9], values[10], values[11], values[12], values[13]
                )
                covariance_position = symmetric_matrix_from_upper(
                    values[14], values[15], values[16], values[17], values[18], values[19]
                )
            samples.append(
                PoseSample(
                    timestamp=values[0],
                    position=(values[1], values[2], values[3]),
                    quaternion=normalize_quaternion((values[4], values[5], values[6], values[7])),
                    covariance_orientation=covariance_orientation,
                    covariance_position=covariance_position,
                )
            )
    if not samples:
        raise ValueError(f"{path} 未解析到有效轨迹数据")
    return samples


def symmetric_matrix_from_upper(
    c00: float,
    c01: float,
    c02: float,
    c11: float,
    c12: float,
    c22: float,
) -> Matrix3:
    """根据上三角元素构造对称 3x3 矩阵。

    Args:
        c00: 第 0 行第 0 列元素。
        c01: 第 0 行第 1 列元素。
        c02: 第 0 行第 2 列元素。
        c11: 第 1 行第 1 列元素。
        c12: 第 1 行第 2 列元素。
        c22: 第 2 行第 2 列元素。

    Returns:
        对称矩阵。
    """
    return ((c00, c01, c02), (c01, c11, c12), (c02, c12, c22))


def match_by_timestamp(
    estimates: Sequence[PoseSample],
    groundtruth: Sequence[PoseSample],
    max_dt: float = 0.02,
) -> List[MatchedPair]:
    """按最近时间戳匹配估计轨迹和真值轨迹。

    Args:
        estimates: 估计轨迹采样。
        groundtruth: 真值轨迹采样。
        max_dt: 最大允许时间差，单位为秒。

    Returns:
        满足阈值的 `(estimate, groundtruth)` 匹配对。
    """
    pairs: List[MatchedPair] = []
    gt_index = 0
    gt_count = len(groundtruth)
    for estimate in estimates:
        while (
            gt_index + 1 < gt_count
            and abs(groundtruth[gt_index + 1].timestamp - estimate.timestamp)
            <= abs(groundtruth[gt_index].timestamp - estimate.timestamp)
        ):
            gt_index += 1
        if gt_count > 0 and abs(groundtruth[gt_index].timestamp - estimate.timestamp) <= max_dt:
            pairs.append((estimate, groundtruth[gt_index]))
    return pairs


def align_posyaw_umeyama(pairs: Sequence[MatchedPair]) -> List[AlignedPair]:
    """使用 yaw-only、known-scale Umeyama 对齐估计轨迹到真值轨迹。

    Args:
        pairs: 时间戳匹配后的轨迹对。

    Returns:
        估计位姿已对齐到真值坐标系的轨迹对。

    Raises:
        ValueError: 匹配数量不足或轨迹退化，无法估计 yaw。
    """
    if len(pairs) < 2:
        raise ValueError("posyaw 对齐至少需要 2 个匹配位姿")

    estimate_mean = mean_position([estimate.position for estimate, _ in pairs])
    truth_mean = mean_position([truth.position for _, truth in pairs])
    correlation_cos = 0.0
    correlation_sin = 0.0
    for estimate, truth in pairs:
        estimate_centered = subtract_vector(estimate.position, estimate_mean)
        truth_centered = subtract_vector(truth.position, truth_mean)
        correlation_cos += (
            estimate_centered[0] * truth_centered[0]
            + estimate_centered[1] * truth_centered[1]
        )
        correlation_sin += (
            estimate_centered[0] * truth_centered[1]
            - estimate_centered[1] * truth_centered[0]
        )
    if abs(correlation_cos) < 1e-12 and abs(correlation_sin) < 1e-12:
        raise ValueError("轨迹水平位移不足，无法估计 yaw 对齐")

    yaw = math.atan2(correlation_sin, correlation_cos)
    rotation_quaternion = yaw_quaternion(yaw)
    rotated_estimate_mean = rotate_yaw(estimate_mean, yaw)
    translation = subtract_vector(truth_mean, rotated_estimate_mean)

    aligned: List[AlignedPair] = []
    for estimate, truth in pairs:
        aligned_position = add_vector(rotate_yaw(estimate.position, yaw), translation)
        aligned_quaternion = normalize_quaternion(
            quat_multiply(estimate.quaternion, quat_inverse(rotation_quaternion))
        )
        aligned.append(
            (
                PoseSample(
                    timestamp=estimate.timestamp,
                    position=aligned_position,
                    quaternion=aligned_quaternion,
                    covariance_orientation=estimate.covariance_orientation,
                    covariance_position=estimate.covariance_position,
                ),
                truth,
            )
        )
    return aligned


def compute_ate(aligned_pairs: Sequence[AlignedPair]) -> dict:
    """计算绝对轨迹误差。

    Args:
        aligned_pairs: 对齐后的估计和真值位姿对。

    Returns:
        包含姿态角度误差和位置误差统计的字典。
    """
    orientation_errors = [
        orientation_error_degrees(estimate.quaternion, truth.quaternion)
        for estimate, truth in aligned_pairs
    ]
    position_errors = [
        vector_norm(subtract_vector(estimate.position, truth.position))
        for estimate, truth in aligned_pairs
    ]
    return {"ori": summarize_errors(orientation_errors), "pos": summarize_errors(position_errors)}


def compute_rpe(
    aligned_pairs: Sequence[AlignedPair],
    segments: Sequence[float] = (8.0, 16.0, 24.0, 32.0, 40.0),
) -> dict:
    """按固定真值距离片段计算相对位姿误差。

    Args:
        aligned_pairs: 对齐后的估计和真值位姿对。
        segments: 需要统计的片段长度，单位为米。

    Returns:
        距离片段到姿态/位置误差统计的映射。
    """
    if not aligned_pairs:
        raise ValueError("没有可用于 RPE 的匹配位姿")

    accumulated_distances = compute_accumulated_distances([truth for _, truth in aligned_pairs])
    rpe = {}
    for segment in segments:
        orientation_errors: List[float] = []
        position_errors: List[float] = []
        comparison_indices = compute_comparison_indices_length(
            accumulated_distances, segment, max_dist_diff=0.5
        )
        for start_index, end_index in enumerate(comparison_indices):
            if end_index == -1:
                continue
            estimate_start, truth_start = aligned_pairs[start_index]
            estimate_end, truth_end = aligned_pairs[end_index]
            estimate_delta = relative_transform_matrix(estimate_start, estimate_end)
            truth_delta = relative_transform_matrix(truth_start, truth_end)
            error_delta = compose_transform_matrix(invert_transform_matrix(truth_delta), estimate_delta)
            endpoint_rotation = pose_body_to_world_rotation(estimate_end)
            orientation_error = matrix_multiply(
                matrix_multiply(endpoint_rotation, error_delta[0]),
                matrix_transpose(endpoint_rotation),
            )
            position_error = matrix_vector_multiply(endpoint_rotation, error_delta[1])
            orientation_errors.append(math.degrees(vector_norm(log_so3(orientation_error))))
            position_errors.append(vector_norm(position_error))
        rpe[float(segment)] = {
            "ori": summarize_errors(orientation_errors),
            "pos": summarize_errors(position_errors),
        }
    return rpe


def compute_nees(aligned_pairs: Sequence[AlignedPair]) -> dict:
    """计算 NEES，若无协方差则输出 OpenVINS 风格警告。

    Args:
        aligned_pairs: 对齐后的估计和真值位姿对。

    Returns:
        包含姿态和位置 NEES 统计的字典。
    """
    has_covariance = all(
        estimate.covariance_orientation is not None and estimate.covariance_position is not None
        for estimate, _ in aligned_pairs
    )
    if not aligned_pairs or not has_covariance:
        print(
            "[TRAJ]: Normalized Estimation Error Squared called but trajectory does not have any covariances..."
        )
        print("[TRAJ]: Did you record using a Odometry or PoseWithCovarianceStamped????")
        return {"ori": Statistics(), "pos": Statistics()}

    orientation_nees: List[float] = []
    position_nees: List[float] = []
    for estimate, truth in aligned_pairs:
        orientation_error = rotation_vector_between(estimate.quaternion, truth.quaternion)
        position_error = subtract_vector(truth.position, estimate.position)
        orientation_nees.append(
            quadratic_form(
                orientation_error,
                invert_matrix3(require_matrix(estimate.covariance_orientation)),
            )
        )
        position_nees.append(
            quadratic_form(position_error, invert_matrix3(require_matrix(estimate.covariance_position)))
        )
    return {"ori": summarize_errors(orientation_nees), "pos": summarize_errors(position_nees)}


def format_report(ate: dict, rpe: dict, nees: dict) -> str:
    """格式化 OpenVINS 风格报告。

    Args:
        ate: ATE 统计结果。
        rpe: RPE 统计结果。
        nees: NEES 统计结果。

    Returns:
        可打印报告文本。
    """
    lines = [
        "Setting printing level to: INFO",
        "======================================",
        "Absolute Trajectory Error",
        "======================================",
        format_pair("rmse", ate["ori"].rmse, ate["pos"].rmse),
        format_pair("mean", ate["ori"].mean, ate["pos"].mean),
        format_pair("min", ate["ori"].min, ate["pos"].min),
        format_pair("max", ate["ori"].max, ate["pos"].max),
        format_pair("std", ate["ori"].std, ate["pos"].std),
        "======================================",
        "Relative Pose Error",
        "======================================",
    ]
    for segment in sorted(rpe):
        segment_stats = rpe[segment]
        lines.append(
            f"seg {int(segment)} - median_ori = {segment_stats['ori'].median:.3f} | "
            f"median_pos = {segment_stats['pos'].median:.3f} "
            f"({segment_stats['pos'].count} samples)"
        )
    lines.extend(
        [
            "======================================",
            "Normalized Estimation Error Squared",
            "======================================",
            format_pair("mean", nees["ori"].mean, nees["pos"].mean),
            format_pair("min", nees["ori"].min, nees["pos"].min),
            format_pair("max", nees["ori"].max, nees["pos"].max),
            format_pair("std", nees["ori"].std, nees["pos"].std),
            "======================================",
        ]
    )
    return "\n".join(lines)


def format_pair(name: str, orientation_value: float, position_value: float) -> str:
    """格式化一行姿态和位置统计。

    Args:
        name: 统计项名称。
        orientation_value: 姿态统计值。
        position_value: 位置统计值。

    Returns:
        格式化后的单行文本。
    """
    orientation_label = f"{name}_ori"
    position_label = f"{name}_pos"
    return f"{orientation_label:<9}= {orientation_value:.3f} | {position_label:<9}= {position_value:.3f}"


def summarize_errors(errors: Sequence[float]) -> Statistics:
    """汇总标量误差序列。

    Args:
        errors: 标量误差序列。

    Returns:
        统计量；空序列返回全 0。
    """
    if not errors:
        return Statistics()
    sorted_errors = sorted(errors)
    count = len(sorted_errors)
    middle = count // 2
    if count % 2 == 1:
        median = sorted_errors[middle]
    else:
        median = 0.5 * (sorted_errors[middle - 1] + sorted_errors[middle])
    mean = sum(sorted_errors) / count
    rmse = math.sqrt(sum(error * error for error in sorted_errors) / count)
    if count > 1:
        std = math.sqrt(sum((error - mean) ** 2 for error in sorted_errors) / (count - 1))
    else:
        std = 0.0
    return Statistics(
        count=count,
        rmse=rmse,
        mean=mean,
        median=median,
        minimum=sorted_errors[0],
        maximum=sorted_errors[-1],
        std=std,
    )


def compute_accumulated_distances(samples: Sequence[PoseSample]) -> List[float]:
    """计算真值轨迹累计里程。

    Args:
        samples: 位姿采样。

    Returns:
        与输入等长的累计距离数组。
    """
    distances = [0.0 for _ in samples]
    for index in range(1, len(samples)):
        step = vector_norm(subtract_vector(samples[index].position, samples[index - 1].position))
        distances[index] = distances[index - 1] + step
    return distances


def compute_comparison_indices_length(
    distances: Sequence[float],
    distance: float,
    max_dist_diff: float,
) -> List[int]:
    """查找每个起点对应的固定距离片段终点。

    Args:
        distances: 累计里程。
        distance: 目标片段长度。
        max_dist_diff: 允许的距离误差。

    Returns:
        每个起点对应的终点索引，找不到时为 `-1`。
    """
    comparison_indices: List[int] = []
    for start_index, start_distance in enumerate(distances):
        best_index = -1
        best_difference = max_dist_diff
        for end_index in range(start_index + 1, len(distances)):
            current_difference = abs((distances[end_index] - start_distance) - distance)
            if current_difference <= best_difference:
                best_difference = current_difference
                best_index = end_index
            elif distances[end_index] - start_distance > distance + max_dist_diff:
                break
        comparison_indices.append(best_index)
    return comparison_indices


def relative_transform(start: PoseSample, end: PoseSample) -> Tuple[Quaternion, Vector3]:
    """计算从起点位姿到终点位姿的相对变换。

    Args:
        start: 起点位姿。
        end: 终点位姿。

    Returns:
        相对旋转四元数和相对平移。
    """
    inverse_start = invert_transform((start.quaternion, start.position))
    return compose_transform(inverse_start, (end.quaternion, end.position))


def relative_transform_matrix(start: PoseSample, end: PoseSample) -> Transform:
    """计算 OpenVINS/JPL SE(3) 相对变换。

    Args:
        start: 起点位姿。
        end: 终点位姿。

    Returns:
        从起点 body 坐标系到终点 body 坐标系的相对变换。
    """
    start_transform = pose_transform_body_to_world(start)
    end_transform = pose_transform_body_to_world(end)
    return compose_transform_matrix(invert_transform_matrix(start_transform), end_transform)


def invert_transform(transform: Tuple[Quaternion, Vector3]) -> Tuple[Quaternion, Vector3]:
    """求 SE(3) 变换逆。

    Args:
        transform: 旋转和平移。

    Returns:
        逆变换。
    """
    rotation, translation = transform
    inverse_rotation = quat_inverse(rotation)
    inverse_translation = rotate_vector(inverse_rotation, scale_vector(translation, -1.0))
    return inverse_rotation, inverse_translation


def invert_transform_matrix(transform: Transform) -> Transform:
    """求矩阵形式 SE(3) 变换逆。

    Args:
        transform: 旋转矩阵和平移。

    Returns:
        逆变换。
    """
    rotation, translation = transform
    inverse_rotation = matrix_transpose(rotation)
    inverse_translation = matrix_vector_multiply(inverse_rotation, scale_vector(translation, -1.0))
    return inverse_rotation, inverse_translation


def compose_transform(
    left: Tuple[Quaternion, Vector3],
    right: Tuple[Quaternion, Vector3],
) -> Tuple[Quaternion, Vector3]:
    """组合两个 SE(3) 变换。

    Args:
        left: 左侧变换。
        right: 右侧变换。

    Returns:
        组合后的变换。
    """
    rotation = normalize_quaternion(quat_multiply(left[0], right[0]))
    translation = add_vector(rotate_vector(left[0], right[1]), left[1])
    return rotation, translation


def compose_transform_matrix(left: Transform, right: Transform) -> Transform:
    """组合两个矩阵形式 SE(3) 变换。

    Args:
        left: 左侧变换。
        right: 右侧变换。

    Returns:
        组合后的变换。
    """
    rotation = matrix_multiply(left[0], right[0])
    translation = add_vector(matrix_vector_multiply(left[0], right[1]), left[1])
    return rotation, translation


def pose_transform_body_to_world(sample: PoseSample) -> Transform:
    """把位姿采样转为 OpenVINS 使用的 body-to-world SE(3)。

    Args:
        sample: 位姿采样。

    Returns:
        旋转和平移组成的 SE(3) 变换。
    """
    return pose_body_to_world_rotation(sample), sample.position


def pose_body_to_world_rotation(sample: PoseSample) -> Matrix3:
    """根据 JPL 四元数计算 body-to-world 旋转矩阵。

    Args:
        sample: 位姿采样。

    Returns:
        body-to-world 旋转矩阵。
    """
    return matrix_transpose(quat_to_rot_jpl(sample.quaternion))


def mean_position(positions: Sequence[Vector3]) -> Vector3:
    """计算位置均值。

    Args:
        positions: 位置序列。

    Returns:
        均值位置。
    """
    count = float(len(positions))
    return (
        sum(position[0] for position in positions) / count,
        sum(position[1] for position in positions) / count,
        sum(position[2] for position in positions) / count,
    )


def rotate_yaw(position: Vector3, yaw: float) -> Vector3:
    """绕 Z 轴旋转三维位置。

    Args:
        position: 原始位置。
        yaw: 旋转角，单位为弧度。

    Returns:
        旋转后位置。
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * position[0] - sin_yaw * position[1],
        sin_yaw * position[0] + cos_yaw * position[1],
        position[2],
    )


def yaw_quaternion(yaw: float) -> Quaternion:
    """构造绕 Z 轴旋转的四元数。

    Args:
        yaw: yaw 角，单位为弧度。

    Returns:
        四元数 `(qx, qy, qz, qw)`。
    """
    return rot_to_quat_jpl(rotation_z(yaw))


def rotation_z(yaw: float) -> Matrix3:
    """构造绕 Z 轴旋转矩阵。

    Args:
        yaw: yaw 角，单位为弧度。

    Returns:
        3x3 旋转矩阵。
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        (cos_yaw, -sin_yaw, 0.0),
        (sin_yaw, cos_yaw, 0.0),
        (0.0, 0.0, 1.0),
    )


def rot_to_quat_jpl(rotation: Matrix3) -> Quaternion:
    """复刻 OpenVINS `rot_2_quat()` 的 JPL 四元数转换。

    Args:
        rotation: 3x3 旋转矩阵。

    Returns:
        JPL 四元数 `(qx, qy, qz, qw)`。
    """
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if rotation[0][0] >= trace and rotation[0][0] >= rotation[1][1] and rotation[0][0] >= rotation[2][2]:
        qx = math.sqrt((1.0 + 2.0 * rotation[0][0] - trace) / 4.0)
        qy = (rotation[0][1] + rotation[1][0]) / (4.0 * qx)
        qz = (rotation[0][2] + rotation[2][0]) / (4.0 * qx)
        qw = (rotation[1][2] - rotation[2][1]) / (4.0 * qx)
    elif rotation[1][1] >= trace and rotation[1][1] >= rotation[0][0] and rotation[1][1] >= rotation[2][2]:
        qy = math.sqrt((1.0 + 2.0 * rotation[1][1] - trace) / 4.0)
        qx = (rotation[0][1] + rotation[1][0]) / (4.0 * qy)
        qz = (rotation[1][2] + rotation[2][1]) / (4.0 * qy)
        qw = (rotation[2][0] - rotation[0][2]) / (4.0 * qy)
    elif rotation[2][2] >= trace and rotation[2][2] >= rotation[0][0] and rotation[2][2] >= rotation[1][1]:
        qz = math.sqrt((1.0 + 2.0 * rotation[2][2] - trace) / 4.0)
        qx = (rotation[0][2] + rotation[2][0]) / (4.0 * qz)
        qy = (rotation[1][2] + rotation[2][1]) / (4.0 * qz)
        qw = (rotation[0][1] - rotation[1][0]) / (4.0 * qz)
    else:
        qw = math.sqrt((1.0 + trace) / 4.0)
        qx = (rotation[1][2] - rotation[2][1]) / (4.0 * qw)
        qy = (rotation[2][0] - rotation[0][2]) / (4.0 * qw)
        qz = (rotation[0][1] - rotation[1][0]) / (4.0 * qw)
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    return normalize_quaternion((qx, qy, qz, qw))


def quat_to_rot_jpl(quaternion: Quaternion) -> Matrix3:
    """复刻 OpenVINS `quat_2_Rot()` 的 JPL 旋转矩阵转换。

    Args:
        quaternion: JPL 四元数 `(qx, qy, qz, qw)`。

    Returns:
        3x3 旋转矩阵。
    """
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    skew = skew_symmetric((qx, qy, qz))
    identity = identity_matrix3()
    vector = (qx, qy, qz)
    return tuple(
        tuple(
            (2.0 * qw * qw - 1.0) * identity[row][column]
            - 2.0 * qw * skew[row][column]
            + 2.0 * vector[row] * vector[column]
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    """归一化四元数。

    Args:
        quaternion: 输入四元数。

    Returns:
        单位四元数。
    """
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm == 0.0:
        raise ValueError("四元数范数不能为 0")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def quat_inverse(quaternion: Quaternion) -> Quaternion:
    """计算单位四元数逆。

    Args:
        quaternion: 单位四元数。

    Returns:
        逆四元数。
    """
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    return (-qx, -qy, -qz, qw)


def quat_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    """计算 OpenVINS/JPL 四元数乘法。

    Args:
        left: 左四元数。
        right: 右四元数。

    Returns:
        乘积四元数。
    """
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    product = (
        lw * rx + rw * lx - ly * rz + lz * ry,
        lw * ry + rw * ly - lz * rx + lx * rz,
        lw * rz + rw * lz - lx * ry + ly * rx,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )
    qx, qy, qz, qw = normalize_quaternion(product)
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    return (qx, qy, qz, qw)


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """用四元数旋转向量。

    Args:
        quaternion: 旋转四元数。
        vector: 三维向量。

    Returns:
        旋转后的向量。
    """
    return matrix_vector_multiply(quat_to_rot_jpl(quaternion), vector)


def quaternion_angle_degrees(left: Quaternion, right: Quaternion) -> float:
    """计算两个姿态之间的最短旋转角。

    Args:
        left: 第一个四元数。
        right: 第二个四元数。

    Returns:
        角度误差，单位为度。
    """
    return orientation_error_degrees(left, right)


def orientation_error_degrees(estimate: Quaternion, truth: Quaternion) -> float:
    """按 OpenVINS ATE 公式计算姿态误差角。

    Args:
        estimate: 估计姿态四元数。
        truth: 真值姿态四元数。

    Returns:
        姿态误差角，单位为度。
    """
    error_rotation = matrix_multiply(
        matrix_transpose(quat_to_rot_jpl(estimate)),
        quat_to_rot_jpl(truth),
    )
    return math.degrees(vector_norm(log_so3(error_rotation)))


def quaternion_rotation_angle_degrees(quaternion: Quaternion) -> float:
    """计算四元数对应的最短旋转角。

    Args:
        quaternion: 旋转四元数。

    Returns:
        旋转角，单位为度。
    """
    return math.degrees(vector_norm(log_so3(quat_to_rot_jpl(quaternion))))


def rotation_vector_between(left: Quaternion, right: Quaternion) -> Vector3:
    """计算两个四元数之间的旋转向量误差。

    Args:
        left: 第一个四元数。
        right: 第二个四元数。

    Returns:
        旋转向量，单位为弧度。
    """
    error_rotation = matrix_multiply(
        matrix_transpose(quat_to_rot_jpl(left)),
        quat_to_rot_jpl(right),
    )
    return log_so3(error_rotation)


def log_so3(rotation: Matrix3) -> Vector3:
    """复刻 OpenVINS `log_so3()` 的旋转矩阵对数映射。

    Args:
        rotation: 3x3 旋转矩阵。

    Returns:
        so(3) 旋转向量，单位为弧度。
    """
    r11, r12, r13 = rotation[0]
    r21, r22, r23 = rotation[1]
    r31, r32, r33 = rotation[2]
    trace = r11 + r22 + r33
    if trace + 1.0 < 1e-10:
        if abs(r33 + 1.0) > 1e-5:
            scale = math.pi / math.sqrt(2.0 + 2.0 * r33)
            return (scale * r13, scale * r23, scale * (1.0 + r33))
        if abs(r22 + 1.0) > 1e-5:
            scale = math.pi / math.sqrt(2.0 + 2.0 * r22)
            return (scale * r12, scale * (1.0 + r22), scale * r32)
        scale = math.pi / math.sqrt(2.0 + 2.0 * r11)
        return (scale * (1.0 + r11), scale * r21, scale * r31)

    trace_minus_three = trace - 3.0
    if trace_minus_three < -1e-7:
        theta = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
        magnitude = theta / (2.0 * math.sin(theta))
    else:
        magnitude = 0.5 - trace_minus_three / 12.0
    return (
        magnitude * (r32 - r23),
        magnitude * (r13 - r31),
        magnitude * (r21 - r12),
    )


def identity_matrix3() -> Matrix3:
    """返回 3x3 单位矩阵。"""
    return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def skew_symmetric(vector: Vector3) -> Matrix3:
    """构造三维向量的反对称矩阵。

    Args:
        vector: 三维向量。

    Returns:
        反对称矩阵。
    """
    return (
        (0.0, -vector[2], vector[1]),
        (vector[2], 0.0, -vector[0]),
        (-vector[1], vector[0], 0.0),
    )


def matrix_transpose(matrix: Matrix3) -> Matrix3:
    """计算 3x3 矩阵转置。"""
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))  # type: ignore[return-value]


def matrix_multiply(left: Matrix3, right: Matrix3) -> Matrix3:
    """计算 3x3 矩阵乘法。"""
    return tuple(
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )  # type: ignore[return-value]


def matrix_vector_multiply(matrix: Matrix3, vector: Vector3) -> Vector3:
    """计算 3x3 矩阵与三维向量乘法。"""
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def invert_matrix3(matrix: Matrix3) -> Matrix3:
    """计算 3x3 矩阵逆。

    Args:
        matrix: 输入矩阵。

    Returns:
        逆矩阵。

    Raises:
        ValueError: 矩阵奇异。
    """
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    determinant = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(determinant) < 1e-18:
        raise ValueError("协方差矩阵不可逆")
    inverse_determinant = 1.0 / determinant
    return (
        (
            (e * i - f * h) * inverse_determinant,
            (c * h - b * i) * inverse_determinant,
            (b * f - c * e) * inverse_determinant,
        ),
        (
            (f * g - d * i) * inverse_determinant,
            (a * i - c * g) * inverse_determinant,
            (c * d - a * f) * inverse_determinant,
        ),
        (
            (d * h - e * g) * inverse_determinant,
            (b * g - a * h) * inverse_determinant,
            (a * e - b * d) * inverse_determinant,
        ),
    )


def quadratic_form(vector: Vector3, matrix: Matrix3) -> float:
    """计算 `vector^T matrix vector`。

    Args:
        vector: 三维向量。
        matrix: 3x3 矩阵。

    Returns:
        二次型结果。
    """
    product = (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )
    return vector[0] * product[0] + vector[1] * product[1] + vector[2] * product[2]


def require_matrix(matrix: Optional[Matrix3]) -> Matrix3:
    """把可选矩阵转为必选矩阵。

    Args:
        matrix: 可选矩阵。

    Returns:
        输入矩阵。

    Raises:
        ValueError: 输入为空。
    """
    if matrix is None:
        raise ValueError("缺少协方差矩阵")
    return matrix


def add_vector(left: Vector3, right: Vector3) -> Vector3:
    """计算向量加法。"""
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def subtract_vector(left: Vector3, right: Vector3) -> Vector3:
    """计算向量减法。"""
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def scale_vector(vector: Vector3, scale: float) -> Vector3:
    """计算向量数乘。"""
    return (scale * vector[0], scale * vector[1], scale * vector[2])


def vector_norm(vector: Vector3) -> float:
    """计算三维向量范数。"""
    return math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])


def build_argument_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。

    Returns:
        参数解析器。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("align_mode", choices=["posyaw"], help="对齐模式，目前仅支持 posyaw")
    parser.add_argument("groundtruth", type=Path, help="真值轨迹文件")
    parser.add_argument("estimate", type=Path, help="估计轨迹文件")
    return parser


def evaluate(align_mode: str, groundtruth_path: Path, estimate_path: Path) -> str:
    """执行单次轨迹评估。

    Args:
        align_mode: 对齐模式，目前仅支持 `posyaw`。
        groundtruth_path: 真值轨迹文件路径。
        estimate_path: 估计轨迹文件路径。

    Returns:
        OpenVINS 风格报告文本。

    Raises:
        ValueError: 对齐模式不支持或轨迹匹配不足。
    """
    if align_mode != "posyaw":
        raise ValueError("当前仅支持 posyaw 对齐模式")
    groundtruth = read_pose_file(groundtruth_path)
    estimates = read_pose_file(estimate_path)
    pairs = match_by_timestamp(estimates, groundtruth)
    if len(pairs) < 3:
        raise ValueError("匹配位姿不足，无法计算误差")
    aligned_pairs = align_posyaw_umeyama(pairs)
    ate = compute_ate(aligned_pairs)
    rpe = compute_rpe(aligned_pairs)
    nees = compute_nees(aligned_pairs)
    return format_report(ate, rpe, nees)


def main() -> int:
    """执行命令行入口。

    Returns:
        进程退出码，0 表示成功。
    """
    args = build_argument_parser().parse_args()
    print(evaluate(args.align_mode, args.groundtruth, args.estimate))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
