#!/usr/bin/env python3
"""评估 Voxel-SVIO 在 EuRoC Vicon Room 数据集上的轨迹误差。

该脚本读取 Voxel-SVIO 输出的估计轨迹和 EuRoC/OpenVINS 格式真值轨迹，
执行时间戳最近邻匹配、pos+yaw 初始对齐，并输出 ATE 与 RPE 统计。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class PoseSample:
    """单个位姿采样。

    Args:
        timestamp: 位姿时间戳，单位为秒。
        position: 世界系下的位置 `(x, y, z)`。
        quaternion: 姿态四元数 `(qx, qy, qz, qw)`。
    """

    timestamp: float
    position: Tuple[float, float, float]
    quaternion: Tuple[float, float, float, float]


MatchedPair = Tuple[PoseSample, PoseSample]
AlignedPair = Tuple[PoseSample, PoseSample]


def read_pose_file(path: Path) -> List[PoseSample]:
    """读取 TUM/EuRoC 风格轨迹文件。

    Args:
        path: 轨迹文件路径，列格式为 `timestamp px py pz qx qy qz qw`。

    Returns:
        按文件顺序排列的位姿列表。

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
            values = [float(value) for value in fields[:8]]
            samples.append(
                PoseSample(
                    timestamp=values[0],
                    position=(values[1], values[2], values[3]),
                    quaternion=(values[4], values[5], values[6], values[7]),
                )
            )
    return samples


def match_by_timestamp(
    estimates: Sequence[PoseSample],
    groundtruth: Sequence[PoseSample],
    max_dt: float,
) -> List[MatchedPair]:
    """按最近时间戳匹配估计轨迹和真值轨迹。

    Args:
        estimates: 估计轨迹采样。
        groundtruth: 真值轨迹采样。
        max_dt: 允许的最大时间差，单位为秒。

    Returns:
        满足时间差阈值的 `(estimate, groundtruth)` 匹配对。
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


def rotate_yaw(position: Tuple[float, float, float], yaw: float) -> Tuple[float, float, float]:
    """绕 Z 轴旋转三维位置。

    Args:
        position: 原始位置。
        yaw: 旋转角，单位为弧度。

    Returns:
        旋转后的位置。
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x, y, z = position
    return (cos_yaw * x - sin_yaw * y, sin_yaw * x + cos_yaw * y, z)


def planar_distance(left: PoseSample, right: PoseSample) -> float:
    """计算两个位姿在 XY 平面的距离。

    Args:
        left: 第一个位姿。
        right: 第二个位姿。

    Returns:
        XY 平面欧氏距离。
    """
    dx = right.position[0] - left.position[0]
    dy = right.position[1] - left.position[1]
    return math.hypot(dx, dy)


def planar_yaw_between(start: PoseSample, end: PoseSample) -> float:
    """根据两个位置点计算 XY 平面运动方向 yaw。

    Args:
        start: 起始位姿。
        end: 终止位姿。

    Returns:
        从起始点指向终止点的 yaw 角，单位为弧度。
    """
    dx = end.position[0] - start.position[0]
    dy = end.position[1] - start.position[1]
    return math.atan2(dy, dx)


def align_estimates_to_groundtruth(pairs: Sequence[MatchedPair]) -> List[AlignedPair]:
    """按起点位置和起步段方向执行 pos+yaw 对齐。

    Args:
        pairs: 时间戳匹配后的估计和真值位姿对。

    Returns:
        估计轨迹已对齐到真值坐标系后的位姿对。

    Raises:
        ValueError: 输入匹配为空。
    """
    if not pairs:
        raise ValueError("没有可用于对齐的匹配位姿")

    min_alignment_distance = 0.05
    first_estimate, first_groundtruth = pairs[0]
    direction_pair = None
    for estimate, truth in pairs[1:]:
        if (
            planar_distance(first_estimate, estimate) >= min_alignment_distance
            and planar_distance(first_groundtruth, truth) >= min_alignment_distance
        ):
            direction_pair = (estimate, truth)
            break

    if direction_pair is None:
        raise ValueError("轨迹位移不足，无法进行 yaw 对齐")

    direction_estimate, direction_groundtruth = direction_pair
    yaw_delta = planar_yaw_between(first_groundtruth, direction_groundtruth) - planar_yaw_between(
        first_estimate, direction_estimate
    )
    first_rotated = rotate_yaw(first_estimate.position, yaw_delta)
    translation = tuple(
        first_groundtruth.position[index] - first_rotated[index] for index in range(3)
    )

    aligned: List[AlignedPair] = []
    for estimate, truth in pairs:
        rotated = rotate_yaw(estimate.position, yaw_delta)
        aligned_position = tuple(rotated[index] + translation[index] for index in range(3))
        aligned_estimate = PoseSample(
            timestamp=estimate.timestamp,
            position=aligned_position,
            quaternion=estimate.quaternion,
        )
        aligned.append((aligned_estimate, truth))
    return aligned


def euclidean_error(left: Tuple[float, float, float], right: Tuple[float, float, float]) -> float:
    """计算两个三维位置的欧氏距离。

    Args:
        left: 第一个位置。
        right: 第二个位置。

    Returns:
        欧氏距离。
    """
    return math.sqrt(sum((left[index] - right[index]) ** 2 for index in range(3)))


def summarize_errors(errors: Sequence[float]) -> dict:
    """汇总误差序列。

    Args:
        errors: 误差值序列。

    Returns:
        包含数量、RMSE、均值、中位数和最大值的字典。

    Raises:
        ValueError: 误差序列为空。
    """
    if not errors:
        raise ValueError("没有误差样本")
    sorted_errors = sorted(errors)
    mid = len(sorted_errors) // 2
    if len(sorted_errors) % 2 == 0:
        median = 0.5 * (sorted_errors[mid - 1] + sorted_errors[mid])
    else:
        median = sorted_errors[mid]
    return {
        "count": len(errors),
        "rmse": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "mean": sum(errors) / len(errors),
        "median": median,
        "max": max(errors),
    }


def compute_ate(aligned_pairs: Sequence[AlignedPair]) -> dict:
    """计算绝对轨迹误差。

    Args:
        aligned_pairs: 对齐后的估计和真值位姿对。

    Returns:
        ATE 统计结果。
    """
    errors = [
        euclidean_error(estimate.position, truth.position) for estimate, truth in aligned_pairs
    ]
    return summarize_errors(errors)


def compute_rpe(aligned_pairs: Sequence[AlignedPair], step: int) -> dict:
    """计算固定采样间隔的相对位置误差。

    Args:
        aligned_pairs: 对齐后的估计和真值位姿对。
        step: 用于计算相对运动的采样间隔。

    Returns:
        RPE 统计结果。

    Raises:
        ValueError: 采样不足或 step 非法。
    """
    if step <= 0:
        raise ValueError("RPE step 必须为正数")
    errors: List[float] = []
    for index in range(0, len(aligned_pairs) - step):
        estimate_a, truth_a = aligned_pairs[index]
        estimate_b, truth_b = aligned_pairs[index + step]
        estimate_delta = tuple(
            estimate_b.position[axis] - estimate_a.position[axis] for axis in range(3)
        )
        truth_delta = tuple(truth_b.position[axis] - truth_a.position[axis] for axis in range(3))
        errors.append(euclidean_error(estimate_delta, truth_delta))
    return summarize_errors(errors)


def format_summary(name: str, summary: dict) -> str:
    """格式化单组误差统计。

    Args:
        name: 统计项名称。
        summary: 统计结果字典。

    Returns:
        可打印的多行文本。
    """
    return (
        f"{name}\n"
        f"  count  = {summary['count']}\n"
        f"  rmse   = {summary['rmse']:.6f}\n"
        f"  mean   = {summary['mean']:.6f}\n"
        f"  median = {summary['median']:.6f}\n"
        f"  max    = {summary['max']:.6f}"
    )


def write_aligned_trajectory(aligned_pairs: Iterable[AlignedPair], output_path: Path) -> None:
    """写出对齐后的估计轨迹。

    Args:
        aligned_pairs: 对齐后的估计和真值位姿对。
        output_path: 输出文件路径。
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as handle:
        handle.write("# timestamp px py pz qx qy qz qw\n")
        for estimate, _ in aligned_pairs:
            px, py, pz = estimate.position
            qx, qy, qz, qw = estimate.quaternion
            handle.write(
                f"{estimate.timestamp:.9f} {px:.9f} {py:.9f} {pz:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )


def build_argument_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。

    Returns:
        参数解析器。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimate", required=True, type=Path, help="估计轨迹文件")
    parser.add_argument("--groundtruth", required=True, type=Path, help="真值轨迹文件")
    parser.add_argument("--output", type=Path, default=None, help="误差报告输出路径")
    parser.add_argument(
        "--aligned-output",
        type=Path,
        default=None,
        help="对齐后的估计轨迹输出路径",
    )
    parser.add_argument("--max-dt", type=float, default=0.02, help="时间匹配最大差值，单位秒")
    parser.add_argument("--rpe-step", type=int, default=10, help="RPE 采样间隔")
    return parser


def main() -> int:
    """执行命令行评估流程。

    Returns:
        进程退出码，0 表示成功。
    """
    args = build_argument_parser().parse_args()
    estimates = read_pose_file(args.estimate)
    groundtruth = read_pose_file(args.groundtruth)
    pairs = match_by_timestamp(estimates, groundtruth, args.max_dt)
    aligned_pairs = align_estimates_to_groundtruth(pairs)
    ate = compute_ate(aligned_pairs)

    lines = [format_summary("Absolute Trajectory Error", ate)]
    if len(aligned_pairs) > args.rpe_step:
        lines.append(format_summary(f"Relative Pose Error step={args.rpe_step}", compute_rpe(aligned_pairs, args.rpe_step)))
    else:
        lines.append(f"Relative Pose Error step={args.rpe_step}\n  skipped = matched samples too few")

    report = "\n\n".join(lines)
    print(report)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report + "\n", encoding="utf-8")
    if args.aligned_output:
        write_aligned_trajectory(aligned_pairs, args.aligned_output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
